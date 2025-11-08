import zmq  
import threading  
import time  
import json
import numpy as np  
import torch  
import cv2  
from operating_platform.robot.robots.utils import RobotDeviceNotConnectedError
from operating_platform.robot.robots.demo_robot.robot_config import DemoRobotRobotConfig
from operating_platform.robot.robots.com_configs.cameras import CameraConfig, OpenCVCameraConfig ,IntelRealSenseCameraConfig
from operating_platform.robot.robots.camera import Camera
  
ipc_address = "ipc:///tmp/dora-robot"   
context = zmq.Context()  
socket = context.socket(zmq.PAIR)  
socket.connect(ipc_address)  
socket.setsockopt(zmq.RCVTIMEO, 300)  

  
recv_follower = {}  
recv_images = {}
lock = threading.Lock()  

def recv_server(key_dict):
    
    while True:
        try:
            message_parts = socket.recv_multipart()
            if len(message_parts) < 1:
                continue

            event_id = message_parts[0].decode('utf-8')
            buffer_bytes = message_parts[1]

            # 解析可选 metadata
            metadata = {}
            if len(message_parts) >= 3:
                try:
                    metadata = json.loads(message_parts[2].decode('utf-8'))
                except Exception:
                    metadata = {}

            # 遍历映射表，动态判断 event_id 属于哪类数据
            for key,value in key_dict.items():
                if key in event_id:
                    data = np.frombuffer(buffer_bytes, dtype=np.float32)
                    with lock:
                        recv_follower[event_id] = data
                    break  
            
            if 'image' in event_id:
                # 解码图像
                if 'depth' not in event_id:
                    img_array = np.frombuffer(buffer_bytes, dtype=np.uint8)
                    frame = img_array.reshape((480, 640, 3))  # 已经是 RGB 格式 
                    if frame is not None:
                        with lock:
                            #print(f"Received event_id = {event_id}")
                            recv_images[event_id] = frame
                elif 'depth' in event_id: 
                    print(len(buffer_bytes))
                    depth_array = np.frombuffer(buffer_bytes, dtype=np.uint16)
                    depth_frame = depth_array.reshape((480, 640))  # 已经是 RGB 格式
                    if depth_frame is not None:
                        # 排除无效值（可选，根据传感器特性，例如0或65535可能是无效深度）
                        valid_mask = (depth_frame > 0) & (depth_frame < 65535)
                        if np.any(valid_mask):
                            # 仅对有效区域归一化，避免无效值干扰
                            valid_depth = depth_frame[valid_mask]
                            min_val = valid_depth.min()
                            max_val = valid_depth.max()
                        else:
                            # 若全无效，强制范围为0~65535
                            min_val, max_val = 0, 65535

                        depth_16bit = cv2.normalize(
                        depth_frame,
                        None,
                        alpha=0,
                        beta=65535,
                        norm_type=cv2.NORM_MINMAX,
                        dtype=cv2.CV_16U  # 转换为16位无符号整数
                        )
                        rgb_depth = cv2.cvtColor(depth_16bit, cv2.COLOR_GRAY2RGB)  # 扩展维度到(480,640,3)以实现保存
                        # 存储归一化后的图像用于显示
                        with lock:
                            # print(f"Received event_id = {event_id}")
                            recv_images[event_id] = rgb_depth
        except Exception as e:
            print(f"[pose_recv_server] Error: {e}")
            continue




class OpenCVCamera:
    def __init__(self, config: OpenCVCameraConfig):
        self.config = config
        self.camera_index = config.camera_index
        self.port = None
    
        
        self.capture_width = config.width
        self.capture_height = config.height

        
        if config.rotation in [-90, 90]:
            self.width = config.height
            self.height = config.width
        else:
            self.width = config.width
            self.height = config.height

        self.fps = config.fps
        self.channels = config.channels
        self.color_mode = config.color_mode
        self.mock = config.mock

        self.camera = None
        self.is_connected = False
        self.thread = None
        self.stop_event = None
        self.color_image = None
        self.logs = {}

class IntelRealSenseCamera:  
    def __init__(self, config: IntelRealSenseCameraConfig):  
        self.config = config  
        self.name = config.name  
        self.serial_number = config.serial_number  
        self.port = None  
    
        self.capture_width = config.width  
        self.capture_height = config.height  
    
        if config.rotation in [-90, 90]:  
            self.width = config.height  
            self.height = config.width  
        else:  
            self.width = config.width  
            self.height = config.height  
  
        self.fps = config.fps  
        self.channels = config.channels  
        self.color_mode = config.color_mode  
        self.use_depth = config.use_depth  
        self.force_hardware_reset = config.force_hardware_reset  
        self.mock = config.mock  
  
        self.camera = None  
        self.pipeline = None    
        self.is_connected = False  
        self.thread = None  
        self.stop_event = None  
        self.color_image = None  
        self.depth_image = None    
        self.logs = {}

def make_cameras_from_configs(camera_configs: dict[str, CameraConfig]) -> list[Camera]:
    cameras = {}

    for key, cfg in camera_configs.items():
        if cfg.type == "opencv":
            cameras[key] = OpenCVCamera(cfg)
        elif cfg.type == "intelrealsense":  
            cameras[key] = IntelRealSenseCamera(cfg)
        else:
            raise ValueError(f"The camera type '{cfg.type}' is not valid.")
    return cameras

class DemoRobotManipulator:  
    def __init__(self, config: DemoRobotRobotConfig):  
        self.config = config  
        self.robot_type = self.config.type  
        
        self.use_videos = self.config.use_videos
        
        self.microphones = self.config.microphones
        
        self.cameras =  make_cameras_from_configs(self.config.cameras)

        self.follower_arms = {
            name: motor_cfg.motors
            for name, motor_cfg in self.config.follower_motors.items()
            if hasattr(motor_cfg, "motors")
        }
        self.key_dict={name: len(motors) for name, motors in self.follower_arms.items()}

        pose_recv_thread = threading.Thread(target=recv_server(self.key_dict), daemon=True)  
        pose_recv_thread.start()   

        
        self.is_connected = False 
        self.logs = {}
        self.frame_counter = 0

    def get_motor_names(self, motor_groups: dict[str, dict]) -> list:   
        return [f"{group}_{motor}" for group, motors in motor_groups.items() for motor in motors]    
  
    @property    
    def camera_features(self) -> dict:   
        cam_ft = {}  
        for cam_key, cam in self.cameras.items():  
            key = f"observation.images.{cam_key}"  
            cam_ft[key] = {  
                "shape": (cam.height, cam.width, cam.channels),  
                "names": ["height", "width", "channels"],  
                "info": None,  
            }  
        return cam_ft 
  
    @property    
    def motor_features(self) -> dict:  
        motor_groups = {**self.follower_arms}  
        action_names = self.get_motor_names(motor_groups)  
        state_names = self.get_motor_names(motor_groups)  
        
        return {  
            "action": {    
                "dtype": "float32",     
                "shape": (len(action_names),),    
                "names": action_names,    
            },  
            "observation.state": {    
                "dtype": "float32",    
                "shape": (len(state_names),),    
                "names": state_names,    
            }
        } 
  
    @property  
    def features(self):  
        return {**self.motor_features, **self.camera_features}  
  
    def connect(self):  
        timeout = 10 
        start_time = time.perf_counter()  
        expected_keys = list(self.key_dict.keys())
        while True:  
            follower_arm_ready = all(key in recv_follower for key in expected_keys)  
            image_ready = 'image' in recv_images  
            print(follower_arm_ready,image_ready)
            if image_ready and follower_arm_ready:  
                break  
  
            if time.perf_counter() - start_time > timeout:  
                raise TimeoutError("设备连接超时")  
  
            time.sleep(0.1)  
  
        self.is_connected = True  
  
    
    def teleop_step(
        self, record_data=False, 
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:

        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "Aloha is not connected. You need to run `robot.connect()`."
            )

        if not record_data:
            return
        
        # print(self.follower_arms)

        follower_joint = {}
        for name in self.follower_arms:
            # print(name)
            for match_name in recv_follower:
                # print(match_name)
                dim=self.key_dict[match_name]
                if name in match_name:
                    now = time.perf_counter()
                    # print(1)
                    byte_array = np.zeros(dim, dtype=np.float32)
                    pose_read = recv_follower[match_name]

                    byte_array[:dim] = pose_read[:]
                    byte_array = np.round(byte_array, 3)
                    
                    follower_joint[name] = torch.from_numpy(byte_array)
                    # print(follower_joint)

                    self.logs[f"read_follower_{name}_get_dt_s"] = time.perf_counter() - now
                    break
            


        state = follower_joint
        action = follower_joint

        state_array = np.concatenate(list(state.values()))
        action_array = np.concatenate(list(action.values()))


        
        # Capture images from cameras
        images = {}
        for name in self.cameras:
            now = time.perf_counter()
            images[name] = recv_images[name]
            images[name] = torch.from_numpy(images[name])
            self.logs[f"read_camera_{name}_dt_s"] = time.perf_counter() - now

        # Populate output dictionnaries and format to pytorch
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = state_array
        action_dict["action"] = action_array


        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = images[name]

        return obs_dict, action_dict
  
    def disconnect(self):  
        self.is_connected = False
