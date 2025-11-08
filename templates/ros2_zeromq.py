#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from xbot_common_interfaces.msg import ServoPose, HybridJointCommand, ServoPoseStamped

from array import array
from copy import deepcopy
import numpy as np
import zmq
import json
import time
import threading

IPC_ADDRESS = "ipc:///tmp/ros2-zeromq-joint-image-base"

class WrapperPublisher:
    def __init__(self, node: Node):
        self.node = node

        self.raw_sub = node.create_subscription(
            ServoPose,
            "/get_pose",
            self.on_get_pose,
            10
        )
        self.wrapped_pub = node.create_publisher(
            ServoPoseStamped,
            "/get_pose_with_header",
            10
        )

        self.raw_sub_ = node.create_subscription(
            ServoPose,
            "/servo_poses",
            self.on_servo_pose,
            10
        )
        self.wrapped_pub_ = node.create_publisher(
            ServoPoseStamped,
            "/servo_poses_with_header",
            10
        )

    def on_get_pose(self, msg: ServoPose):
        wrapped = ServoPoseStamped()
        wrapped.servo = msg

        # 统一使用 left_pose 时间戳作为主同步时钟
        wrapped.header.stamp = msg.left_pose.header.stamp

        self.wrapped_pub.publish(wrapped)
    
    def on_servo_pose(self, msg: ServoPose):
        wrapped = ServoPoseStamped()
        wrapped.servo = msg

        # 统一使用 left_pose 时间戳作为主同步时钟
        wrapped.header.stamp = msg.left_pose.header.stamp

        self.wrapped_pub_.publish(wrapped)

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_zeromq_bridge')

        self.publisher_arm_joint = self.create_publisher(
            HybridJointCommand,
            '/wr1_controller/commands',
            10)
        
        self.publisher_hand_joint = self.create_publisher(
            HybridJointCommand,
            '/hand_controller/commands',
            10)
        self.wrapper_publish = WrapperPublisher(self)
        self.latest_arm_cmd = None
        self.latest_hand_cmd = None

        self.create_subscription(HybridJointCommand, '/wr1_controller/commands', self.arm_cmd_callback, 10)
        self.create_subscription(HybridJointCommand, '/hand_controller/commands', self.hand_cmd_callback, 10)

    
        
        self.arm_msg_template = self.init_arm_command()
        self.hand_msg_template = self.init_hand_command()


        # self.publisher_base = self.create_publisher(
        #     TwistStamped,
        #     '/motion_target/target_base',
        #     10)

        print(1)
        self._context = zmq.Context()
        self.socket = self._context.socket(zmq.PAIR)
        self.socket.bind(IPC_ADDRESS)
        self.socket.setsockopt(zmq.SNDHWM, 2000)
        self.socket.setsockopt(zmq.SNDBUF, 2**25)
        self.socket.setsockopt(zmq.SNDTIMEO, 2000)
        self.socket.setsockopt(zmq.RCVTIMEO, 2000)
        self.socket.setsockopt(zmq.LINGER, 0)
        
        self.min_interval_ns = 1e9 / 30
        self.last_send_time_ns = 0
        self.frame_count = 0

        self._init_message_synchronizer()


        self.running = True
        self.zeromq_thread = threading.Thread(target=self.zeromq_receive_loop)
        self.zeromq_thread.daemon = True
        self.zeromq_thread.start()

        self.get_logger().info("Joint + Image + Base 同步节点已启动")


    def init_arm_command(self):
        msg = HybridJointCommand()
        msg.joint_name = [
            "ankle_joint", "knee_joint", "hip_joint", "waist_yaw_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_arm_yaw_joint",
            "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint",
            "left_wrist_roll_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_arm_yaw_joint", "right_elbow_pitch_joint", "right_elbow_yaw_joint",
            "right_wrist_pitch_joint", "right_wrist_roll_joint"
        ]
        msg.feedforward = [0.0] * 18
        msg.kp = [1500.0, 1500.0, 1500.0, 500.0] + [500.0] * 14
        msg.kd = [40.0, 40.0, 40.0, 15.0] + [15.0] * 14
        return msg

    def init_hand_command(self):
        msg = HybridJointCommand()
        msg.joint_name = [
            "left_hand_thumb_bend_joint", "left_hand_thumb_rota_joint1", "left_hand_thumb_rota_joint2",
            "left_hand_index_bend_joint", "left_hand_index_joint1", "left_hand_index_joint2",
            "left_hand_mid_joint1", "left_hand_mid_joint2", "left_hand_ring_joint1", "left_hand_ring_joint2",
            "left_hand_pinky_joint1", "left_hand_pinky_joint2",
            "right_hand_thumb_bend_joint", "right_hand_thumb_rota_joint1", "right_hand_thumb_rota_joint2",
            "right_hand_index_bend_joint", "right_hand_index_joint1", "right_hand_index_joint2",
            "right_hand_mid_joint1", "right_hand_mid_joint2", "right_hand_ring_joint1", "right_hand_ring_joint2",
            "right_hand_pinky_joint1", "right_hand_pinky_joint2"
        ]
        msg.feedforward = [350.0] * 24
        msg.kp = [100.0] * 24
        msg.kd = [0.0] * 24
        msg.velocity = [0.0] * 24
        return msg

    def arm_cmd_callback(self, msg):
        self.latest_arm_cmd = msg

    def hand_cmd_callback(self, msg):
        self.latest_hand_cmd = msg


    def _init_message_synchronizer(self):
        self.sub_joint = Subscriber(self, JointState, '/joint_states')
        self.sub_image = Subscriber(self, Image, '/camera/camera/color/image_raw')
        # self.sub_base  = Subscriber(self, Odometry, '/wr1_base_drive_controller/odom')
        self.sub_image_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.sub_get_pose = Subscriber(self, ServoPoseStamped, '/get_pose_with_header')
        self.sub_servo_poses = Subscriber(self, ServoPoseStamped, '/servo_poses_with_header')
        print(2)
        self.sync = ApproximateTimeSynchronizer(
            # [self.sub_joint, self.sub_image, self.sub_base,self.sub_image_depth],
            [self.sub_joint, self.sub_image,self.sub_image_depth,self.sub_get_pose,self.sub_servo_poses],
            queue_size=10,
            slop=0.04,
            allow_headerless=True
        )
        print(3)
        self.sync.registerCallback(self.sync_callback)
        print(4)

    def sync_callback(self, joint_msg,image_msg, image_depth_msg,get_pose_msg , servo_poses_msg):
        try:
            current_time_ns = time.time_ns()
            # print((current_time_ns - self.last_send_time_ns) / 1000000)
            # if (current_time_ns - self.last_send_time_ns) < self.min_interval_ns:
            #    return
            self.last_send_time_ns = current_time_ns

            # print(5)

            joint_data = np.array(joint_msg.position, dtype=np.float32)

            left_arm_index = [6, 8, 9, 10, 12, 28, 46],
            right_arm_index =  [2, 3, 4, 5, 7, 30, 32],
            left_hand_index =  [11, 14, 15, 16, 22, 23, 33, 35, 36, 37, 39, 44],
            right_hand_index = [17, 18, 19, 20, 21, 24, 26, 27, 29, 31, 38, 40],
            other_joints_index = [0, 1, 13, 25, 34, 41, 42, 43, 45, 47]
            
            left_finger_data=joint_data[left_hand_index]
            right_finger_data=joint_data[right_hand_index]
            left_arm_data = joint_data[left_arm_index]
            right_arm_data = joint_data[right_arm_index]
            other_joints_data = joint_data[other_joints_index]
            # print(joint_data)
            
            img_bytes = image_msg.data
            img_meta = {
                "encoding": "bgr8",
                "width": 640,
                "height": 480,
                "timestamp": image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
            }

            # base_data = np.array([
            #     base_msg.pose.pose.position.x,
            #     base_msg.pose.pose.position.y,
            #     base_msg.pose.pose.position.z,
            #     base_msg.pose.pose.orientation.x,
            #     base_msg.pose.pose.orientation.y,
            #     base_msg.pose.pose.orientation.z,
            #     base_msg.pose.pose.orientation.w,
            #     base_msg.twist.twist.linear.x,
            #     base_msg.twist.twist.linear.y,
            #     base_msg.twist.twist.linear.z,
            #     base_msg.twist.twist.angular.x,
            #     base_msg.twist.twist.angular.y,
            #     base_msg.twist.twist.angular.z
            # ], dtype=np.float32)
            # print(base_data)

            img_depth_bytes = image_depth_msg.data
            img_depth_meta = {
                "encoding": "bgr8",
                "width": 640,
                "height": 480,
                "timestamp": image_depth_msg.header.stamp.sec + image_depth_msg.header.stamp.nanosec * 1e-9
            }

            get_pose_data = np.array([
                get_pose_msg.servo.left_pose.pose.position.x,
                get_pose_msg.servo.left_pose.pose.position.y,
                get_pose_msg.servo.left_pose.pose.position.z,
                get_pose_msg.servo.left_pose.pose.orientation.x,
                get_pose_msg.servo.left_pose.pose.orientation.y,
                get_pose_msg.servo.left_pose.pose.orientation.z,
                get_pose_msg.servo.left_pose.pose.orientation.w,
                get_pose_msg.servo.right_pose.pose.position.x,
                get_pose_msg.servo.right_pose.pose.position.y,
                get_pose_msg.servo.right_pose.pose.position.z,
                get_pose_msg.servo.right_pose.pose.orientation.x,
                get_pose_msg.servo.right_pose.pose.orientation.y,
                get_pose_msg.servo.right_pose.pose.orientation.z,
                get_pose_msg.servo.right_pose.pose.orientation.w,
                get_pose_msg.servo.head_pose.pose.position.x,
                get_pose_msg.servo.head_pose.pose.position.y,
                get_pose_msg.servo.head_pose.pose.position.z,
                get_pose_msg.servo.head_pose.pose.orientation.x,
                get_pose_msg.servo.head_pose.pose.orientation.y,
                get_pose_msg.servo.head_pose.pose.orientation.z,
                get_pose_msg.servo.head_pose.pose.orientation.w,
            ], dtype=np.float32)

            servo_poses_data = np.array([
                servo_poses_msg.servo.left_pose.pose.position.x,
                servo_poses_msg.servo.left_pose.pose.position.y,
                servo_poses_msg.servo.left_pose.pose.position.z,
                servo_poses_msg.servo.left_pose.pose.orientation.x,
                servo_poses_msg.servo.left_pose.pose.orientation.y,
                servo_poses_msg.servo.left_pose.pose.orientation.z,
                servo_poses_msg.servo.left_pose.pose.orientation.w,
                servo_poses_msg.servo.right_pose.pose.position.x,
                servo_poses_msg.servo.right_pose.pose.position.y,
                servo_poses_msg.servo.right_pose.pose.position.z,
                servo_poses_msg.servo.right_pose.pose.orientation.x,
                servo_poses_msg.servo.right_pose.pose.orientation.y,
                servo_poses_msg.servo.right_pose.pose.orientation.z,
                servo_poses_msg.servo.right_pose.pose.orientation.w,
                servo_poses_msg.servo.head_pose.pose.position.x,
                servo_poses_msg.servo.head_pose.pose.position.y,
                servo_poses_msg.servo.head_pose.pose.position.z,
                servo_poses_msg.servo.head_pose.pose.orientation.x,
                servo_poses_msg.servo.head_pose.pose.orientation.y,
                servo_poses_msg.servo.head_pose.pose.orientation.z,
                servo_poses_msg.servo.head_pose.pose.orientation.w,
            ], dtype=np.float32)
            # print(servo_poses_data)


            
            # self.send_zmq_event("base_state", base_data)
            self.send_zmq_event("ego_image", img_bytes, img_meta)
            self.send_zmq_event("ego_image_depth", img_depth_bytes, img_depth_meta)
            self.send_zmq_event("left_hand_observation", get_pose_data[:7])
            self.send_zmq_event("right_hand_observation", get_pose_data[7:14])
            self.send_zmq_event("head_observation", get_pose_data[14:])
            self.send_zmq_event("left_hand_action", servo_poses_data[:7])
            self.send_zmq_event("right_hand_action", servo_poses_data[7:14])
            self.send_zmq_event("head_action", servo_poses_data[14:])
            self.send_zmq_event("left_finger", left_finger_data)
            self.send_zmq_event("right_finger", right_finger_data)
            self.send_zmq_event("left_arm", left_arm_data)
            self.send_zmq_event("right_arm", right_arm_data)
            self.send_zmq_event("task_joints", other_joints_data)


        except Exception as e:
                self.get_logger().error(f"同步回调出错: {e}")

    def send_zmq_event(self, event_id, data_bytes, meta=None):
        """发送单条 ZeroMQ 消息，带 event_id"""
        try:
            # 如果传入的是 numpy array，就转换成 bytes
            if isinstance(data_bytes, np.ndarray):
                data_bytes = data_bytes.tobytes()
            meta_bytes = json.dumps(meta).encode('utf-8') if meta else b'{}'

            self.socket.send_multipart([
                event_id.encode('utf-8'),
                data_bytes,
                meta_bytes
            ], flags=zmq.NOBLOCK)

        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f"ZeroMQ 发送错误 ({event_id}): {e}")

    def zeromq_receive_loop(self):
        """ZeroMQ -> ROS2 接收线程"""
        while self.running:
            try:
                if self.socket.poll(timeout=100): 
                    event_id, buffer_bytes = self.socket.recv_multipart()
                    event_id = event_id.decode('utf-8')

                    array_data = np.frombuffer(buffer_bytes, dtype=np.float32)
                    msg = Float32MultiArray()
                    msg.data = array_data.tolist()
                    goal_data = np.asarray(msg.data, dtype=np.float32)
                    print(event_id)
                    # print(msg.data)
                    if 'joint' in event_id:
                        print(1)
                        
                        arm_msg = deepcopy(self.latest_arm_cmd)
                        hand_msg = deepcopy(self.latest_hand_cmd)
                        hand_position_index=[10,7,1,2,4,5,11,6,9,8,12,3,22,17,15,24,21,13,20,16,23,18,14,19]
                        arm_position_index=[25, 26, 31, 27, 29, 30, 28, 32, 37, 33, 38, 34, 35, 36]
                        arm_position=[goal_data[i-1] for i in arm_position_index]
                        arm_msg.position[4:] = array('d', [float(x) for x in arm_position])
                       
                        hand_position = [goal_data[i-1] for i in hand_position_index]
                        hand_msg.position[:] = array('d', [float(x) for x in hand_position])
                        # print(hand_msg)
                        arm_msg.header.stamp = self.get_clock().now().to_msg()
                        hand_msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher_arm_joint.publish(arm_msg)
                        self.publisher_hand_joint.publish(hand_msg)
                    
                    # if 'base' in event_id:
                    #     self.publisher_base.publish(msg)
                    
                    
                time.sleep(0.05)
            except zmq.Again:
                continue
            except Exception as e:
                self.get_logger().error(f"ZeroMQ->ROS2 接收错误: {e}")
                time.sleep(0.1)

    def destroy(self):
        """安全退出线程并清理资源"""
        self.running = False
        if self.zeromq_thread.is_alive():
            self.zeromq_thread.join()
        self.socket.close()
        self._context.term()
        super().destroy_node()


def main():
    rclpy.init()
    node = ROS2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
