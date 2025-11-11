#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image,CompressedImage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
import os
import numpy as np
import zmq
import json
import time
import threading
os.chdir(os.path.dirname(os.path.abspath(__file__)))

IPC_ADDRESS = "ipc:///tmp/ros2-zeromq-image-pose-joints-odom"


class ROS2BridgeNode(Node,):
    def __init__(self,topic_dict):
        super().__init__('ros2_zeromq_bridge')

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.bind(IPC_ADDRESS)
        self.socket.setsockopt(zmq.SNDHWM, 2000)
        self.socket.setsockopt(zmq.SNDBUF, 2 ** 25)
        self.socket.setsockopt(zmq.SNDTIMEO, 2000)
        self.socket.setsockopt(zmq.RCVTIMEO, 2000)
        self.socket.setsockopt(zmq.LINGER, 0)

        self.min_interval_ns = 1e9 / 30  
        self.last_send_time_ns = 0

        self.topic_dict = topic_dict
        self._init_message_synchronizer()

        #预留
        self.running = True
        self.zeromq_thread = threading.Thread(target=self.zeromq_receive_loop, daemon=True)
        self.zeromq_thread.start()

        self.get_logger().info("桥接节点已启动")


    def _init_message_synchronizer(self,topic_dict):
        """定义 ROS2 订阅与时间同步"""
        subs=[]
        self.sub_map={}
        for comp_id, info in topic_dict.items():
            topic = info["topic"]
            msg_type_name = info["msgs"]

            if msg_type_name == "PoseStamped":
                msg_type = PoseStamped
            elif msg_type_name == "Odometry":
                msg_type = Odometry
            elif msg_type_name == "JointState":
                msg_type = JointState
            elif msg_type_name == "Image":
                msg_type = Image
            elif msg_type_name == "CompressedImage":
                msg_type = CompressedImage
            else:
                self.get_logger().warn(f"未知消息类型: {msg_type_name}, 跳过 {comp_id}")
                continue

            sub = Subscriber(self, msg_type, topic)
            subs.append(sub)
            self.sub_map[comp_id] = (sub, info)

        self.sync = ApproximateTimeSynchronizer(
            subs,
            queue_size=10,
            slop=0.04,
            allow_headerless=True
        )
        self.sync.registerCallback(self.sync_callback)


    def sync_callback(self, *msgs):
        try:
            now = time.time_ns()
            if (now - self.last_send_time_ns) < self.min_interval_ns:
                return
            self.last_send_time_ns = now

            for msg, (comp_id, info) in zip(msgs, self.sub_map.items()):
                msg_type = info["msgs"]

                # PoseStamped
                if msg_type == "PoseStamped":
                    data = np.array([
                        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                        msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w
                    ], dtype=np.float32)
                    self.send_zmq_event(comp_id, data)

                # Odometry
                elif msg_type == "Odometry":
                    pose = msg.pose.pose
                    twist = msg.twist.twist
                    data = np.array([
                        pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                        twist.linear.x, twist.linear.y, twist.linear.z,
                        twist.angular.x, twist.angular.y, twist.angular.z
                    ], dtype=np.float32)
                    self.send_zmq_event(comp_id, data)

                # JointState
                elif msg_type == "JointState":
                    data = np.array(msg.position, dtype=np.float32)
                    self.send_zmq_event(comp_id, data)

                # Image
                elif msg_type == "Image":
                    encoding = info.get("encoding", None)
                    if encoding is None:
                        encoding = getattr(msg, "encoding", "bgr8")

                    meta = {
                        "encoding": encoding,
                        "width": msg.width,
                        "height": msg.height,
                        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    }
                    self.send_zmq_event(comp_id, msg.data, meta)

                elif msg_type == "CompressedImage":
                    # encoding = info.get("encoding", None)
                    encoding = None
                    if encoding is None:
                        encoding = getattr(msg, "encoding", "jpeg")

                    meta = {
                        "encoding": encoding,
                        "width": msg.width,
                        "height": msg.height,
                        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    }
                    self.send_zmq_event(comp_id, msg.data, meta)

        except Exception as e:
            self.get_logger().error(f"同步回调出错: {e}")


    def send_zmq_event(self, event_id, data_bytes, meta=None):
        """发送单条 ZeroMQ 消息，带事件 ID"""
        try:
            if isinstance(data_bytes, np.ndarray):
                data_bytes = data_bytes.tobytes()
            meta_bytes = json.dumps(meta).encode('utf-8') if meta else b'{}'
            self.socket.send_multipart([event_id.encode(), data_bytes, meta_bytes], flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f"ZeroMQ 发送错误 ({event_id}): {e}")


    def zeromq_receive_loop(self):
        
        while self.running:
            time.sleep(0.1)


    def destroy(self):
        """安全退出并释放资源"""
        self.running = False
        if self.zeromq_thread.is_alive():
            self.zeromq_thread.join()
        self.socket.close()
        self.context.term()
        super().destroy_node()


def main():
    topic_dict= parse_topic_dict_from_config("../../../RoboQT/config/robot_config.json")
    rclpy.init()
    node = ROS2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

def parse_topic_dict_from_config(json_path):
    """从配置文件中解析 topic → 消息类型 → 附加信息"""
    with open(json_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    topic_dict = {}

    for comp in config.get("components", []):
        comp_id = comp.get("id")
        params = comp.get("params", {})
        topic = params.get("topic", "")
        msgs = params.get("msgs", "")
        outputs_info = comp.get("outputs_info", {})

        if "image" in topic.lower():
            encoding = params.get("encoding", "bgr8")
            topic_dict[comp_id] = {
                "topic": topic,
                "msgs": msgs,
                "encoding": encoding
            }

        else:
            if len(outputs_info) > 0:
                first_key = list(outputs_info.keys())[0]
                motors = outputs_info[first_key].get("motors", {})
            else:
                motors = {}

            topic_dict[comp_id] = {
                "topic": topic,
                "msgs": msgs,
                "motors_len": len(motors)
            }

    return topic_dict



if __name__ == "__main__":
    main()
