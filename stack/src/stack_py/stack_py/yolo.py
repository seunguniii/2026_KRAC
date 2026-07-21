import time
import numpy as np

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import UInt32
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import CompressedImage, PointCloud2

from px4_msgs.msg import DistanceSensor

#if yaw information is added something like TargetKalmanYaw should be added.
from .kalman import TargetKalman2D

from .mission_manager import (
    MissionManager,
    NodeName,
    NodeState,
)

class Yolo(Node):
    def __init__(self):
        super().__init__('yolo')
        
        self._bridge = CvBridge()

        self.status_publisher = self.create_publisher(UInt32, '/nodes/yolo/status', 10)
        self.target_publisher = self.create_publisher(Quaternion, '/nodes/yolo/target', 10)
        
        #yolo debug frames
        self.stream_publisher = self.create_publisher(CompressedImage, '/nodes/yolo/stream', 10)

        self.stream_subscriber = self.create_subscription(
            CompressedImage, '/nodes/vision/stream', self.stream_callback, 10
        )
        self.command_subscriber = self.create_subscription(
            UInt32, '/mission/command', self.command_callback, 10
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )
        
        #lidar related code should be kept:
        #altitude is used for distance calculation
        #and is sent to target node for further control
        self._lidar_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self._lidar_cb,
            qos_profile_sub
        )
    
        #do NOT use 0, will cause division-by-0 error
        self._altitude = 1.0 

        #TODO: tune values for actual aircraft
        FPS = 30
        self._target_kf = TargetKalman2D(
            process_var=0.01, 
            measurement_var=0.08, 
            default_dt=1.0/FPS
        )
    
        self.mm = MissionManager()
        self.self_state = NodeState.IDLE
        
        self.timer = self.create_timer(1.0/FPS, self.report_status)


    def _lidar_cb(self, msg) -> None:
        self._altitude = msg.current_distance


    def command_callback(self, msg):
        cmd = msg.data
        if self.mm.get_node(cmd) != NodeName.YOLO:
            return

        command = self.mm.get_command(cmd)
        if command != self.self_state:
            self.self_state = command
            self.get_logger().info("Command recieved from MISSION")
            
            if self.self_state == NodeState.IDLE:
                self._target_kf.reset()


    def report_status(self) -> None:
        msg = UInt32()
        msg.data = self.mm.pack(NodeName.YOLO, self.self_state)
        self.status_publisher.publish(msg)


    def stream_callback(self, msg: CompressedImage) -> None:
        
        #TODO: set when node should run
        if self.self_state != NodeState.BUSY:
            return

        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process_frame(frame)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to decode compressed image: {e}')


    def _process_frame(self, frame: np.ndarray) -> None:
        #TODO: add yolo and draw debug frames. refer to marker node.
        pass

    #publishes target state to target node.
    def _publish_coordinates(self, x: float, y: float, z: float, yaw: float):
        msg = Quaternion()
        msg.x, msg.y, msg.z, msg.yaw = 0., 0., 0., 0.
        self.target_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    yolo = Yolo()
    rclpy.spin(yolo)

    yolo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()

