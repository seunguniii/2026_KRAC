import time
import struct
import numpy as np
import cv2

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import UInt32

from px4_msgs.msg import DistanceSensor

from .mission_manager import (
    MissionManager,
    NodeName,
    NodeState,
)


class Gripper(Node):
    def __init__(self):
        super().__init__('gripper')
        self.status_publisher = self.create_publisher(UInt32, '/nodes/gripper/status', 10)
        
        self.command_subscriber = self.create_subscription(
            UInt32, '/mission/command', self.command_callback, 10
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        #self._lidar_sub = self.create_subscription(
        #    DistanceSensor,
        #    '/fmu/out/distance_sensor',
        #    self._lidar_cb,
        #    qos_profile_sub
        #)
        
        self.mm = MissionManager()
        self.self_state = NodeState.IDLE
        
        self.timer = self.create_timer(1.0/FPS, self.report_status)


    def command_callback(self, msg):
        cmd = msg.data
        if self.mm.get_node(cmd) != NodeName.GRIPPER:
            return

        command = self.mm.get_command(cmd)
        if command != self.self_state:
            self.self_state = command
            self.get_logger().info("Command recieved from MISSION")


    def report_status(self) -> None:
        msg = UInt32()
        msg.data = self.mm.pack(NodeName.GRIPPER, self.self_state)
        self.status_publisher.publish(msg)


    #main logic
    def stream_callback(self, msg: CompressedImage) -> None:
        if self.self_state != NodeState.BUSY:
            return
        
        
        #do stuffs


    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param7: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    gripper = Gripper()
    rclpy.spin(gripper)

    marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()

