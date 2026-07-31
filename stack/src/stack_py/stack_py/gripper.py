import time
import struct
import numpy as np
import cv2

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import UInt32, Float64, Int32
from px4_msgs.msg import DistanceSensor, VehicleCommand

from .mission_manager import (
    MissionManager,
    NodeName,
    NodeState,
)


class Gripper(Node):
    def __init__(self):
        super().__init__('gripper')
        
        self.status_publisher = self.create_publisher(UInt32, '/nodes/gripper/status', 10)
        self.command_subscriber = self.create_subscription(UInt32, '/mission/command', self.command_callback, 10)

        self.ground_control_sub = self.create_subscription(Int32, '/ground/gripper_control', self.ground_control_callback, 10)

        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.pub_left_h = self.create_publisher(Float64, '/gripper/left/horizontal', 10)
        self.pub_right_h = self.create_publisher(Float64, '/gripper/right/horizontal', 10)
        self.pub_left_v = self.create_publisher(Float64, '/gripper/left/vertical', 10)
        self.pub_right_v = self.create_publisher(Float64, '/gripper/right/vertical', 10)

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )
        
        self.mm = MissionManager()
        self.self_state = NodeState.IDLE
        
        self.timer = self.create_timer(0.1, self.report_status)
        self.control_timer = self.create_timer(0.1, self.gripper_callback)

    def ground_control_callback(self, msg: Int32) -> None:
        cmd_data = msg.data
        if cmd_data == 0:
            self.get_logger().error("Command received from GROUND. ABORT.")
            self.self_state = NodeState.ABORT

        if cmd_data == 1:
            self.get_logger().info("Command received from GROUND. Setting state to SUCCESS")
            self.self_state = NodeState.SUCCESS

        elif cmd_data == 2:
            self.get_logger().info("Command received from GROUND. Executing RESCUE phase 1")
            self._actuate_horizontal(left_val=-0.1183, right_val=0.1183)

        elif cmd_data == 3:
            self.get_logger().info("Command received from GROUND. Executing RESCUE phase 2")
            self._actuate_vertical(left_val=0.12, right_val=0.12)
            
        elif cmd_data == 4:
            self.get_logger().info("Command received from GROUND. Executing DROP phase 1")
            self._actuate_horizontal(left_val=0.0, right_val=0.0)

        elif cmd_data == 5:
            self.get_logger().info("Command received from GROUND. Executing DROP phase 2")
            self._actuate_vertical(left_val=0.0, right_val=0.0)

        else:
            self.get_logger().warn(f"Ground Control: Unhandled command '{cmd_data}' received.")

    def command_callback(self, msg):
        cmd = msg.data
        if self.mm.get_node(cmd) != NodeName.GRIPPER:
            return

        command = self.mm.get_command(cmd)
        if command != self.self_state:
            self.self_state = command
            self.get_logger().info("Command received from MISSION")

    def report_status(self) -> None:
        msg = UInt32()
        msg.data = self.mm.pack(NodeName.GRIPPER, self.self_state)
        self.status_publisher.publish(msg)

    # Main actuation logic loop
    def gripper_callback(self) -> None:
        if self.self_state != NodeState.BUSY:
            return

        # Continuous or state-machine-driven gripper behaviors when BUSY

    def _actuate_horizontal(self, left_val: float, right_val: float) -> None:
        msg_left = Float64(data=float(left_val))
        msg_right = Float64(data=float(right_val))
        
        self.pub_left_h.publish(msg_left)
        self.pub_right_h.publish(msg_right)

    def _actuate_vertical(self, left_val: float, right_val: float) -> None:
        msg_left = Float64(data=float(left_val))
        msg_right = Float64(data=float(right_val))
        
        self.pub_left_v.publish(msg_left)
        self.pub_right_v.publish(msg_right)

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
    try:
        rclpy.spin(gripper)
    except KeyboardInterrupt:
        pass
    finally:
        gripper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
