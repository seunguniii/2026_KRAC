#!/usr/bin/env python3

import os
import time
import math
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from std_msgs.msg import UInt32
from sensor_msgs.msg import CompressedImage
from px4_msgs.msg import VehicleOdometry

from .mission_manager import (
    MissionManager,
    NodeName,
    NodeState,
)

class MissionGui(Node):
    def __init__(self):
        super().__init__("mission_gui")
        self.mm = MissionManager()
        
        self.PANEL_W = 640
        self.PANEL_H = 360
        self.TRAJ_SIZE = self.PANEL_H * 2
        self.DISPLAY_SCALE = 0.7
        #frame size: 2000X720 * display scale

        self.frames = {
            "raw": np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8),
            "yolo": np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8),
            "marker": np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8),
        }
        
        self.last_image_time = {
            "raw": 0.0,
            "yolo": 0.0,
            "marker": 0.0,
        }
        self.last_status_time = 0.0

        self.wanted_pts = np.array([])
        self.actual_pts_list = []
        self.actual_pts = np.array([])

        self.load_trajectory_from_yaml()

        #TODO: change these to use gstreamer pipelines
        self.raw_sub = self.create_subscription(
            CompressedImage, "/nodes/vision/stream", self.raw_callback, 10
        )
        self.yolo_sub = self.create_subscription(
            CompressedImage, "/nodes/yolo/stream", self.yolo_callback, 10
        )
        self.marker_sub = self.create_subscription(
            CompressedImage, "/nodes/marker/stream", self.marker_callback, 10
        )
        self.status_sub = self.create_subscription(
            UInt32, "/mission/summary", self.status_callback, 10
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(UInt32, "ground/command", 10)
        self.timer = self.create_timer(0.03, self.update_gui)
        self.get_logger().info("Mission GUI started")

    def load_trajectory_from_yaml(self):
        try:
            mission_launch_share = get_package_share_directory('mission_launch')
            default_yaml_path = os.path.join(mission_launch_share, 'config', 'trajectory.yaml')
        except PackageNotFoundError:
            self.get_logger().warn("Package 'mission_launch' not found in share index.")
            default_yaml_path = ''

        self.declare_parameter('trajectory_file', default_yaml_path)
        yaml_file = self.get_parameter('trajectory_file').get_parameter_value().string_value

        if not yaml_file or not os.path.exists(yaml_file):
            self.get_logger().error(f"Trajectory YAML file not found at '{yaml_file}'")
            return

        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
            
            raw_pts = data.get('waypoints', data.get('setpoints', data.get('trajectory', [])))
            
            pts = []
            for p in raw_pts:
                if isinstance(p, dict):
                    e_val = p.get('e', p.get('x', None))
                    n_val = p.get('n', p.get('y', None))
                    
                    if e_val is not None and n_val is not None:
                        pts.append([float(e_val), float(n_val)])

                elif isinstance(p, (list, tuple)) and len(p) >= 2:
                    pts.append([float(p[0]), float(p[1])])

            if pts:
                self.wanted_pts = np.array(pts)
                self.get_logger().info(f"Loaded {len(self.wanted_pts)} setpoints from '{yaml_file}'")
            else:
                self.get_logger().warn(f"No valid setpoint keys found in '{yaml_file}'")

        except Exception as e:
            self.get_logger().error(f"Failed to parse trajectory.yaml: {e}")

    def raw_callback(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        self.frames["raw"] = cv2.resize(img, (self.PANEL_W, self.PANEL_H))
        self.last_image_time["raw"] = time.time()

    def yolo_callback(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        self.frames["yolo"] = cv2.resize(img, (self.PANEL_W, self.PANEL_H))
        self.last_image_time["yolo"] = time.time()

    def marker_callback(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR) 
        self.frames["marker"] = cv2.resize(img, (self.PANEL_W, self.PANEL_H))
        self.last_image_time["marker"] = time.time()
            
    def status_callback(self, msg):
        self.mm.set_raw(msg.data)
        self.last_status_time = time.time()

    def odom_callback(self, msg):
        actual_n = msg.position[0]
        actual_e = msg.position[1]

        if math.isnan(actual_n) or math.isnan(actual_e):
            return

        self.actual_pts_list.append([float(actual_e), float(actual_n)])
        self.actual_pts = np.array(self.actual_pts_list)

    def send_command(self, node: NodeName, state: NodeState):
        cmd = self.mm.pack(node, state)
        msg = UInt32()
        msg.data = cmd

        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)
        
        self.get_logger().info(f"Command sent: {node.name} -> {state.name}")

    def lost_frame(self, title):
        frame = np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8)
        cv2.putText(frame, title, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        text = "NO SIGNAL"
        (font_w, font_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
        x = (self.PANEL_W - font_w) // 2
        y = (self.PANEL_H + font_h) // 2
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        
        return frame
        
    def draw_status_panel(self):
        panel = np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8)
        x, y = 20, 40

        cv2.putText(panel, "MISSION STATUS", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        y += 35
        
        mode = self.mm.get_summary_mode()
        cv2.putText(panel, f"MISSION MODE : {mode.name}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
        y += 40

        for node in NodeName:
            try:
                state = self.mm.get(node)
            except Exception:
                state = NodeState.ABORT
            color = self.state_color(state)
            text = f"{node.name:<18}: {state.name}"
            cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            y += 25
        
        color = ((0, 255, 0) if self.status_alive else (0, 0, 255))
        text = "MASTER STATUS : OK" if self.status_alive else "MASTER STATUS : TIMEOUT"
        cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        y += 25

        color = ((0, 255, 0) if self.raw_alive else (0, 0, 255))
        text = "VIDEO LINK : OK" if self.raw_alive else "VIDEO LINK : TIMEOUT"
        cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        h = panel.shape[0]
        cv2.putText(panel, "[S] START", (520, h - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(panel, "[A] ABORT", (520, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(panel, "[Q] QUIT", (520, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
       
        return panel

    def draw_trajectory_panel(self):
        panel = np.zeros((self.TRAJ_SIZE, self.TRAJ_SIZE, 3), dtype=np.uint8)
        cv2.putText(panel, "TRAJECTORY MAP", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if self.wanted_pts.size == 0:
            cv2.putText(panel, "NO TRAJECTORY LOADED", (200, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
            return panel

        #map boundaries
        min_x, max_x = np.min(self.wanted_pts[:, 0]), np.max(self.wanted_pts[:, 0])
        min_y, max_y = np.min(self.wanted_pts[:, 1]), np.max(self.wanted_pts[:, 1])

        range_x = max(max_x - min_x, 1.0)
        range_y = max(max_y - min_y, 1.0)
        max_range = max(range_x, range_y)
        
        center_x = (max_x + min_x) / 2.0
        center_y = (max_y + min_y) / 2.0

        margin = 40
        drawable_size = self.TRAJ_SIZE - (2 * margin)
        scale = drawable_size / max_range

        def map_coords(pts):
            if pts.size == 0:
                return np.array([])
            mapped_x = (pts[:, 0] - center_x) * scale + (self.TRAJ_SIZE / 2)
            mapped_y = (self.TRAJ_SIZE / 2) - (pts[:, 1] - center_y) * scale
            return np.int32(np.column_stack((mapped_x, mapped_y)))

        wanted_mapped = map_coords(self.wanted_pts)
        cv2.polylines(panel, [wanted_mapped], isClosed=False, color=(255, 150, 50), thickness=2)

        if self.actual_pts.size > 0:
            actual_mapped = map_coords(self.actual_pts)
            cv2.polylines(panel, [actual_mapped], isClosed=False, color=(50, 255, 50), thickness=2)

        cv2.putText(panel, "Planned Path", (20, self.TRAJ_SIZE - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 150, 50), 2)
        cv2.putText(panel, "Actual Path", (20, self.TRAJ_SIZE - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 50), 2)

        return panel

    @staticmethod
    def state_color(state):
        if state == NodeState.IDLE: return (180, 180, 180)
        elif state == NodeState.BUSY: return (0, 255, 255)
        elif state == NodeState.SUCCESS: return (0, 255, 0)
        elif state == NodeState.ABORT: return (0, 0, 255)
        return (255, 255, 255)

    def update_gui(self):
        timeout = 2.0
        now = time.time()
        self.status_alive = (now - self.last_status_time) < timeout
        self.raw_alive = (now - self.last_image_time["raw"]) < timeout
        self.yolo_alive = (now - self.last_image_time["yolo"]) < timeout
        self.marker_alive = (now - self.last_image_time["marker"]) < timeout
        
        raw = self.frames["raw"].copy() if self.raw_alive else self.lost_frame("RAW")
        yolo = self.frames["yolo"].copy() if self.yolo_alive else self.lost_frame("YOLO")
        marker = self.frames["marker"].copy() if self.marker_alive else self.lost_frame("MARKER")
            
        status = self.draw_status_panel()
        traj_panel = self.draw_trajectory_panel()

        # Compose 2000x720 layout
        top = np.hstack((raw, yolo))
        bottom = np.hstack((status, marker))
        left_dashboard = np.vstack((top, bottom))
        final_dashboard = np.hstack((left_dashboard, traj_panel))

        display_w = int(final_dashboard.shape[1] * self.DISPLAY_SCALE)
        display_h = int(final_dashboard.shape[0] * self.DISPLAY_SCALE)
        
        display_dashboard = cv2.resize(final_dashboard, (display_w, display_h), interpolation=cv2.INTER_AREA)

        cv2.imshow("Mission GUI", display_dashboard)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.get_logger().info("Starting MISSION node.")
            self.send_command(NodeName.MISSION, NodeState.BUSY)
        elif key == ord('a'):
            self.get_logger().error("!!!ABORTING MISSION!!!")
            self.send_command(NodeName.MISSION, NodeState.ABORT)
        elif key == ord('q'):
            self.get_logger().info("Closing Mission GUI.")
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MissionGui()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
