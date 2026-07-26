#!/usr/bin/env python3

import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32
from sensor_msgs.msg import CompressedImage

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

        self.raw_sub = self.create_subscription(
            CompressedImage,
            "/nodes/vision/stream",
            self.raw_callback,
            10
        )

        self.yolo_sub = self.create_subscription(
            CompressedImage,
            "/nodes/yolo/stream",
            self.yolo_callback,
            10
        )

        self.marker_sub = self.create_subscription(
            CompressedImage,
            "/nodes/marker/stream",
            self.marker_callback,
            10
        )

        self.status_sub = self.create_subscription(
            UInt32,
            "/mission/summary",
            self.status_callback,
            10
        )

        self.cmd_pub = self.create_publisher(UInt32, "ground/command", 10)
        self.timer = self.create_timer(0.03, self.update_gui)
        self.get_logger().info("Mission GUI started")


    def raw_callback(self, msg):
        img = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        self.frames["raw"] = cv2.resize(img, (self.PANEL_W, self.PANEL_H))
        self.last_image_time["raw"] = time.time()


    def yolo_callback(self, msg):
        img = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        self.frames["yolo"] = cv2.resize(img, (self.PANEL_W, self.PANEL_H))
        self.last_image_time["yolo"] = time.time()

    def marker_callback(self, msg):
        img = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        ) 

        self.frames["marker"] = cv2.resize(img, (self.PANEL_W, self.PANEL_H))
        self.last_image_time["marker"] = time.time()
            
    def status_callback(self, msg):
        self.mm.set_raw(msg.data)
        self.last_status_time = time.time()


    def send_command(self, node: NodeName, state: NodeState):
        cmd = self.mm.pack(node, state)

        msg = UInt32()
        msg.data = cmd

	#send x3 for safety measures
        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)
        
        self.get_logger().info(
            f"Command sent: "
            f"{node.name} -> {state.name}"
        )

    def lost_frame(self, title):
        frame = np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8)
        
        cv2.putText(
            frame, title, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 2
        )
        
        text = "NO SIGNAL"
        (font_w, font_h), baseline = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3
        )
        x = (self.PANEL_W - font_w) // 2
        y = (self.PANEL_H + font_h) // 2
        cv2.putText(
            frame, text, (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2, (0, 0, 255), 3
        )
        
        return frame
        
    def draw_status_panel(self):
        panel = np.zeros((self.PANEL_H, self.PANEL_W, 3), dtype=np.uint8)
        x = 20
        y = 40

        cv2.putText(
            panel, "MISSION STATUS", (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255,255,255), 2
        )
        
        y += 35
        
        mode = self.mm.get_summary_mode()
        cv2.putText(
            panel, f"MISSION MODE : {mode.name}", (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255,255,255), 2
        )

        y += 40

        for node in NodeName:
            try:
                state = self.mm.get(node)
            except Exception:
                state = NodeState.ABORT
            color = self.state_color(state)
            text = (
                f"{node.name:<18}"
                f": {state.name}"
            )

            cv2.putText(
                panel, text, (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 2
            )

            y += 25
        
        #green if alive else red
        color = ((0, 255, 0) if self.status_alive else (0, 0, 255))

        text = (
            "MASTER STATUS : OK"
            if self.status_alive
            else "MASTER STATUS : TIMEOUT"
        )

        cv2.putText(
            panel, text, (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, color, 2
        )

        y += 25


        color = ((0, 255, 0) if self.raw_alive else (0, 0, 255))

        text = (
            "VIDEO LINK : OK"
            if self.raw_alive
            else "VIDEO LINK : TIMEOUT"
        )

        cv2.putText(
            panel, text, (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, color, 2
        )

        h = panel.shape[0]

        cv2.putText(
            panel, "[S] START",
            (520, h - 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2
        )

        cv2.putText(
            panel, "[A] ABORT",
            (520, h - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 0, 255), 2
        )

        cv2.putText(
            panel, "[Q] QUIT",
            (520, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 2
        )
       
        return panel
        

    @staticmethod
    def state_color(state):
        if state == NodeState.IDLE:
            return (180, 180, 180)
        elif state == NodeState.BUSY:
            return (0, 255, 255)
        elif state == NodeState.SUCCESS:
            return (0, 255, 0)
        elif state == NodeState.ABORT:
            return (0, 0, 255)

        return (255, 255, 255)


    def update_gui(self):
        timeout = 2.0
        now = time.time()
        self.status_alive = (now - self.last_status_time) < 2.0
        self.raw_alive = now - self.last_image_time["raw"] < timeout
        self.yolo_alive = now - self.last_image_time["yolo"] < timeout
        self.marker_alive = now - self.last_image_time["marker"] < timeout
        
        if self.raw_alive:
            raw = self.frames["raw"].copy()
        else:
            raw = self.lost_frame("RAW")

        if self.yolo_alive:
            yolo = self.frames["yolo"].copy()
        else:
            yolo = self.lost_frame("YOLO")

        if self.marker_alive:
            marker = self.frames["marker"].copy()
        else:
            marker = self.lost_frame("MARKER")
            
        status = self.draw_status_panel()

        '''
        cv2.putText(raw, "RAW", (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255,255,255), 2)

        cv2.putText(yolo, "YOLO", (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255,255,255), 2)

        cv2.putText(marker, "Marker", (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255,255,255), 2)
        '''
        
        top = np.hstack((raw, yolo))
        raw_row = np.hstack((status, marker))

        dashboard = np.vstack((top, raw_row))

        cv2.imshow("Mission GUI", dashboard)

        key = cv2.waitKey(1) & 0xFF

	#start
        if key == ord('s'):
            self.get_logger().info("Starting MISSION node.")
            self.send_command(NodeName.MISSION, NodeState.BUSY)

        #abort mission
        elif key == ord('a'):
            self.get_logger().error("!!!ABORTING MISSION!!!")
            self.send_command(NodeName.MISSION, NodeState.ABORT)
            
        #quit gui
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
