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

# ============================================================
# GUI NODE
# ============================================================

class MissionGui(Node):

    def __init__(self):

        super().__init__("mission_gui")

        self.mm = MissionManager()

        self.frame = np.zeros(
            (720, 1280, 3),
            dtype=np.uint8
        )

        self.last_image_time = 0.0
        self.last_status_time = 0.0

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/vision/debug_image/compressed",
            self.image_callback,
            10
        )

        self.status_sub = self.create_subscription(
            UInt32,
            "/mission/summary",
            self.status_callback,
            10
        )

        # ====================================================
        # PUBLISHERS
        # ====================================================

        self.cmd_pub = self.create_publisher(
            UInt32,
            "/master/cmd",
            10
        )

        # ====================================================
        # GUI TIMER
        # ====================================================

        self.timer = self.create_timer(
            0.03,
            self.update_gui
        )

        self.get_logger().info(
            "Mission GUI started"
        )

    # ========================================================
    # CALLBACKS
    # ========================================================

    def image_callback(self, msg):

        try:

            np_arr = np.frombuffer(
                msg.data,
                np.uint8
            )

            self.frame = cv2.imdecode(
                np_arr,
                cv2.IMREAD_COLOR
            )

            self.last_image_time = time.time()

        except Exception as e:

            self.get_logger().error(
                f"Image decode failed: {e}"
            )

    def status_callback(self, msg):

        self.mm.set_raw(msg.data)

        self.last_status_time = time.time()

    # ========================================================
    # COMMANDS
    # ========================================================

    def send_command(
        self,
        node: NodeName,
        state: NodeState
    ):

        cmd = self.mm.pack(node, state)

        msg = UInt32()
        msg.data = cmd

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f"Command sent: "
            f"{node.name} -> {state.name}"
        )

    # ========================================================
    # DRAWING
    # ========================================================

    def draw_status_panel(self, frame):

        x = 20
        y = 40

        # ----------------------------------------------------
        # TITLE
        # ----------------------------------------------------

        cv2.putText(
            frame,
            "MISSION STATUS",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        y += 50

        # ----------------------------------------------------
        # NODE STATES
        # ----------------------------------------------------

        for node in NodeName:

            try:

                state = self.mm.get(node)

            except Exception:

                state = NodeState.ERROR

            color = self.state_color(state)

            text = (
                f"{node.name:<18}"
                f": {state.name}"
            )

            cv2.putText(
                frame,
                text,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2
            )

            y += 40

        # ----------------------------------------------------
        # STATUS LINK
        # ----------------------------------------------------

        status_alive = (
            time.time()
            - self.last_status_time
        ) < 2.0

        color = (
            (0, 255, 0)
            if status_alive
            else (0, 0, 255)
        )

        text = (
            "MASTER STATUS : OK"
            if status_alive
            else "MASTER STATUS : TIMEOUT"
        )

        cv2.putText(
            frame,
            text,
            (x, y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

        # ----------------------------------------------------
        # VIDEO LINK
        # ----------------------------------------------------

        image_alive = (
            time.time()
            - self.last_image_time
        ) < 2.0

        color = (
            (0, 255, 0)
            if image_alive
            else (0, 0, 255)
        )

        text = (
            "VIDEO LINK : OK"
            if image_alive
            else "VIDEO LINK : TIMEOUT"
        )

        cv2.putText(
            frame,
            text,
            (x, y + 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

        # ----------------------------------------------------
        # CONTROLS
        # ----------------------------------------------------

        h = frame.shape[0]

        cv2.putText(
            frame,
            "[S] START",
            (20, h - 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            "[A] ABORT",
            (20, h - 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            "[Q] QUIT",
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

    # ========================================================
    # COLORS
    # ========================================================

    @staticmethod
    def state_color(state):

        if state == NodeState.IDLE:
            return (180, 180, 180)

        elif state == NodeState.BUSY:
            return (0, 255, 255)

        elif state == NodeState.COMPLETED:
            return (0, 255, 0)

        elif state == NodeState.ERROR:
            return (0, 0, 255)

        return (255, 255, 255)

    # ========================================================
    # GUI LOOP
    # ========================================================

    def update_gui(self):

        frame = self.frame.copy()

        self.draw_status_panel(frame)

        cv2.imshow(
            "Mission GUI",
            frame
        )

        key = cv2.waitKey(1) & 0xFF

        # ----------------------------------------------------
        # START
        # ----------------------------------------------------

        if key == ord('s'):

            self.send_command(
                NodeName.MASTER,
                NodeState.BUSY
            )

        # ----------------------------------------------------
        # ABORT
        # ----------------------------------------------------

        elif key == ord('a'):

            self.send_command(
                NodeName.MASTER,
                NodeState.ERROR
            )

        # ----------------------------------------------------
        # QUIT
        # ----------------------------------------------------

        elif key == ord('q'):

            self.get_logger().info(
                "Closing Mission GUI"
            )

            cv2.destroyAllWindows()

            rclpy.shutdown()


# ============================================================
# MAIN
# ============================================================

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
