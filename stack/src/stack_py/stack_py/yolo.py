import time
import math
import numpy as np
import cv2
import torch
import os

from typing import Optional, Tuple

from ultralytics import YOLO as YOLOModel

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import UInt32
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from px4_msgs.msg import DistanceSensor

from .kalman import TargetKalman2D
from .mission_manager import (
    MissionManager,
    NodeName,
    NodeState,
)


class YOLO(Node):
    #TODO: find values for the actual aircraft
    _CAMERA_MATRIX = np.array(
        [[827.99145461, 0.0, 249.63373237],
         [0.0, 826.30893069, 260.11920342],
         [0.0, 0.0, 1.0]]
    )
    _DIST_COEFFS = np.array(
        [[-0.27436478, 0.31753802, 0.00183457, -0.01212723, 0.05024013]]
    )

    def __init__(self):
        super().__init__('yolo')

        self._bridge = CvBridge()

        model_path = os.path.expanduser("~/2026_KRAC/stack/src/stack_py/resource/yolo.pt")
        self.model = YOLOModel(model_path)
        self.image_size = 480
        self.confidence_threshold = 0.35
        if torch.cuda.is_available():
            self.device = "0"
            self.get_logger().warn("CUDA-supported GPU found.")
        else:
            self.device = "cpu"
            self.get_logger().warn("No CUDA-supported GPU found. Fall back to CPU.")

        self.status_publisher = self.create_publisher(UInt32, '/nodes/yolo/status', 10)
        self.target_publisher = self.create_publisher(Quaternion, '/nodes/yolo/target', 10)
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
            depth=10
        )

        self._lidar_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self._lidar_cb,
            qos_profile_sub
        )
    
        self._altitude = 1.0 

        # TODO: tune values for actual aircraft
        FPS = 30
        self._target_kf = TargetKalman2D(
            process_var=0.01, 
            measurement_var=0.08, 
            default_dt=1.0/FPS
        )
    
        self._target_predict_timeout = 5.0
        self._last_detect_time = None
        
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
        if self.self_state != NodeState.BUSY:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().error('Failed to decode image buffer via cv2.imdecode')
                return
                
            self._process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process stream: {e}')

    def _process_frame(self, frame: np.ndarray) -> None:
        z = self._altitude 
        
        height, width = frame.shape[:2]
        center_x = width*0.5
        center_y = height*0.5
        fx, fy = self._CAMERA_MATRIX[0, 0], self._CAMERA_MATRIX[1, 1]

        cv2.drawMarker(
            frame, (int(center_x), int(center_y)), 
            (255, 0, 0), cv2.MARKER_CROSS, 20, 2
        )

        try:
            results = self.model(
                frame,
                imgsz=self.image_size,
                conf=self.confidence_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as error:
            self.get_logger().error(f"YOLO inference failed: {error}")
            return

        obb = results[0].obb
        detection_found = obb is not None and len(obb) > 0

        target_angle_deg = float("nan")

        if detection_found and z > 0.05:
            xywhr = obb.xywhr.detach().cpu().numpy()
            polygons = obb.xyxyxyxy.detach().cpu().numpy()
            confidences = obb.conf.detach().cpu().numpy()
            
            target_index = int(np.argmax(confidences))

            (cx, cy, tw, th, trad) = xywhr[target_index]
            target_polygon = polygons[target_index].round().astype(np.int32)

            target_angle_deg = self.compute_long_axis_angle_deg(target_polygon) - 90.0
            
            dx = cx - center_x
            dy = center_y - cy
            
            raw_x_m = dx / fx * z
            raw_y_m = dy / fy * z

            smooth_x, smooth_y = self._target_kf.update(raw_x_m, raw_y_m)
            self._last_detect_time = time.monotonic()
            
            smooth_px = int((smooth_x * fx / z) + center_x)
            smooth_py = int(center_y - (smooth_y * fy / z))

            cv2.polylines(frame, [target_polygon.reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=3)
            cv2.circle(frame, (int(cx), int(cy)), 8, (0, 0, 255), -1)
            cv2.putText(
                frame, "RAW", (int(cx) + 10, int(cy)), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
            )

            cv2.circle(frame, (smooth_px, smooth_py), 8, (0, 255, 0), -1)
            cv2.putText(
                frame, "KF", (smooth_px + 10, smooth_py), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

            self._publish_coordinates(smooth_x, smooth_y, z, target_angle_deg)

        else:
            now = time.monotonic()
            if (self._last_detect_time and 
               (now - self._last_detect_time <= self._target_predict_timeout)):
                smooth_x, smooth_y = self._target_kf.predict_only()
                
                if z > 0.05:
                    smooth_px = int((smooth_x * fx / z) + center_x)
                    smooth_py = int(center_y - (smooth_y * fy / z))
                    
                    cv2.circle(frame, (smooth_px, smooth_py), 8, (0, 255, 255), -1)
                    cv2.putText(
                        frame, "COAST", (smooth_px + 10, smooth_py), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
                    )
                
                self._publish_coordinates(smooth_x, smooth_y, z, target_angle_deg)
            else:
                self._target_kf.reset()
                self._publish_coordinates(float('nan'), float('nan'), z, float('nan'))

        try:
            annotated_msg = self._bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            self.stream_publisher.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to encode annotated image: {e}')

    @staticmethod
    def compute_long_axis_angle_deg(polygon: np.ndarray) -> float:
        points = np.asarray(polygon, dtype=np.float32).reshape(4, 2)
        edge_vectors = np.roll(points, -1, axis=0) - points
        edge_lengths_sq = np.sum(edge_vectors * edge_vectors, axis=1)
        longest_edge = edge_vectors[int(np.argmax(edge_lengths_sq))]

        dx = float(longest_edge[0])
        dy = float(longest_edge[1])

        if dx == 0.0 and dy == 0.0:
            return float("nan")

        return math.degrees(math.atan2(dy, dx)) % 180.0

    def _publish_coordinates(self, x: float, y: float, z: float, w: float = 0.0):
        msg = Quaternion()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.w = float(w)
        self.target_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    yolo_node = YOLO()
    rclpy.spin(yolo_node)

    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
