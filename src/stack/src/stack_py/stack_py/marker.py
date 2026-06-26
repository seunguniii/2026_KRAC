import time
import struct
import numpy as np
import cv2

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

from .kalman import TargetKalman2D
from .mission_manager import (
    MissionManager,
    NodeName,
    NodeState,
)

class Marker(Node):
    _ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    except AttributeError:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

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
        super().__init__('marker')
        
        self._bridge = CvBridge()
        
        self.status_publisher = self.create_publisher(UInt32, '/nodes/marker/status', 10)
        self.target_publisher = self.create_publisher(Point, '/nodes/marker/target', 10)
        self.stream_publisher = self.create_publisher(CompressedImage, '/nodes/marker/stream', 10)

        self.stream_subscriber = self.create_subscription(
            CompressedImage, '/nodes/vision/stream', self.stream_callback, 10
        )
        self.command_subscriber = self.create_subscription(
            UInt32, '/mission/command', self.command_callback, 10
        )


        #TODO: parameterize these
        is_simulation_ = True
        world_ = "aruco" 
        airframe_ = "standard_vtol_sensors"
        
        self._lidar_sub = self.create_subscription(
            PointCloud2,
            f"/world/{world_}/model/{airframe_}_0/link/lidar_sensor_link/sensor/lidar/scan/points",
            self._lidar_cb,
            10
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
    
        self._target_predict_timeout = 5.0
        self._last_detect_time = None
        
        self.mm = MissionManager()
        self.self_state = NodeState.IDLE
        
        self.timer = self.create_timer(1.0/FPS, self.report_status)


    def _lidar_cb(self, msg: PointCloud2) -> None:
        raw = bytes(msg.data)
        first_four = raw[0:4]
        self._altitude = struct.unpack('<f', first_four)[0] 


    def command_callback(self, msg):
        cmd = msg.data
        if self.mm.get_node(cmd) != NodeName.MARKER:
            return

        command = self.mm.get_command(cmd)
        if command != self.self_state:
            self.self_state = command
            self.get_logger().info("Command recieved from MISSION")
            
            if self.self_state == NodeState.IDLE:
                self._target_kf.reset()


    def report_status(self) -> None:
        msg = UInt32()
        msg.data = self.mm.pack(NodeName.MARKER, self.self_state)
        self.status_publisher.publish(msg)


    #main logic
    def stream_callback(self, msg: CompressedImage) -> None:
        if self.self_state != NodeState.BUSY:
            return

        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process_frame(frame)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to decode compressed image: {e}')


    def _process_frame(self, frame: np.ndarray) -> None:
        tag_center = self._detect_first_tag(frame)
        z = self._altitude 
        
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        fx, fy = self._CAMERA_MATRIX[0, 0], self._CAMERA_MATRIX[1, 1]

        cv2.drawMarker(
            frame, (center_x, center_y), 
            (255, 0, 0), cv2.MARKER_CROSS, 20, 2
        )

        if tag_center is not None and z > 0.05:
            cx, cy = tag_center
            
            dx = cx - center_x
            dy = center_y - cy
            raw_x_m = dx / fx * z
            raw_y_m = dy / fy * z

            smooth_x, smooth_y = self._target_kf.update(raw_x_m, raw_y_m)
            self._last_detect_time = time.monotonic()
            
            smooth_px = int((smooth_x * fx / z) + center_x)
            smooth_py = int(center_y - (smooth_y * fy / z))

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

            self._publish_coordinates(smooth_x, smooth_y, z)

        else:
            now = time.monotonic()
            if (self._last_detect_time and 
               (now - self._last_detect_time <= self._target_predict_timeout)):
                smooth_x, smooth_y = self._target_kf.predict_only()
                
                smooth_px = int((smooth_x * fx / z) + center_x)
                smooth_py = int(center_y - (smooth_y * fy / z))
                
                cv2.circle(frame, (smooth_px, smooth_py), 8, (0, 255, 255), -1)
                cv2.putText(
                    frame, "COAST", (smooth_px + 10, smooth_py), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
                )
                
                self._publish_coordinates(smooth_x, smooth_y, z)
            else:
                self._target_kf.reset()
                self._publish_coordinates(float('nan'), float('nan'), z)

        try:
            annotated_msg = self._bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            self.stream_publisher.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to encode annotated image: {e}')


    def _detect_first_tag(self, frame: np.ndarray) -> Optional[Tuple[float, float]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self._ARUCO_DICT,
            parameters=self._ARUCO_PARAMS,
        )
        
        if ids is not None and len(ids) > 0:
            pts = corners[0].reshape(4, 2)
            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))
            return cx, cy
            
        return None
        

    def _publish_coordinates(self, x: float, y: float, z: float):
        msg = Point()
        msg.x, msg.y, msg.z = x, y, z
        self.target_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    marker = Marker()
    rclpy.spin(marker)

    marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()

