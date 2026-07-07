#!/usr/bin/env python3
from __future__ import annotations

import math
import struct
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Float32
from ultralytics import YOLO


class TargetKalman2D:
    """
    2차원 표적 위치 칼만필터.

    상태:
        [x_m, y_m, vx_mps, vy_mps]

    측정:
        [raw_x_m, raw_y_m]
    """

    def __init__(
        self,
        process_var: float = 0.01,
        measurement_var: float = 0.08,
        default_dt: float = 1.0 / 30.0,
    ) -> None:
        self.kf = cv2.KalmanFilter(4, 2)

        self.process_var = float(process_var)
        self.measurement_var = float(measurement_var)
        self.default_dt = float(default_dt)

        self.initialized = False
        self.last_time = time.monotonic()

        self.kf.transitionMatrix = np.eye(4, dtype=np.float32)

        self.kf.measurementMatrix = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=np.float32,
        )

        self.kf.processNoiseCov = np.array(
            [
                [self.process_var, 0.0, 0.0, 0.0],
                [0.0, self.process_var, 0.0, 0.0],
                [0.0, 0.0, self.process_var * 10.0, 0.0],
                [0.0, 0.0, 0.0, self.process_var * 10.0],
            ],
            dtype=np.float32,
        )

        self.kf.measurementNoiseCov = (
            np.eye(2, dtype=np.float32) * self.measurement_var
        )
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

    def reset(self) -> None:
        self.initialized = False
        self.last_time = time.monotonic()
        self.kf.statePost = np.zeros((4, 1), dtype=np.float32)
        self.kf.statePre = np.zeros((4, 1), dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

    def _get_dt(self) -> float:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.001 or dt > 1.0:
            dt = self.default_dt

        return dt

    def _update_transition(self, dt: float) -> None:
        self.kf.transitionMatrix = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    def update(
        self,
        raw_x_m: float,
        raw_y_m: float,
    ) -> Tuple[float, float]:
        if not math.isfinite(raw_x_m) or not math.isfinite(raw_y_m):
            return self.predict_only()

        dt = self._get_dt()
        self._update_transition(dt)

        if not self.initialized:
            initial_state = np.array(
                [
                    [raw_x_m],
                    [raw_y_m],
                    [0.0],
                    [0.0],
                ],
                dtype=np.float32,
            )

            self.kf.statePost = initial_state.copy()
            self.kf.statePre = initial_state.copy()
            self.initialized = True

            return float(raw_x_m), float(raw_y_m)

        self.kf.predict()

        measurement = np.array(
            [
                [raw_x_m],
                [raw_y_m],
            ],
            dtype=np.float32,
        )

        estimated = self.kf.correct(measurement)

        return (
            float(estimated[0, 0]),
            float(estimated[1, 0]),
        )

    def predict_only(self) -> Tuple[float, float]:
        if not self.initialized:
            return float("nan"), float("nan")

        dt = self._get_dt()
        self._update_transition(dt)

        predicted = self.kf.predict()

        return (
            float(predicted[0, 0]),
            float(predicted[1, 0]),
        )


class PersonDetector(Node):
    def __init__(self) -> None:
        super().__init__("person_detector")

        # ------------------------------------------------------------------
        # ROS 파라미터
        # ------------------------------------------------------------------
        self.declare_parameter(
            "model_path",
            "/home/leejunho/best.pt",
        )
        self.declare_parameter(
            "image_topic",
            "/landing/raw_video",
        )
        self.declare_parameter(
            "frame_id",
            "camera_frame",
        )
        self.declare_parameter(
            "airframe",
            "x500_lidar_down_0",
        )
        self.declare_parameter(
            "world",
            "lying_person_test",
        )
        self.declare_parameter(
            "lidar_topic",
            "",
        )
        self.declare_parameter(
            "lidar_altitude",
            0.17,
        )

        # 첨부 코드에서 사용한 카메라 내부 파라미터
        self.declare_parameter(
            "camera_fx",
            827.99145461,
        )
        self.declare_parameter(
            "camera_fy",
            826.30893069,
        )

        self.declare_parameter(
            "confidence_threshold",
            0.5,
        )
        self.declare_parameter(
            "image_size",
            480,
        )
        self.declare_parameter(
            "device",
            "0",
        )
        self.declare_parameter(
            "show_window",
            True,
        )
        self.declare_parameter(
            "use_filter",
            True,
        )
        self.declare_parameter(
            "target_kf_process_var",
            0.01,
        )
        self.declare_parameter(
            "target_kf_measurement_var",
            0.08,
        )
        self.declare_parameter(
            "target_predict_timeout",
            5.0,
        )
        self.declare_parameter(
            "expected_camera_rate_hz",
            30.0,
        )
        self.declare_parameter(
            "minimum_valid_altitude",
            0.05,
        )

        model_path = str(
            self.get_parameter("model_path").value
        )
        image_topic = str(
            self.get_parameter("image_topic").value
        )

        self.frame_id = str(
            self.get_parameter("frame_id").value
        )
        self.camera_fx = float(
            self.get_parameter("camera_fx").value
        )
        self.camera_fy = float(
            self.get_parameter("camera_fy").value
        )
        self.lidar_altitude = float(
            self.get_parameter("lidar_altitude").value
        )
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.image_size = int(
            self.get_parameter("image_size").value
        )
        self.device = str(
            self.get_parameter("device").value
        )
        self.show_window = bool(
            self.get_parameter("show_window").value
        )
        self.use_filter = bool(
            self.get_parameter("use_filter").value
        )
        self.target_kf_process_var = float(
            self.get_parameter("target_kf_process_var").value
        )
        self.target_kf_measurement_var = float(
            self.get_parameter("target_kf_measurement_var").value
        )
        self.target_predict_timeout = float(
            self.get_parameter("target_predict_timeout").value
        )
        self.expected_camera_rate_hz = float(
            self.get_parameter("expected_camera_rate_hz").value
        )
        self.minimum_valid_altitude = float(
            self.get_parameter("minimum_valid_altitude").value
        )

        airframe = str(
            self.get_parameter("airframe").value
        )
        world = str(
            self.get_parameter("world").value
        )
        lidar_topic_parameter = str(
            self.get_parameter("lidar_topic").value
        )

        if lidar_topic_parameter:
            self.lidar_topic = lidar_topic_parameter
        else:
            self.lidar_topic = (
                f"/world/{world}/model/{airframe}"
                "/link/lidar_sensor_link/sensor/lidar/scan/points"
            )

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------------------------------------------------------------
        # ROS 인터페이스
        # ------------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            image_qos,
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            sensor_qos,
        )

        # 기존 호환용:
        # x, y = 정규화된 영상 중심 오차
        # z    = 선택된 bounding box의 영상 면적 비율
        self.error_pub = self.create_publisher(
            Vector3,
            "/person_tracker/error",
            10,
        )

        self.visible_pub = self.create_publisher(
            Bool,
            "/person_tracker/visible",
            10,
        )

        # OBB 장축 방향 각도 [deg], 영상 +x축 기준 0~180도
        # OBB는 방향성이 없으므로 0도와 180도는 같은 축이다.
        self.angle_pub = self.create_publisher(
            Float32,
            "/person_tracker/angle_deg",
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            "/person_tracker/debug_image",
            image_qos,
        )

        # 첨부 코드와 같은 실제 거리 단위 좌표 발행
        # x, y = 칼만필터가 적용된 표적 상대 위치 [m]
        # z    = LiDAR 기준 고도 [m]
        self.coordinate_pub = self.create_publisher(
            PointStamped,
            "/landing/coordinates",
            10,
        )

        self.bridge = CvBridge()

        # ------------------------------------------------------------------
        # YOLO
        # ------------------------------------------------------------------
        self.model = YOLO(model_path)

        self.get_logger().info(
            f"Loaded YOLO task: {self.model.task}"
        )
        self.get_logger().info(
            f"Model classes: {self.model.names}"
        )

        if self.model.task != "obb":
            self.get_logger().warning(
                "Loaded model is not an OBB model. "
                "Expected model.task == 'obb'."
            )

        # ------------------------------------------------------------------
        # 칼만필터 및 추적 상태
        # ------------------------------------------------------------------
        self.target_kf = TargetKalman2D(
            process_var=self.target_kf_process_var,
            measurement_var=self.target_kf_measurement_var,
            default_dt=1.0 / max(self.expected_camera_rate_hz, 1.0),
        )

        self.altitude_m = float("nan")

        self.raw_x_m = float("nan")
        self.raw_y_m = float("nan")
        self.filtered_x_m = float("nan")
        self.filtered_y_m = float("nan")

        self.last_valid_measurement_time: Optional[float] = None
        self.tracking_mode = "none"

        self.predicted_center_x = float("nan")
        self.predicted_center_y = float("nan")

        self.add_on_set_parameters_callback(
            self.on_parameter_update
        )

        # ------------------------------------------------------------------
        # OpenCV 디버그 창
        # ------------------------------------------------------------------
        self.window_name = "YOLO Person Detection"

        if self.show_window:
            cv2.namedWindow(
                self.window_name,
                cv2.WINDOW_NORMAL,
            )
            cv2.resizeWindow(
                self.window_name,
                960,
                720,
            )

        self.get_logger().info(
            "PersonDetector node started."
        )
        self.get_logger().info(
            f"Python executable: {sys.executable}"
        )
        self.get_logger().info(
            f"Image topic: {image_topic}"
        )
        self.get_logger().info(
            f"LiDAR topic: {self.lidar_topic}"
        )
        self.get_logger().info(
            "Target selection: highest confidence detection"
        )
        self.get_logger().info(
            "Metric coordinates: /landing/coordinates"
        )
        self.get_logger().info(
            "Normalized error: /person_tracker/error"
        )
        self.get_logger().info(
            "OBB long-axis angle: /person_tracker/angle_deg"
        )

    # ----------------------------------------------------------------------
    # 파라미터 갱신
    # ----------------------------------------------------------------------
    def recreate_target_kf(self) -> None:
        self.target_kf = TargetKalman2D(
            process_var=self.target_kf_process_var,
            measurement_var=self.target_kf_measurement_var,
            default_dt=1.0 / max(self.expected_camera_rate_hz, 1.0),
        )

        self.raw_x_m = float("nan")
        self.raw_y_m = float("nan")
        self.filtered_x_m = float("nan")
        self.filtered_y_m = float("nan")

        self.last_valid_measurement_time = None
        self.tracking_mode = "none"

        self.predicted_center_x = float("nan")
        self.predicted_center_y = float("nan")

        self.get_logger().info(
            "[TARGET_KF] filter recreated"
        )

    def on_parameter_update(
        self,
        params,
    ) -> SetParametersResult:
        recreate_filter = False

        for param in params:
            if param.name == "target_kf_process_var":
                value = float(param.value)

                if value <= 0.0 or value > 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "target_kf_process_var must be "
                            "in (0.0, 1.0]"
                        ),
                    )

                self.target_kf_process_var = value
                recreate_filter = True

            elif param.name == "target_kf_measurement_var":
                value = float(param.value)

                if value <= 0.0 or value > 10.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "target_kf_measurement_var must be "
                            "in (0.0, 10.0]"
                        ),
                    )

                self.target_kf_measurement_var = value
                recreate_filter = True

            elif param.name == "target_predict_timeout":
                value = float(param.value)

                if value < 0.0 or value > 30.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "target_predict_timeout must be "
                            "between 0.0 and 30.0 seconds"
                        ),
                    )

                self.target_predict_timeout = value

            elif param.name == "use_filter":
                self.use_filter = bool(param.value)

            elif param.name == "camera_fx":
                value = float(param.value)

                if value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="camera_fx must be positive",
                    )

                self.camera_fx = value

            elif param.name == "camera_fy":
                value = float(param.value)

                if value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="camera_fy must be positive",
                    )

                self.camera_fy = value

            elif param.name == "lidar_altitude":
                self.lidar_altitude = float(param.value)

            elif param.name == "minimum_valid_altitude":
                value = float(param.value)

                if value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "minimum_valid_altitude must be "
                            "zero or positive"
                        ),
                    )

                self.minimum_valid_altitude = value

        if recreate_filter:
            self.recreate_target_kf()

        return SetParametersResult(successful=True)

    # ----------------------------------------------------------------------
    # LiDAR
    # ----------------------------------------------------------------------
    def lidar_callback(
        self,
        msg: PointCloud2,
    ) -> None:
        if len(msg.data) < 4:
            self.get_logger().warning(
                "LiDAR PointCloud2 data is shorter than 4 bytes."
            )
            self.altitude_m = float("nan")
            return

        try:
            endian = ">" if msg.is_bigendian else "<"
            raw_distance = struct.unpack(
                f"{endian}f",
                bytes(msg.data[:4]),
            )[0]

            corrected_altitude = (
                float(raw_distance)
                - self.lidar_altitude
            )

            if math.isfinite(corrected_altitude):
                self.altitude_m = corrected_altitude
            else:
                self.altitude_m = float("nan")

        except (struct.error, ValueError) as error:
            self.get_logger().warning(
                f"LiDAR conversion failed: {error}"
            )
            self.altitude_m = float("nan")

    # ----------------------------------------------------------------------
    # 영상 처리
    # ----------------------------------------------------------------------
    def image_callback(
        self,
        msg: Image,
    ) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"Image conversion failed: {error}"
            )
            return

        height, width = frame.shape[:2]

        image_center_x = width / 2.0
        image_center_y = height / 2.0

        try:
            results = self.model(
                frame,
                imgsz=self.image_size,
                conf=self.confidence_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as error:
            self.get_logger().error(
                f"YOLO inference failed: {error}"
            )
            return

        result = results[0]

        # OBB 모델의 결과는 result.boxes가 아니라 result.obb에 저장된다.
        obb = result.obb

        # 모든 OBB를 자동으로 그리지 않고,
        # 신뢰도가 가장 높은 추적 대상 하나만 직접 표시한다.
        annotated_frame = frame.copy()

        visible_msg = Bool()
        detection_found = (
            obb is not None
            and len(obb) > 0
        )

        target_confidence = float("nan")
        target_size_ratio = float("nan")
        target_angle_rad = float("nan")
        target_angle_deg = float("nan")
        normalized_dx = float("nan")
        normalized_dy = float("nan")

        target_polygon: Optional[np.ndarray] = None
        measured_center: Optional[
            Tuple[float, float]
        ] = None

        if detection_found:
            # [cx, cy, width, height, rotation(rad)]
            xywhr = (
                obb.xywhr
                .detach()
                .cpu()
                .numpy()
            )

            # 회전 사각형의 꼭짓점 4개: shape=(N, 4, 2)
            polygons = (
                obb.xyxyxyxy
                .detach()
                .cpu()
                .numpy()
            )

            confidences = (
                obb.conf
                .detach()
                .cpu()
                .numpy()
            )

            # 동일 클래스가 여러 개 검출되면
            # 신뢰도가 가장 높은 객체 하나만 선택
            target_index = int(
                np.argmax(confidences)
            )

            (
                target_center_x,
                target_center_y,
                target_width,
                target_height,
                target_angle_rad,
            ) = xywhr[target_index]

            target_polygon = (
                polygons[target_index]
                .round()
                .astype(np.int32)
            )

            # 화면에 실제로 그려지는 OBB의 가장 긴 변을 기준으로
            # 장축 각도를 계산한다. 영상 +x축(오른쪽)이 0도이고,
            # 영상 아래쪽 방향으로 각도가 증가하며 범위는 0~180도이다.
            target_angle_deg = self.compute_long_axis_angle_deg(
                target_polygon
            )-90.0

            target_confidence = float(
                confidences[target_index]
            )

            measured_center = (
                float(target_center_x),
                float(target_center_y),
            )

            pixel_dx = (
                target_center_x
                - image_center_x
            )
            pixel_dy = (
                image_center_y
                - target_center_y
            )

            normalized_dx = (
                pixel_dx
                / (width / 2.0)
            )
            normalized_dy = (
                pixel_dy
                / (height / 2.0)
            )

            # OBB 면적 = width * height
            target_size_ratio = (
                float(target_width * target_height)
                / float(width * height)
            )

            # 기존 제어 코드 호환용 정규화 오차
            error_msg = Vector3()
            error_msg.x = float(normalized_dx)
            error_msg.y = float(normalized_dy)
            error_msg.z = float(target_size_ratio)
            self.error_pub.publish(error_msg)

            visible_msg.data = True

            # 픽셀 오차를 LiDAR 고도와 초점거리로 실제 거리[m] 변환
            if self.valid_altitude():
                self.raw_x_m = (
                    float(pixel_dx)
                    / self.camera_fx
                    * self.altitude_m
                )
                self.raw_y_m = (
                    float(pixel_dy)
                    / self.camera_fy
                    * self.altitude_m
                )

                if self.use_filter:
                    (
                        self.filtered_x_m,
                        self.filtered_y_m,
                    ) = self.target_kf.update(
                        self.raw_x_m,
                        self.raw_y_m,
                    )
                else:
                    self.filtered_x_m = self.raw_x_m
                    self.filtered_y_m = self.raw_y_m

                self.last_valid_measurement_time = (
                    time.monotonic()
                )
                self.tracking_mode = "measurement"

                self.predicted_center_x = float("nan")
                self.predicted_center_y = float("nan")

            else:
                self.raw_x_m = float("nan")
                self.raw_y_m = float("nan")
                self.handle_missing_metric_measurement(
                    image_center_x=image_center_x,
                    image_center_y=image_center_y,
                )

        else:
            visible_msg.data = False
            self.raw_x_m = float("nan")
            self.raw_y_m = float("nan")

            self.handle_missing_metric_measurement(
                image_center_x=image_center_x,
                image_center_y=image_center_y,
            )

        self.visible_pub.publish(visible_msg)

        angle_msg = Float32()
        angle_msg.data = float(target_angle_deg)
        self.angle_pub.publish(angle_msg)

        self.publish_coordinates(msg)
        self.draw_debug_overlay(
            frame=annotated_frame,
            image_center_x=image_center_x,
            image_center_y=image_center_y,
            detection_found=detection_found,
            target_polygon=target_polygon,
            measured_center=measured_center,
            target_confidence=target_confidence,
            target_angle_rad=target_angle_rad,
            target_angle_deg=target_angle_deg,
            target_size_ratio=target_size_ratio,
            normalized_dx=normalized_dx,
            normalized_dy=normalized_dy,
        )
        self.publish_debug_image(
            annotated_frame,
            msg,
        )
        self.show_debug_window(
            annotated_frame,
        )

    @staticmethod
    def compute_long_axis_angle_deg(
        polygon: np.ndarray,
    ) -> float:
        """
        OBB 꼭짓점에서 가장 긴 변의 축 각도를 계산한다.

        반환 범위는 [0, 180)도이다. OBB에는 앞/뒤 방향성이 없으므로
        theta와 theta + 180도는 동일한 방향으로 취급한다.
        """
        points = np.asarray(polygon, dtype=np.float32).reshape(4, 2)

        edge_vectors = np.roll(points, -1, axis=0) - points
        edge_lengths_sq = np.sum(edge_vectors * edge_vectors, axis=1)
        longest_edge = edge_vectors[int(np.argmax(edge_lengths_sq))]

        dx = float(longest_edge[0])
        dy = float(longest_edge[1])

        if dx == 0.0 and dy == 0.0:
            return float("nan")

        return math.degrees(math.atan2(dy, dx)) % 180.0

    def valid_altitude(self) -> bool:
        return (
            math.isfinite(self.altitude_m)
            and self.altitude_m
            > self.minimum_valid_altitude
        )

    def handle_missing_metric_measurement(
        self,
        image_center_x: float,
        image_center_y: float,
    ) -> None:
        now = time.monotonic()

        can_predict = (
            self.use_filter
            and self.target_kf.initialized
            and self.last_valid_measurement_time is not None
            and (
                now
                - self.last_valid_measurement_time
                <= self.target_predict_timeout
            )
        )

        if can_predict:
            (
                self.filtered_x_m,
                self.filtered_y_m,
            ) = self.target_kf.predict_only()

            self.tracking_mode = "prediction"

            if (
                self.valid_altitude()
                and math.isfinite(self.filtered_x_m)
                and math.isfinite(self.filtered_y_m)
            ):
                self.predicted_center_x = (
                    image_center_x
                    + (
                        self.filtered_x_m
                        / self.altitude_m
                        * self.camera_fx
                    )
                )
                self.predicted_center_y = (
                    image_center_y
                    - (
                        self.filtered_y_m
                        / self.altitude_m
                        * self.camera_fy
                    )
                )
            else:
                self.predicted_center_x = float("nan")
                self.predicted_center_y = float("nan")

        else:
            self.filtered_x_m = float("nan")
            self.filtered_y_m = float("nan")

            self.predicted_center_x = float("nan")
            self.predicted_center_y = float("nan")

            self.tracking_mode = "none"

            if self.target_kf.initialized:
                self.target_kf.reset()

            self.last_valid_measurement_time = None

    # ----------------------------------------------------------------------
    # 발행
    # ----------------------------------------------------------------------
    def publish_coordinates(
        self,
        original_msg: Image,
    ) -> None:
        coordinate_msg = PointStamped()
        coordinate_msg.header.stamp = (
            original_msg.header.stamp
        )
        coordinate_msg.header.frame_id = (
            self.frame_id
        )

        coordinate_msg.point.x = float(
            self.filtered_x_m
        )
        coordinate_msg.point.y = float(
            self.filtered_y_m
        )
        coordinate_msg.point.z = float(
            self.altitude_m
        )

        self.coordinate_pub.publish(
            coordinate_msg
        )

    def publish_debug_image(
        self,
        frame: np.ndarray,
        original_msg: Image,
    ) -> None:
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8",
            )
            debug_msg.header = original_msg.header

            self.debug_image_pub.publish(
                debug_msg
            )

        except Exception as error:
            self.get_logger().error(
                f"Debug image publish failed: {error}"
            )

    # ----------------------------------------------------------------------
    # 디버그 표시
    # ----------------------------------------------------------------------
    def draw_debug_overlay(
        self,
        frame: np.ndarray,
        image_center_x: float,
        image_center_y: float,
        detection_found: bool,
        target_polygon: Optional[np.ndarray],
        measured_center: Optional[
            Tuple[float, float]
        ],
        target_confidence: float,
        target_angle_rad: float,
        target_angle_deg: float,
        target_size_ratio: float,
        normalized_dx: float,
        normalized_dy: float,
    ) -> None:
        # 영상 중심
        cv2.drawMarker(
            frame,
            (
                int(image_center_x),
                int(image_center_y),
            ),
            (255, 0, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )

        if (
            detection_found
            and target_polygon is not None
            and measured_center is not None
        ):
            target_center_x, target_center_y = (
                measured_center
            )

            # OBB 회전 사각형 표시
            cv2.polylines(
                frame,
                [target_polygon.reshape((-1, 1, 2))],
                isClosed=True,
                color=(0, 255, 0),
                thickness=3,
            )

            # 실제 YOLO 측정 중심
            cv2.drawMarker(
                frame,
                (
                    int(target_center_x),
                    int(target_center_y),
                ),
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=24,
                thickness=2,
            )

            cv2.line(
                frame,
                (
                    int(image_center_x),
                    int(image_center_y),
                ),
                (
                    int(target_center_x),
                    int(target_center_y),
                ),
                (0, 255, 255),
                2,
            )

            # OBB 장축을 자홍색 선으로 표시한다.
            if math.isfinite(target_angle_deg):
                axis_rad = math.radians(target_angle_deg)
                polygon_points = target_polygon.reshape(4, 2).astype(np.float32)
                edge_vectors = (
                    np.roll(polygon_points, -1, axis=0)
                    - polygon_points
                )
                axis_half_length = 0.5 * math.sqrt(
                    float(np.max(np.sum(edge_vectors * edge_vectors, axis=1)))
                )
                axis_dx = axis_half_length * math.cos(axis_rad)
                axis_dy = axis_half_length * math.sin(axis_rad)

                cv2.line(
                    frame,
                    (
                        int(target_center_x - axis_dx),
                        int(target_center_y - axis_dy),
                    ),
                    (
                        int(target_center_x + axis_dx),
                        int(target_center_y + axis_dy),
                    ),
                    (255, 0, 255),
                    3,
                )

            status_text = "PERSON DETECTED"
            status_color = (0, 255, 0)

        else:
            status_text = "PERSON NOT DETECTED"
            status_color = (0, 0, 255)

        # 칼만 예측 중심
        if (
            self.tracking_mode == "prediction"
            and math.isfinite(self.predicted_center_x)
            and math.isfinite(self.predicted_center_y)
        ):
            cv2.drawMarker(
                frame,
                (
                    int(self.predicted_center_x),
                    int(self.predicted_center_y),
                ),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=24,
                thickness=2,
            )

        cv2.putText(
            frame,
            status_text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color,
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            (
                f"mode={self.tracking_mode}  "
                f"confidence={target_confidence:.2f}"
            ),
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            (
                f"OBB axis angle={target_angle_deg:.1f} deg  "
                f"raw r={math.degrees(target_angle_rad):.1f} deg"
            ),
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            (
                f"norm dx={normalized_dx:+.3f}  "
                f"dy={normalized_dy:+.3f}  "
                f"size={target_size_ratio:.4f}"
            ),
            (20, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            (
                f"raw metric x={self.raw_x_m:+.3f}  "
                f"y={self.raw_y_m:+.3f} m"
            ),
            (20, 175),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            (
                f"published x={self.filtered_x_m:+.3f}  "
                f"y={self.filtered_y_m:+.3f} m"
            ),
            (20, 210),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"altitude={self.altitude_m:.3f} m",
            (20, 245),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def show_debug_window(
        self,
        frame: np.ndarray,
    ) -> None:
        if not self.show_window:
            return

        cv2.imshow(
            self.window_name,
            frame,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.show_window = False
            cv2.destroyWindow(
                self.window_name
            )

    def destroy_node(self) -> None:
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[PersonDetector] = None

    try:
        node = PersonDetector()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(
            f"PersonDetector error: {error}"
        )
        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
