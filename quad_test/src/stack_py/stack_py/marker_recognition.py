#!/usr/bin/env python3
from __future__ import annotations

import os
import math
import socket
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, DistanceSensor, VehicleAttitude
from smbus import SMBus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib  # noqa: E402


# --------------------------------------------------------------------------- #
#  GStreamer pipelines
#
#  Jetson Orin Nano has NO hardware video ENCODER (no NVENC).
#    - decode : nvv4l2decoder      -> hardware, use it
#    - encode : x264enc (software) -> only real option, keep the load low
# --------------------------------------------------------------------------- #

def build_gst_src_pipeline(rtsp_url: str, hw_decode: bool = True) -> str:
    """A8 mini H.265 RTSP -> BGR frames."""
    if hw_decode:
        decode = "nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! "
    else:
        decode = "avdec_h265 ! "

    return (
        f"rtspsrc location={rtsp_url} "
        "protocols=GST_RTSP_LOWER_TRANS_UDP "
        "latency=50 drop-on-latency=true do-retransmission=false ! "
        "rtph265depay ! h265parse ! "
        f"{decode}"
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1 sync=false"
    )


def build_rtsp_launch(
    width: int,
    height: int,
    fps: int,
    bitrate_bps: int,
) -> str:
    """appsrc (BGR debug frames) -> x264 (software) -> RTP H.264."""
    bitrate_kbps = max(200, bitrate_bps // 1000)

    return (
        "( "
        "appsrc name=source is-live=true block=false "
        "format=GST_FORMAT_TIME do-timestamp=true stream-type=0 "
        f"caps=video/x-raw,format=BGR,width={width},height={height},"
        f"framerate={fps}/1 ! "
        "queue max-size-buffers=2 leaky=downstream ! "
        "videoconvert ! video/x-raw,format=I420 ! "
        "x264enc "
        "tune=zerolatency "
        "speed-preset=ultrafast "
        f"bitrate={bitrate_kbps} "
        f"key-int-max={fps} "
        "bframes=0 "
        "! video/x-h264,profile=baseline ! "
        "h264parse config-interval=1 ! "
        "rtph264pay name=pay0 pt=96 config-interval=1 "
        ")"
    )


class DebugRtspFactory(GstRtspServer.RTSPMediaFactory):
    """RTSP media factory that lets the node push OpenCV frames into appsrc.

    Frames are pushed from the ROS camera timer (real-time paced at stream_fps)
    and appsrc stamps each buffer with the pipeline running-time via
    do-timestamp=true, so the stream stays wall-clock paced with minimal
    latency.  push_frame() is a no-op until a client connects and
    media-configure hands us the appsrc.
    """

    def __init__(self, launch: str, logger) -> None:
        super().__init__()

        self._logger = logger
        self._lock = threading.Lock()
        self._appsrc = None

        self.set_launch(launch)
        self.set_shared(True)
        self.set_latency(0)

        self.connect("media-configure", self._on_media_configure)

    def _on_media_configure(self, factory, media) -> None:
        appsrc = media.get_element().get_by_name_recurse_up("source")

        with self._lock:
            self._appsrc = appsrc

        self._logger.info("[RTSP] client connected, appsrc ready")

    def has_client(self) -> bool:
        with self._lock:
            return self._appsrc is not None

    def push_frame(self, frame: np.ndarray) -> None:
        with self._lock:
            appsrc = self._appsrc

        if appsrc is None:
            return

        data = frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)

        ret = appsrc.emit("push-buffer", buf)

        if ret != Gst.FlowReturn.OK and ret == Gst.FlowReturn.FLUSHING:
            with self._lock:
                self._appsrc = None


class SiyiA8MiniUDP:
    def __init__(self, ip: str, port: int = 37260, timeout: float = 0.2):
        self.ip = ip
        self.port = port
        self.seq = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass

    def crc16_ccitt(self, data: bytes, init: int = 0x0000) -> int:
        crc = init

        for b in data:
            crc ^= b << 8

            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF

        return crc

    def build_packet(self, cmd_id: int, payload: bytes = b"", need_ack: bool = True) -> bytes:
        stx = bytes([0x55, 0x66])
        ctrl = bytes([0x01 if need_ack else 0x00])
        data_len = len(payload).to_bytes(2, byteorder="little")
        seq = self.seq.to_bytes(2, byteorder="little")
        cmd = bytes([cmd_id])

        packet_without_crc = stx + ctrl + data_len + seq + cmd + payload
        crc = self.crc16_ccitt(packet_without_crc)
        crc_bytes = crc.to_bytes(2, byteorder="little")

        self.seq = (self.seq + 1) & 0xFFFF

        return packet_without_crc + crc_bytes

    def send_packet(self, packet: bytes) -> Optional[bytes]:
        self.sock.sendto(packet, (self.ip, self.port))

        try:
            data, _ = self.sock.recvfrom(1024)
            return data
        except socket.timeout:
            return None

    def absolute_zoom(self, zoom: float) -> Optional[bytes]:
        zoom = float(zoom)

        if zoom < 1.0:
            zoom = 1.0
        if zoom > 30.0:
            zoom = 30.0

        zoom_int = int(zoom)
        zoom_decimal = int(round((zoom - zoom_int) * 10.0))

        if zoom_decimal > 9:
            zoom_int += 1
            zoom_decimal = 0

        payload = bytes([zoom_int, zoom_decimal])
        packet = self.build_packet(cmd_id=0x0F, payload=payload, need_ack=True)

        return self.send_packet(packet)


class TargetKalman2D:
    """
    2D target Kalman filter.

    State:
        x = [x_m, y_m, vx_mps, vy_mps]^T

    Measurement:
        z = [raw_x_m, raw_y_m]^T
    """

    def __init__(
        self,
        process_var: float = 0.01,
        measurement_var: float = 0.08,
        default_dt: float = 1.0 / 30.0,
    ) -> None:
        self.kf = cv2.KalmanFilter(4, 2)

        self.default_dt = default_dt
        self.initialized = False
        self.last_time = time.monotonic()

        self.kf.transitionMatrix = np.eye(4, dtype=np.float32)

        self.kf.measurementMatrix = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
            ],
            dtype=np.float32,
        )

        self.kf.processNoiseCov = np.array(
            [
                [process_var, 0, 0, 0],
                [0, process_var, 0, 0],
                [0, 0, process_var * 10.0, 0],
                [0, 0, 0, process_var * 10.0],
            ],
            dtype=np.float32,
        )

        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * measurement_var
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

    def reset(self) -> None:
        self.initialized = False
        self.last_time = time.monotonic()

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
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

    def update(self, raw_x_m: float, raw_y_m: float) -> Tuple[float, float]:
        dt = self._get_dt()
        self._update_transition(dt)

        if not math.isfinite(raw_x_m) or not math.isfinite(raw_y_m):
            return self.predict_only()

        if not self.initialized:
            self.kf.statePost = np.array(
                [
                    [raw_x_m],
                    [raw_y_m],
                    [0.0],
                    [0.0],
                ],
                dtype=np.float32,
            )
            self.initialized = True
            return raw_x_m, raw_y_m

        self.kf.predict()

        measurement = np.array(
            [
                [raw_x_m],
                [raw_y_m],
            ],
            dtype=np.float32,
        )

        estimated = self.kf.correct(measurement)

        return float(estimated[0, 0]), float(estimated[1, 0])

    def predict_only(self) -> Tuple[float, float]:
        if not self.initialized:
            return float("nan"), float("nan")

        dt = self._get_dt()
        self._update_transition(dt)

        predicted = self.kf.predict()

        return float(predicted[0, 0]), float(predicted[1, 0])


class MarkerRecognition(Node):

    I2C_BUS = 7
    LIDAR_ADDR = 0x62

    ACQ_COMMAND = 0x00
    STATUS = 0x01
    DISTANCE_HIGH = 0x0F
    DISTANCE_LOW = 0x10

    _ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h10)

    try:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    except AttributeError:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

    # AprilTag families detect best with AprilTag-specific corner refinement:
    # it sharpens corner/centre localisation and improves pose accuracy.
    _ARUCO_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

    _CAMERA_MATRIX_1X = np.array([
        [735.73139009, 0.0, 642.68011744],
        [0.0, 734.73302999, 375.24685578],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    _DIST_COEFFS_1X = np.array(
        [-0.1088407, 0.12443118, 0.00029209, 0.00033456, -0.04728482],
        dtype=np.float64,
    )

    _CAMERA_MATRIX_4X = np.array([
        [1355.46604, 0.0, 612.878333],
        [0.0, 1348.64952, 347.526288],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    _DIST_COEFFS_4X = np.array(
        [-0.21032195, 0.37100331, -0.00570627, -0.0043965, -0.39666385],
        dtype=np.float64,
    )

    def __init__(self) -> None:
        super().__init__("marker_recognition")

        os.environ.setdefault("GST_DEBUG", "2")

        # ---------------- parameters ----------------
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("debug", True)
        self.declare_parameter("show_window", False)
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("lidar_altitude", 0.17)
        self.declare_parameter("lidar_attitude_gate_deg", 20.0)

        self.declare_parameter("siyi_ip", "192.168.144.25")
        self.declare_parameter("siyi_port", 37260)
        self.declare_parameter("auto_zoom_threshold_m", 10.0)
        self.declare_parameter("auto_zoom_factor", 4.0)
        self.declare_parameter("auto_zoom_enable", True)

        self.declare_parameter("target_kf_process_var", 0.01)
        self.declare_parameter("target_kf_measurement_var", 0.08)
        self.declare_parameter("target_predict_timeout", 5.0)

        # ---- video in ----
        self.declare_parameter("camera_rtsp_url", "rtsp://192.168.144.25:8554/main.264")
        self.declare_parameter("hw_decode", True)

        # ---- video out (RTSP debug server) ----
        self.declare_parameter("rtsp_port", 8555)
        self.declare_parameter("rtsp_mount", "/debug")
        self.declare_parameter("stream_bitrate", 2000000)   # 2 Mbps, HM30-safe
        self.declare_parameter("stream_scale", 0.5)         # 720p -> 640x360
        self.declare_parameter("stream_fps", 15)            # x264 CPU 절약

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_debug = bool(self.get_parameter("debug").value)
        self._show_window = bool(self.get_parameter("show_window").value)
        self.frame_rate = float(self.get_parameter("frame_rate").value)
        self._lidar_altitude = float(self.get_parameter("lidar_altitude").value)

        self._siyi_ip = str(self.get_parameter("siyi_ip").value)
        self._siyi_port = int(self.get_parameter("siyi_port").value)
        self._auto_zoom_threshold_m = float(self.get_parameter("auto_zoom_threshold_m").value)
        self._auto_zoom_factor = float(self.get_parameter("auto_zoom_factor").value)
        self._auto_zoom_enable = bool(self.get_parameter("auto_zoom_enable").value)
        self._lidar_attitude_gate_deg = float(
            self.get_parameter("lidar_attitude_gate_deg").value
        )

        self._target_kf_process_var = float(
            self.get_parameter("target_kf_process_var").value
        )
        self._target_kf_measurement_var = float(
            self.get_parameter("target_kf_measurement_var").value
        )
        self._target_predict_timeout = float(
            self.get_parameter("target_predict_timeout").value
        )

        self._camera_rtsp_url = str(self.get_parameter("camera_rtsp_url").value)
        self._hw_decode = bool(self.get_parameter("hw_decode").value)

        self._rtsp_port = int(self.get_parameter("rtsp_port").value)
        self._rtsp_mount = str(self.get_parameter("rtsp_mount").value)
        self._stream_bitrate = int(self.get_parameter("stream_bitrate").value)
        self._stream_scale = float(self.get_parameter("stream_scale").value)
        self._stream_fps = int(self.get_parameter("stream_fps").value)

        self.add_on_set_parameters_callback(self._on_param_update)

        # ---------------- SIYI zoom control ----------------
        self._siyi = SiyiA8MiniUDP(
            ip=self._siyi_ip,
            port=self._siyi_port,
            timeout=0.2,
        )

        self.get_logger().info(
            f"SIYI A8 mini UDP control target: {self._siyi_ip}:{self._siyi_port}"
        )

        # ---------------- lidar ----------------
        self._i2c_bus = SMBus(self.I2C_BUS)
        self._lidar_timer = self.create_timer(0.05, self._lidar_timer_cb)

        # ---------------- state ----------------
        self.x_m = 0.0
        self.y_m = 0.0

        self.raw_x_m = float("nan")
        self.raw_y_m = float("nan")

        self._filtered_altitude: Optional[float] = None
        self._altitude = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._have_attitude = False
        self._last_lidar_log_time = 0.0
        self._lidar_log_interval = 0.5
        self._last_detect_preprocess = "none"
        self._lidar_reject_count = 0

        self._current_zoom_factor = 1.0

        self._target_kf = TargetKalman2D(
            process_var=self._target_kf_process_var,
            measurement_var=self._target_kf_measurement_var,
            default_dt=1.0 / self.frame_rate,
        )

        self._last_target_detect_time: Optional[float] = None
        self._target_display_mode = "none"
        self._pred_cx = float("nan")
        self._pred_cy = float("nan")

        # stream frame pacing (decouple stream fps from processing fps)
        self._last_stream_push = 0.0
        self._stream_period = 1.0 / max(1, self._stream_fps)

        # ---------------- camera capture ----------------
        src_pipeline = build_gst_src_pipeline(
            self._camera_rtsp_url,
            hw_decode=self._hw_decode,
        )

        self.get_logger().info(f"Opening camera pipeline:\n{src_pipeline}")

        self._cap = cv2.VideoCapture(src_pipeline, cv2.CAP_GSTREAMER)

        if not self._cap.isOpened():
            self.get_logger().error("Unable to open camera (GStreamer pipeline failed)")
            raise RuntimeError("Camera open failed")

        ok, first_frame = self._cap.read()

        if not ok or first_frame is None:
            self.get_logger().error("Camera opened but first frame read failed")
            raise RuntimeError("Camera first frame failed")

        self._frame_h, self._frame_w = first_frame.shape[:2]

        # x264 needs even dimensions
        self._stream_w = int(self._frame_w * self._stream_scale) // 2 * 2
        self._stream_h = int(self._frame_h * self._stream_scale) // 2 * 2

        self.get_logger().info(
            f"Camera stream: {self._frame_w}x{self._frame_h}  ->  "
            f"debug stream: {self._stream_w}x{self._stream_h} @ {self._stream_fps} fps, "
            f"{self._stream_bitrate // 1000} kbps (x264 software)"
        )

        # ---------------- RTSP debug server ----------------
        self._rtsp_factory: Optional[DebugRtspFactory] = None
        self._rtsp_server = None
        self._glib_loop = None
        self._glib_thread = None

        if self._publish_debug:
            self._start_rtsp_server()

        # ---------------- ROS pubs / subs ----------------
        self._gimbal_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        distance_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._distance_sensor_pub = self.create_publisher(
            DistanceSensor,
            "/fmu/in/distance_sensor",
            distance_qos,
        )

        self._gimbal_configured = False
        self._gimbal_tick = 0

        attitude_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._attitude_cb,
            attitude_qos,
        )

        self._pub_point = self.create_publisher(PointStamped, "/nodes/marker/target", 10)

        # ---------------- timers ----------------
        self._gimbal_timer = self.create_timer(1.0, self._set_gimbal_pitch_down)
        self._camera_timer = self.create_timer(1.0 / self.frame_rate, self._camera_timer_cb)

        self._camera_matrix_1x = self._CAMERA_MATRIX_1X.copy()
        self._dist_coeffs_1x = self._DIST_COEFFS_1X.copy()

        self._camera_matrix_4x = self._CAMERA_MATRIX_4X.copy()
        self._dist_coeffs_4x = self._DIST_COEFFS_4X.copy()

        self.get_logger().info("Using hard-coded 1x / 4x camera calibrations.")
        self.get_logger().info(
            f"Target KF enabled: "
            f"process_var={self._target_kf_process_var:.4f}, "
            f"measurement_var={self._target_kf_measurement_var:.4f}, "
            f"predict_timeout={self._target_predict_timeout:.1f}s"
        )

    # ----------------------------------------------------------------- #
    #  RTSP debug server
    # ----------------------------------------------------------------- #

    def _start_rtsp_server(self) -> None:
        Gst.init(None)

        launch = build_rtsp_launch(
            width=self._stream_w,
            height=self._stream_h,
            fps=self._stream_fps,
            bitrate_bps=self._stream_bitrate,
        )

        self.get_logger().info(f"RTSP factory launch:\n{launch}")

        self._rtsp_factory = DebugRtspFactory(launch, self.get_logger())

        self._rtsp_server = GstRtspServer.RTSPServer()
        self._rtsp_server.set_service(str(self._rtsp_port))

        mounts = self._rtsp_server.get_mount_points()
        mounts.add_factory(self._rtsp_mount, self._rtsp_factory)

        self._rtsp_server.attach(None)

        self._glib_loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(
            target=self._glib_loop.run,
            daemon=True,
        )
        self._glib_thread.start()

        local_ip = self._get_local_ip()

        self.get_logger().info(
            f"[RTSP] debug stream ready:  "
            f"rtsp://{local_ip}:{self._rtsp_port}{self._rtsp_mount}"
        )

    def _get_local_ip(self) -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "<jetson-ip>"

    def _stop_rtsp_server(self) -> None:
        if self._glib_loop is not None:
            try:
                self._glib_loop.quit()
            except Exception:
                pass

    def _push_debug_frame(self, frame: np.ndarray) -> None:
        """Rate-limit + downscale before handing the frame to x264.

        NOTE: do NOT gate this on "is a client connected".  The RTSP server
        needs frames flowing into appsrc in order to preroll the pipeline when
        a client sends DESCRIBE.  Gating on the client creates a deadlock:
        no preroll -> no media-configure -> no appsrc -> no frames -> no preroll.
        push_frame() itself is a no-op while appsrc is None, so the cost when
        nobody is watching is just the resize.
        """
        if self._rtsp_factory is None:
            return

        now = time.monotonic()

        if now - self._last_stream_push < self._stream_period:
            return

        self._last_stream_push = now

        if (frame.shape[1], frame.shape[0]) != (self._stream_w, self._stream_h):
            frame = cv2.resize(
                frame,
                (self._stream_w, self._stream_h),
                interpolation=cv2.INTER_AREA,
            )

        if not frame.flags["C_CONTIGUOUS"]:
            frame = np.ascontiguousarray(frame)

        self._rtsp_factory.push_frame(frame)

    # ----------------------------------------------------------------- #
    #  Target KF
    # ----------------------------------------------------------------- #

    def _recreate_target_kf(self) -> None:
        self._target_kf = TargetKalman2D(
            process_var=self._target_kf_process_var,
            measurement_var=self._target_kf_measurement_var,
            default_dt=1.0 / self.frame_rate,
        )

        self._last_target_detect_time = None
        self._target_display_mode = "none"
        self._pred_cx = float("nan")
        self._pred_cy = float("nan")

        self.x_m = float("nan")
        self.y_m = float("nan")
        self.raw_x_m = float("nan")
        self.raw_y_m = float("nan")

        self.get_logger().info(
            f"[TARGET_KF] recreated: "
            f"process_var={self._target_kf_process_var:.4f}, "
            f"measurement_var={self._target_kf_measurement_var:.4f}"
        )

    # ----------------------------------------------------------------- #
    #  Lidar
    # ----------------------------------------------------------------- #

    def _wait_lidar_ready(self, timeout: float = 0.05) -> bool:
        start = time.time()

        while time.time() - start < timeout:
            status = self._i2c_bus.read_byte_data(self.LIDAR_ADDR, self.STATUS)

            if (status & 0x01) == 0:
                return True

            time.sleep(0.001)

        return False

    def _read_lidar_distance_cm(self) -> int:
        self._i2c_bus.write_byte_data(
            self.LIDAR_ADDR,
            self.ACQ_COMMAND,
            0x04,
        )

        if not self._wait_lidar_ready():
            raise TimeoutError("Lidar Lite v3HP measurement timeout")

        high = self._i2c_bus.read_byte_data(
            self.LIDAR_ADDR,
            self.DISTANCE_HIGH,
        )

        low = self._i2c_bus.read_byte_data(
            self.LIDAR_ADDR,
            self.DISTANCE_LOW,
        )

        distance_cm = (high << 8) | low
        return distance_cm

    def _lidar_timer_cb(self) -> None:
        try:
            distance_cm = self._read_lidar_distance_cm()
            distance_m = distance_cm / 100.0

            raw_altitude = distance_m - self._lidar_altitude

            if not self._is_lidar_attitude_valid():
                if self._filtered_altitude is not None:
                    self._altitude = self._filtered_altitude
                else:
                    self._altitude = raw_altitude

                self.get_logger().warn(
                    f"[LIDAR_GATE] raw={raw_altitude:.3f} m ignored, "
                    f"hold altitude={self._altitude:.3f} m"
                )
            else:
                self._altitude = self._filter_lidar_altitude(raw_altitude)

            if not math.isfinite(self._altitude):
                return

            msg = DistanceSensor()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000

            msg.device_id = 0
            msg.min_distance = 0.05
            msg.max_distance = 40.0
            msg.current_distance = float(self._altitude + self._lidar_altitude)
            msg.variance = 0.0
            msg.type = 0
            msg.h_fov = 0.0
            msg.v_fov = 0.0
            msg.orientation = 25

            try:
                msg.q = [float("nan"), float("nan"), float("nan"), float("nan")]
            except Exception:
                pass

            try:
                msg.signal_quality = 100
            except Exception:
                pass

            now_sec = self.get_clock().now().nanoseconds * 1e-9

            if now_sec - self._last_lidar_log_time >= self._lidar_log_interval:
                self._last_lidar_log_time = now_sec

                self.get_logger().info(
                    f"lidar distance={distance_m:.3f} m, "
                    f"altitude={self._altitude:.3f} m, "
                    f"roll={math.degrees(self._roll):.1f} deg, "
                    f"pitch={math.degrees(self._pitch):.1f} deg"
                )

            self._distance_sensor_pub.publish(msg)
            self._check_auto_zoom_by_altitude_direct_udp()

        except Exception as e:
            self.get_logger().warn(f"lidar read/publish failed: {e}")

    def _check_auto_zoom_by_altitude_direct_udp(self) -> None:
        if not self._auto_zoom_enable:
            return

        if not math.isfinite(self._altitude):
            return

        zoom_in_threshold = self._auto_zoom_threshold_m
        zoom_out_threshold = self._auto_zoom_threshold_m - 0.5

        target_zoom = self._current_zoom_factor

        if self._current_zoom_factor <= 1.01:
            if self._altitude >= zoom_in_threshold:
                target_zoom = self._auto_zoom_factor
        else:
            if self._altitude <= zoom_out_threshold:
                target_zoom = 1.0

        if abs(self._current_zoom_factor - target_zoom) < 0.01:
            return

        ack = self._siyi.absolute_zoom(target_zoom)

        if ack is not None:
            self.get_logger().info(
                f"A8 mini zoom changed: "
                f"{self._current_zoom_factor:.1f}x -> {target_zoom:.1f}x, "
                f"altitude={self._altitude:.2f} m, "
                f"ACK={ack.hex(' ').upper()}"
            )
        else:
            self.get_logger().warn(
                f"A8 mini zoom command sent, but no ACK: "
                f"{self._current_zoom_factor:.1f}x -> {target_zoom:.1f}x, "
                f"altitude={self._altitude:.2f} m"
            )

        self._current_zoom_factor = target_zoom

    # ----------------------------------------------------------------- #
    #  Gimbal
    # ----------------------------------------------------------------- #

    def _publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 191
        msg.from_external = True

        self._gimbal_pub.publish(msg)

    def _set_gimbal_pitch_down(self) -> None:
        self._gimbal_tick += 1

        # LAND 등 다른 모드가 짐벌 제어권(primary control)을 가져가도 곧바로 되찾도록
        # CONFIGURE를 약 3초마다 재전송해 primary control을 계속 유지한다. (1Hz 타이머 기준 3틱)
        if self._gimbal_tick % 3 == 1:
            self._publish_vehicle_command(
                command=VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                param1=1.0,      # primary control sysid
                param2=191.0,    # primary control compid
                param3=-1.0,     # secondary sysid unused
                param4=-1.0,     # secondary compid unused
                param5=0.0,
                param6=0.0,
                param7=154.0,    # gimbal device id
            )

            if not self._gimbal_configured:
                self._gimbal_configured = True
                self.get_logger().info("Sent gimbal configure command")

        # pitch/roll을 지구(수평선) 기준으로 lock -> 기체가 pitch/roll로 기울어도 항상 정하방(-90도) 유지
        # yaw는 lock하지 않아 기체 heading을 따라감 (좌표 변환이 카메라 전방 = 기체 전방을 가정)
        # param5 = ROLL_LOCK(4) | PITCH_LOCK(8) | YAW_IN_VEHICLE_FRAME(32) = 44
        self._publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            param1=-90.0,          # pitch: 지구 기준 절대 -90도 (정하방)
            param2=0.0,            # yaw: 기체 프레임 0도 -> heading 추종
            param3=float("nan"),   # pitch rate: NaN(rate 아님) -> 각도 setpoint 사용. 0.0이면 짐벌이 안 움직임(현 위치 유지)
            param4=float("nan"),   # yaw rate: NaN
            param5=44.0,           # ROLL_LOCK|PITCH_LOCK|YAW_IN_VEHICLE_FRAME
            param6=0.0,
            param7=154.0,          # gimbal device id
        )

    # ----------------------------------------------------------------- #
    #  Main camera loop
    # ----------------------------------------------------------------- #

    def _camera_timer_cb(self) -> None:
        ret, frame = self._cap.read()

        if not ret:
            self.get_logger().error("Frame capture failed")
            return

        detected = False
        tag_centre = self._detect_first_tag(frame)

        camera_matrix, _ = self._get_current_camera_calibration()
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]

        height, width = frame.shape[:2]
        cx0 = width / 2.0
        cy0 = height / 2.0

        z = self._altitude

        if tag_centre is not None:
            detected = True

            cx, cy = tag_centre

            dx_px = cx - cx0
            dy_px = cy0 - cy

            if math.isfinite(z) and z >= 0.05:
                raw_x_m = dx_px / fx * z
                raw_y_m = dy_px / fy * z

                self.raw_x_m = raw_x_m
                self.raw_y_m = raw_y_m

                self.x_m, self.y_m = self._target_kf.update(raw_x_m, raw_y_m)

                self._last_target_detect_time = time.monotonic()
                self._target_display_mode = "raw"
                self._pred_cx = float("nan")
                self._pred_cy = float("nan")
            else:
                self.raw_x_m = float("nan")
                self.raw_y_m = float("nan")
                self.x_m = float("nan")
                self.y_m = float("nan")
                self._target_display_mode = "raw"

            if self._publish_debug:
                cv2.drawMarker(
                    frame,
                    (int(cx), int(cy)),
                    (0, 255, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )

                cv2.drawMarker(
                    frame,
                    (int(cx0), int(cy0)),
                    (255, 0, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )

                cv2.line(
                    frame,
                    (int(cx0), int(cy0)),
                    (int(cx), int(cy)),
                    (0, 255, 0),
                    3,
                    cv2.LINE_AA,
                )

        else:
            self.raw_x_m = float("nan")
            self.raw_y_m = float("nan")

            now = time.monotonic()

            if (
                self._last_target_detect_time is not None
                and now - self._last_target_detect_time <= self._target_predict_timeout
            ):
                self.x_m, self.y_m = self._target_kf.predict_only()
                self._target_display_mode = "predict"

                if (
                    math.isfinite(self.x_m)
                    and math.isfinite(self.y_m)
                    and math.isfinite(z)
                    and z > 0.05
                ):
                    self._pred_cx = cx0 + self.x_m / z * fx
                    self._pred_cy = cy0 - self.y_m / z * fy
                else:
                    self._pred_cx = float("nan")
                    self._pred_cy = float("nan")

            else:
                self.x_m = float("nan")
                self.y_m = float("nan")
                self._pred_cx = float("nan")
                self._pred_cy = float("nan")
                self._target_display_mode = "none"
                self._target_kf.reset()

        if self._publish_debug and self._target_display_mode == "predict":
            if math.isfinite(self._pred_cx) and math.isfinite(self._pred_cy):
                cv2.drawMarker(
                    frame,
                    (int(self._pred_cx), int(self._pred_cy)),
                    (0, 0, 255),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )

        if detected:
            cv2.putText(
                frame,
                f"DETECTED ({self._last_detect_preprocess})",
                (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 0),
                3,
                cv2.LINE_AA,
            )
        elif self._target_display_mode == "predict":
            cv2.putText(
                frame,
                "TARGET LOST - KF PREDICT",
                (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame,
                "NOT DETECTED",
                (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )

        cv2.putText(
            frame,
            f"ALT: {self._altitude:.2f} m",
            (30, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 0, 255),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"raw: {self.raw_x_m:.2f}, {self.raw_y_m:.2f} m",
            (30, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        body_right_preview = self.x_m if math.isfinite(self.x_m) else float("nan")
        body_forward_preview = self.y_m if math.isfinite(self.y_m) else float("nan")

        cv2.putText(
            frame,
            f"cam filt: {self.x_m:.2f}, {self.y_m:.2f} m",
            (30, 220),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"pub body R,F: {body_right_preview:.2f}, {body_forward_preview:.2f} m",
            (30, 260),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        calib_label = "4X" if abs(self._current_zoom_factor - 4.0) < 0.2 else "1X"
        cv2.putText(
            frame,
            f"ZOOM: {self._current_zoom_factor:.1f}x  CALIB: {calib_label}",
            (30, 300),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        # Coordinate convention for /landing/coordinates:
        #   point.x = body right(+), [m]
        #   point.y = body forward(+), [m]
        #   point.z = altitude, [m]
        #
        # The raw vision estimate self.x_m / self.y_m is in CAMERA frame:
        #   self.x_m = camera right(+), [m]
        #   self.y_m = camera forward(+), [m]
        #
        # Current camera mounting:
        #   camera yaw is aligned with body yaw (camera faces the nose).
        #
        # Therefore:
        #   body_right   = camera_right
        #   body_forward = camera_forward
        body_right_m = float(self.x_m)
        body_forward_m = float(self.y_m)

        msg.point.x = body_right_m
        msg.point.y = body_forward_m
        msg.point.z = float(self._altitude)

        self._pub_point.publish(msg)

        # ---- debug frame -> RTSP server (rate-limited + downscaled) ----
        if self._publish_debug:
            self._push_debug_frame(frame)

        if self._show_window:
            cv2.imshow("landing_monitor", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                self.get_logger().info("Monitor window close requested")
                rclpy.shutdown()

    # ----------------------------------------------------------------- #
    #  Detection
    # ----------------------------------------------------------------- #

    def _detect_first_tag(self, frame: np.ndarray) -> Optional[Tuple[float, float]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )

        gray_clahe = clahe.apply(gray)
        gray_clahe = cv2.GaussianBlur(gray_clahe, (3, 3), 0)

        gray_gamma = self._apply_gamma(gray, gamma=1.5)
        gray_gamma = cv2.GaussianBlur(gray_gamma, (3, 3), 0)

        candidates = [
            ("raw", gray),
            ("clahe", gray_clahe),
            ("gamma", gray_gamma),
        ]

        for name, img in candidates:
            corners, ids, _ = self._detect_aruco_from_gray(img)
            center = self._select_largest_marker_center(corners, ids)

            if center is not None:
                self._last_detect_preprocess = name
                return center

        self._last_detect_preprocess = "none"
        return None

    def destroy_node(self) -> bool:
        try:
            self._stop_rtsp_server()
        except Exception:
            pass

        try:
            self._siyi.close()
        except Exception:
            pass

        return super().destroy_node()

    def _attitude_cb(self, msg: VehicleAttitude) -> None:
        q = msg.q

        qw = float(q[0])
        qx = float(q[1])
        qy = float(q[2])
        qz = float(q[3])

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        self._roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)

        if abs(sinp) >= 1.0:
            self._pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self._pitch = math.asin(sinp)

        self._have_attitude = True

    def _is_lidar_attitude_valid(self) -> bool:
        if not self._have_attitude:
            return True

        roll_deg = math.degrees(self._roll)
        pitch_deg = math.degrees(self._pitch)

        gate = self._lidar_attitude_gate_deg

        if abs(roll_deg) > gate or abs(pitch_deg) > gate:
            self.get_logger().warn(
                f"[LIDAR_GATE] hold altitude due to attitude: "
                f"roll={roll_deg:.1f} deg, pitch={pitch_deg:.1f} deg, "
                f"gate={gate:.1f} deg"
            )
            return False

        return True

    def _filter_lidar_altitude(self, raw_altitude: float) -> float:
        if not math.isfinite(raw_altitude):
            self._lidar_reject_count += 1

            self.get_logger().warn(
                f"[LIDAR] reject non-finite altitude, raw={raw_altitude}"
            )

            if self._filtered_altitude is not None:
                return self._filtered_altitude

            return float("nan")

        if raw_altitude < -0.5 or raw_altitude > 40.0:
            self._lidar_reject_count += 1

            self.get_logger().warn(
                f"[LIDAR] reject invalid range: raw={raw_altitude:.2f} m"
            )

            if self._filtered_altitude is not None:
                return self._filtered_altitude

            return float("nan")

        self._lidar_reject_count = 0
        self._filtered_altitude = raw_altitude

        return raw_altitude

    def _on_param_update(self, params):
        for param in params:
            if param.name == "auto_zoom_threshold_m":
                self._auto_zoom_threshold_m = float(param.value)
                self.get_logger().info(
                    f"[PARAM] auto_zoom_threshold_m updated: "
                    f"{self._auto_zoom_threshold_m:.2f} m"
                )

            elif param.name == "auto_zoom_factor":
                value = float(param.value)

                if value < 1.0 or value > 30.0:
                    return SetParametersResult(
                        successful=False,
                        reason="auto_zoom_factor must be between 1.0 and 30.0"
                    )

                self._auto_zoom_factor = value
                self.get_logger().info(
                    f"[PARAM] auto_zoom_factor updated: "
                    f"{self._auto_zoom_factor:.1f}x"
                )

            elif param.name == "auto_zoom_enable":
                self._auto_zoom_enable = bool(param.value)
                self.get_logger().info(
                    f"[PARAM] auto_zoom_enable updated: "
                    f"{self._auto_zoom_enable}"
                )

            elif param.name == "lidar_attitude_gate_deg":
                value = float(param.value)

                if value <= 0.0 or value > 45.0:
                    return SetParametersResult(
                        successful=False,
                        reason="lidar_attitude_gate_deg must be in (0, 45]"
                    )

                self._lidar_attitude_gate_deg = value
                self.get_logger().info(
                    f"[PARAM] lidar_attitude_gate_deg updated: "
                    f"{self._lidar_attitude_gate_deg:.1f} deg"
                )

            elif param.name == "lidar_altitude":
                value = float(param.value)

                if value < 0.0 or value > 2.0:
                    return SetParametersResult(
                        successful=False,
                        reason="lidar_altitude must be between 0.0 and 2.0 m"
                    )

                self._lidar_altitude = value
                self.get_logger().info(
                    f"[PARAM] lidar_altitude updated: "
                    f"{self._lidar_altitude:.3f} m"
                )

            elif param.name == "show_window":
                self._show_window = bool(param.value)
                self.get_logger().info(
                    f"[PARAM] show_window updated: "
                    f"{self._show_window}"
                )

            elif param.name == "debug":
                self._publish_debug = bool(param.value)
                self.get_logger().info(
                    f"[PARAM] debug updated: "
                    f"{self._publish_debug}"
                )

            elif param.name == "frame_rate":
                self.frame_rate = float(param.value)
                self.get_logger().warn(
                    "[PARAM] frame_rate updated, but camera timer period is not "
                    "changed until node restart"
                )

            elif param.name in ("stream_bitrate", "stream_scale", "stream_fps"):
                self.get_logger().warn(
                    f"[PARAM] {param.name} updated, but takes effect only after "
                    "node restart"
                )

            elif param.name == "target_predict_timeout":
                value = float(param.value)

                if value < 0.0 or value > 30.0:
                    return SetParametersResult(
                        successful=False,
                        reason="target_predict_timeout must be between 0.0 and 30.0 s"
                    )

                self._target_predict_timeout = value
                self.get_logger().info(
                    f"[PARAM] target_predict_timeout updated: "
                    f"{self._target_predict_timeout:.1f} s"
                )

            elif param.name == "target_kf_process_var":
                value = float(param.value)

                if value <= 0.0 or value > 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason="target_kf_process_var must be in (0.0, 1.0]"
                    )

                self._target_kf_process_var = value
                self._recreate_target_kf()

            elif param.name == "target_kf_measurement_var":
                value = float(param.value)

                if value <= 0.0 or value > 10.0:
                    return SetParametersResult(
                        successful=False,
                        reason="target_kf_measurement_var must be in (0.0, 10.0]"
                    )

                self._target_kf_measurement_var = value
                self._recreate_target_kf()

        return SetParametersResult(successful=True)

    def _apply_gamma(self, gray: np.ndarray, gamma: float = 1.5) -> np.ndarray:
        table = np.array([
            ((i / 255.0) ** gamma) * 255.0
            for i in range(256)
        ]).astype("uint8")

        return cv2.LUT(gray, table)

    def _detect_aruco_from_gray(self, gray: np.ndarray):
        camera_matrix, dist_coeffs = self._get_current_camera_calibration()

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self._ARUCO_DICT,
            parameters=self._ARUCO_PARAMS,
            cameraMatrix=camera_matrix,
            distCoeff=dist_coeffs,
        )

        return corners, ids, rejected

    def _select_largest_marker_center(self, corners, ids) -> Optional[Tuple[float, float]]:
        if ids is None or len(ids) == 0:
            return None

        best_idx = 0
        best_area = -1.0

        for i, c in enumerate(corners):
            pts = c.reshape(4, 2)
            area = cv2.contourArea(pts.astype(np.float32))

            if area > best_area:
                best_area = area
                best_idx = i

        pts = corners[best_idx].reshape(4, 2)
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))

        return cx, cy

    def _get_current_camera_calibration(self):
        if abs(self._current_zoom_factor - 4.0) < 0.2:
            return self._camera_matrix_4x, self._dist_coeffs_4x

        return self._camera_matrix_1x, self._dist_coeffs_1x


def main(args=None):
    rclpy.init(args=args)

    node: Optional[MarkerRecognition] = None

    try:
        node = MarkerRecognition()
        rclpy.spin(node)

    except Exception:
        import traceback
        traceback.print_exc()

    finally:
        if node is not None:
            if hasattr(node, "_cap") and node._cap is not None:
                node._cap.release()

            node.destroy_node()

        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
