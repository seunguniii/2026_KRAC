#!/usr/bin/env python3
import time
import threading

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SiyiCamNode(Node):
    """
    SIYI A8 mini RTSP 스트림을 받아서
    - 480p로 리사이즈
    - /camera/image_raw 토픽으로 sensor_msgs/Image 퍼블리시
    """

    def __init__(self):
        super().__init__('streaming_test')

        # 파라미터 선언
        self.declare_parameter('rtsp_url', 'rtsp://192.168.144.25:8554/main.264')
        self.declare_parameter('target_height', 480)   # 세로 해상도 (480p)
        self.declare_parameter('frame_rate', 15.0)     # 퍼블리시 목표 FPS

        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.target_height = self.get_parameter('target_height').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value

        # QoS: 센서 데이터용 (low latency)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', qos)
        self.bridge = CvBridge()

        self.get_logger().info(f'RTSP URL: {self.rtsp_url}')
        self.get_logger().info(f'Target height: {self.target_height}')
        self.get_logger().info(f'Frame rate: {self.frame_rate} fps')

        self._stop_flag = False
        self._thread = threading.Thread(target=self.capture_loop, daemon=True)
        self._thread.start()

    def make_gst_pipeline(self) -> str:
        """
        GStreamer 파이프라인 문자열 생성.
        - UDP RTSP
        - 낮은 latency
        - 오래된 프레임 drop
        - avdec_h264 (CPU 디코더; nvv4l2decoder 대신 안정성 우선)
        """
        pipeline = (
            f"rtspsrc location={self.rtsp_url} "
            "protocols=GST_RTSP_LOWER_TRANS_UDP "
            "latency=50 drop-on-latency=true do-retransmission=false ! "
            "rtph264depay ! h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=1 max-buffers=1 sync=false"
        )
        return pipeline

    def capture_loop(self):
        gst = self.make_gst_pipeline()
        self.get_logger().info(f'Using GStreamer pipeline:\n{gst}')

        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().error('Failed to open RTSP stream (VideoCapture is not opened). '
                                    'GStreamer + OpenCV 빌드 확인 필요.')
            return

        period = 1.0 / self.frame_rate if self.frame_rate > 0.0 else 0.0

        while rclpy.ok() and not self._stop_flag:
            loop_start = time.time()

            ret, frame = cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Failed to read frame from RTSP stream')
                time.sleep(0.1)
                continue

            h, w, _ = frame.shape
            if h <= 0 or w <= 0:
                continue

            # 480p로 리사이즈 (세로 기준)
            if self.target_height > 0 and h != self.target_height:
                new_h = self.target_height
                new_w = int(w * (new_h / h))
                frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

            # OpenCV BGR -> ROS Image
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'siyi_a8mini_link'

            self.publisher_.publish(msg)

            if period > 0.0:
                elapsed = time.time() - loop_start
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        cap.release()
        self.get_logger().info('Capture loop stopped, RTSP closed.')

    def stop(self):
        self._stop_flag = True
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SiyiCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
