#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.image_sub = self.create_subscription(
            Image,
            '/landing/video',
            self.image_callback,
            qos
        )

        self.error_pub = self.create_publisher(Vector3, '/person_tracker/error', 10)
        self.visible_pub = self.create_publisher(Bool, '/person_tracker/visible', 10)

        self.bridge = CvBridge()

        # YOLO11 모델 로드 (사람만 보게 classes=[0] 사용 예정)
        self.model = YOLO('/home/leejunho/yolo11n.pt')

        self.get_logger().info('PersonDetector node started.')

    def image_callback(self, msg: Image):
        # ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        cx, cy = w / 2.0, h / 2.0

        # YOLO 추론 (사람 클래스만)
        results = self.model(
            frame,
            imgsz=480,
            conf=0.4,
            classes=[0],   # person만
            device=0,
            verbose=False
        )

        result = results[0]
        boxes = result.boxes

        visible_msg = Bool()
        err_msg = Vector3()

        if boxes is None or len(boxes) == 0:
            # 사람 없음
            visible_msg.data = False
            self.visible_pub.publish(visible_msg)
            # error는 0,0,0으로 두거나 publish 안 해도 됨
            return

        # 가장 큰 bbox (면적 기준) 하나 선택
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()

        areas = []
        for (x1, y1, x2, y2) in xyxy:
            areas.append((x2 - x1) * (y2 - y1))
        idx = int(np.argmax(areas))
        x1, y1, x2, y2 = xyxy[idx]

        # 중심점
        u = 0.5 * (x1 + x2)
        v = 0.5 * (y1 + y2)

        # 정규화된 dx, dy ([-1, 1])
        dx = (u - cx) / (w / 2.0)       # 오른쪽 +, 왼쪽 -
        dy = (cy - v) / (h / 2.0)       # 위쪽 +, 아래쪽 -

        # 사이즈 (0~1)
        size = areas[idx] / float(w * h)

        err_msg.x = float(dx)
        err_msg.y = float(dy)
        err_msg.z = float(size)

        visible_msg.data = True

        self.error_pub.publish(err_msg)
        self.visible_pub.publish(visible_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
