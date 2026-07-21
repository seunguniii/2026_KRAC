#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gcs_button_gui.py
지상 통제소(GCS)용 Human-in-the-Loop Y/N 제어 패널 (PyQt5 + ROS 2)
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, 
                             QVBoxLayout, QHBoxLayout, QLabel, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont


class GcsButtonGui(QWidget, Node):
    def __init__(self):
        # ROS 2 노드 및 PyQt 초기화
        Node.__init__(self, 'gcs_button_gui_node')
        QWidget.__init__(self)

        # 1) ROS 2 퍼블리셔 및 서브스크라이버 설정
        self.user_input_pub = self.create_publisher(String, '/rescue_system/user_input', 10)
        self.state_sub = self.create_subscription(
            String, '/rescue_system/state', self.state_callback, 10
        )

        # 2) UI 레이아웃 및 디자인 설정
        self.init_ui()

        # 3) PyQt 타이머를 이용해 ROS 2 스핀(Spin)을 정기적으로 수행 (메인 스레드 락 방지)
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10) # 10ms 마다 ROS 이벤트 처리

    def init_ui(self):
        """ UI 화면 구성 """
        self.setWindowTitle("🚁 조난자 구조 시스템 - HITL 제어 패널")
        self.setFixedSize(400, 300)
        # RViz 창 위에 항상 뜨도록 설정 (Option)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

        # 메인 레이아웃
        layout = QVBoxLayout()

        # [상단] 시스템 상태 표시 라벨
        self.status_title = QLabel("현재 기체 구조 시스템 상태:")
        self.status_title.setFont(QFont("Arial", 11))
        self.status_title.setAlignment(Qt.AlignCenter)

        self.status_label = QLabel("상태 수신 대기 중...")
        self.status_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "color: #2c3e50; background-color: #ecf0f1; border-radius: 5px; padding: 10px;"
        )

        layout.addWidget(self.status_title)
        layout.addWidget(self.status_label)

        # 구분선
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # [중단] 안내 문구
        self.info_label = QLabel("파지(Grasp) 완료 시 아래 버튼을 누르세요")
        self.info_label.setFont(QFont("Arial", 10))
        self.info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.info_label)

        # [하단] Y / N 버튼 레이아웃
        btn_layout = QHBoxLayout()

        # Y 버튼 (승강)
        self.btn_y = QPushButton("YES (Y)\n[ 승강 진행 ]")
        self.btn_y.setFont(QFont("Arial", 13, QFont.Bold))
        self.btn_y.setFixedHeight(80)
        self.btn_y.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71; color: white; border-radius: 10px;
            }
            QPushButton:hover { background-color: #27ae60; }
            QPushButton:pressed { background-color: #1e8449; }
        """)
        self.btn_y.clicked.connect(lambda: self.send_command('Y'))

        # N 버튼 (해제)
        self.btn_n = QPushButton("NO (N)\n[ 파지 해제 ]")
        self.btn_n.setFont(QFont("Arial", 13, QFont.Bold))
        self.btn_n.setFixedHeight(80)
        self.btn_n.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c; color: white; border-radius: 10px;
            }
            QPushButton:hover { background-color: #c0392b; }
            QPushButton:pressed { background-color: #962d22; }
        """)
        self.btn_n.clicked.connect(lambda: self.send_command('N'))

        btn_layout.addWidget(self.btn_y)
        btn_layout.addWidget(self.btn_n)

        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def state_callback(self, msg: String):
        """ 젯슨의 /rescue_system/state 토픽 수신 시 화면 라벨 업데이트 """
        current_state = msg.data
        self.status_label.setText(current_state)

        # WAITING_USER 상태일 때 라벨 강조 효과
        if current_state == "WAITING_USER":
            self.status_label.setStyleSheet(
                "color: white; background-color: #f39c12; border-radius: 5px; padding: 10px;"
            )
        else:
            self.status_label.setStyleSheet(
                "color: #2c3e50; background-color: #ecf0f1; border-radius: 5px; padding: 10px;"
            )

    def send_command(self, cmd_str: str):
        """ 버튼 클릭 시 Y 또는 N 토픽 송신 """
        msg = String()
        msg.data = cmd_str
        self.user_input_pub.publish(msg)
        self.get_logger().info(f"지상 UI에서 사용자 명령 퍼블리시: '{cmd_str}'")

    def spin_ros(self):
        """ PyQt 타이머에 의해 10ms 마다 호출되는 ROS 2 스핀 함수 """
        rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    gui_node = GcsButtonGui()
    gui_node.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
