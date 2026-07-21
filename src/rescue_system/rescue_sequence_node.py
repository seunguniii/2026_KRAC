#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rescue_sequence_node.py

자율 비행 조난자 구조용 고정익 비행기 - 그리퍼/승강 시퀀스 제어 노드

[하드웨어]
- 컴패니언 컴퓨터: Jetson Orin Nano (Ubuntu 22.04, ROS 2 Humble, Python 3)
- 그리퍼 구동: MG996R 서보모터 1개 (PCA9685 채널 0번, I2C 연결)
- 비행 컨트롤러: Pixhawk (연직 레일 승강 시스템 연동은 스텁 처리, 추후 실장)

[상태 머신]
IDLE -> (미션 트리거) -> GRASPING -> WAITING_USER -> (Y) -> ASCENDING
                                                  -> (N) -> RELEASING -> IDLE
"""

import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool

# ----------------------------------------------------------------------------
# PCA9685 / 서보킷 제어 라이브러리
#   - Jetson 실기 환경에서는 adafruit-circuitpython-servokit 이 필요합니다.
#   - 코드 리뷰/시뮬레이션 환경(라이브러리 미설치) 에서도 노드 로직을 확인할 수
#     있도록, 라이브러리 임포트 실패 시 콘솔 출력만 하는 더미(Mock) 클래스로
#     대체하는 안전장치를 넣었습니다. 실기에서는 반드시 정상 임포트되어야 합니다.
# ----------------------------------------------------------------------------
try:
    from adafruit_servokit import ServoKit
    SERVOKIT_AVAILABLE = True
except ImportError:
    SERVOKIT_AVAILABLE = False


class MockServoKit:
    """
    adafruit_servokit 라이브러리가 없는 환경(예: 개발 PC)에서
    노드 로직 테스트를 위해 사용하는 더미 클래스입니다.
    실제 젯슨 보드에서는 사용되지 않고, 위의 SERVOKIT_AVAILABLE 분기로
    실제 ServoKit 이 사용됩니다.
    """
    class _MockChannel:
        def __init__(self, ch_id):
            self.ch_id = ch_id
            self._angle = 0.0

        @property
        def angle(self):
            return self._angle

        @angle.setter
        def angle(self, value):
            self._angle = value
            # 실제 하드웨어가 없을 때 콘솔로만 확인
            # print(f"[MOCK PCA9685] channel {self.ch_id} angle -> {value:.1f}")

    def __init__(self, channels=16):
        self.servo = [self._MockChannel(i) for i in range(channels)]


# =============================================================================
# 상태(State) 정의
# =============================================================================
class RescueState(Enum):
    IDLE = auto()          # 대기 상태: 그리퍼 완전 개방(0도)
    GRASPING = auto()      # 파지 진행: 0 -> 60도 저속 구동
    WAITING_USER = auto()  # 사용자(HITL) 승인 대기
    ASCENDING = auto()     # 승인(Y) 이후 연직 레일 승강 (Pixhawk 연동 스텁)
    RELEASING = auto()     # 거부(N) 이후 그리퍼 해제: 60 -> 0도 저속 구동


class RescueSequenceNode(Node):
    """
    조난자 구조 시퀀스를 관리하는 ROS 2 노드.

    핵심 설계 포인트:
    - time.sleep() 을 전혀 사용하지 않고, rclpy 의 create_timer() 를 이용해
      짧은 주기(예: 20ms)마다 콜백을 실행시켜 서보 각도를 조금씩 증가/감소시킵니다.
      이렇게 하면 서보가 움직이는 동안에도 ROS 2 콜백(토픽 수신, 다른 타이머 등)이
      블로킹되지 않고 계속 처리됩니다 (비동기/논블로킹 동작).
    - 상태 머신(RescueState)을 통해 현재 시퀀스 단계를 명확히 관리합니다.
    """

    # ------------------------------------------------------------------
    # 사용자 설정 파라미터 (필요 시 ROS 2 파라미터로 승격 가능)
    # ------------------------------------------------------------------
    PCA9685_CHANNEL = 0            # PCA9685 상의 서보 연결 채널 번호
    ANGLE_OPEN = 0.0                # 그리퍼 완전 개방 각도 (이론값, 추후 실측 후 보정 예정)
    ANGLE_GRASP = 60.0               # 그리퍼 파지 완료 각도 (이론값, 추후 실측 후 보정 예정)
    STEP_DEGREE = 1.0               # 타이머 1회당 증가/감소시키는 각도(도) - 값이 작을수록 더 부드러움
    STEP_PERIOD_SEC = 0.02           # 타이머 주기 (초) - 20ms 마다 STEP_DEGREE 만큼 이동 => 저속 구동

    # Pixhawk 관련 - 포트/파라미터가 아직 확정되지 않아 변수로만 선언해 둡니다.
    # 추후 실제 연동 시 이 값들을 실측 값으로 교체하거나 launch 파라미터로 분리하세요.
    PIXHAWK_SERIAL_PORT = None       # 예: "/dev/ttyTHS1" 등, 추후 확정 예정 (미정 -> None)
    PIXHAWK_BAUDRATE = None          # 예: 921600 등, 추후 확정 예정 (미정 -> None)

    def __init__(self):
        super().__init__('rescue_sequence_node')

        # 콜백 그룹: 여러 콜백(타이머 + 구독자)이 동시에 존재해도
        # 서로를 블로킹하지 않도록 재진입 가능한 콜백 그룹을 사용합니다.
        self._cb_group = ReentrantCallbackGroup()

        # -----------------------------------------------------------
        # 1) PCA9685 / 서보 초기화
        # -----------------------------------------------------------
        if SERVOKIT_AVAILABLE:
            # 실제 하드웨어 초기화 (I2C 버스를 통해 PCA9685 와 통신)
            self.kit = ServoKit(channels=16)
            self.get_logger().info("ServoKit(PCA9685) 하드웨어 초기화 완료.")
        else:
            # 라이브러리 미설치 환경 - 더미로 대체 (실기에서는 발생하면 안 됨)
            self.kit = MockServoKit(channels=16)
            self.get_logger().warn(
                "adafruit_servokit 라이브러리를 찾을 수 없어 Mock 모드로 동작합니다. "
                "실제 젯슨 보드에서는 반드시 라이브러리를 설치하세요."
            )

        # 노드 시작 시 그리퍼를 확실히 '개방' 상태로 초기화
        self.current_angle = self.ANGLE_OPEN
        self._set_servo_angle(self.current_angle)

        # -----------------------------------------------------------
        # 2) 상태 변수 초기화
        # -----------------------------------------------------------
        self.state = RescueState.IDLE
        self.get_logger().info(f"초기 상태: {self.state.name} (그리퍼 {self.ANGLE_OPEN}도 개방)")

        # 저속 구동 타이머 핸들 (필요 시에만 생성/해제하여 리소스 낭비 방지)
        self._motion_timer = None
        self._motion_target_angle = None   # 현재 저속 구동의 목표 각도
        self._motion_direction = 0          # +1: 증가(GRASPING), -1: 감소(RELEASING)

        # -----------------------------------------------------------
        # 3) 구독자(Subscriber) / 서비스 등 인터페이스 설정
        # -----------------------------------------------------------

        # (a) 외부 미션 트리거: 비행기가 조난자 위치에 도달했음을 알리는 토픽
        #     예시로 std_msgs/Bool 타입을 사용 (True 수신 시 GRASPING 시작)
        #     실제 미션 플래너 노드에서 이 토픽으로 True 를 1회 publish 하도록 구성하세요.
        self.mission_trigger_sub = self.create_subscription(
            Bool,
            '/rescue_system/mission_trigger',
            self.mission_trigger_callback,
            10,
            callback_group=self._cb_group,
        )

        # (b) 지상 통제소(GCS) 사용자 입력 토픽: 'Y' 또는 'N'
        self.user_input_sub = self.create_subscription(
            String,
            '/rescue_system/user_input',
            self.user_input_callback,
            10,
            callback_group=self._cb_group,
        )

        # (c) 현재 상태를 외부(GCS/RViz)에 알리기 위한 상태 퍼블리셔
        #     RViz 패널이나 GCS 소프트웨어에서 이 토픽을 구독해 현재 상태를 표시할 수 있습니다.
        self.state_pub = self.create_publisher(String, '/rescue_system/state', 10)
        self._publish_state()

        self.get_logger().info("구조 시퀀스 제어 노드가 준비되었습니다. 미션 트리거 대기 중...")

    # =========================================================================
    # 서보 각도 저수준 제어 함수
    # =========================================================================
    def _set_servo_angle(self, angle: float):
        """
        PCA9685의 지정 채널에 실제 각도 값을 씁니다.
        각도는 0~180도 범위로 클램핑하여 서보/기구 손상을 방지합니다.
        """
        angle = max(0.0, min(180.0, angle))
        self.kit.servo[self.PCA9685_CHANNEL].angle = angle
        self.current_angle = angle

    def _publish_state(self):
        """현재 상태 머신 상태를 토픽으로 퍼블리시 (GCS/RViz 모니터링용)."""
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    # =========================================================================
    # 타이머 기반 "저속 구동" 제어 루프
    #   - time.sleep() 대신 create_timer() 를 사용하여 ROS 2 스핀(spin)을
    #     블로킹하지 않고 각도를 조금씩(STEP_DEGREE) 목표까지 이동시킵니다.
    # =========================================================================
    def _start_slow_motion(self, target_angle: float, on_complete):
        """
        현재 각도에서 target_angle 까지 STEP_DEGREE 씩 STEP_PERIOD_SEC 주기로
        서서히 이동시키는 타이머를 시작합니다.

        :param target_angle: 도달해야 할 목표 각도
        :param on_complete: 목표 각도 도달 시 호출할 콜백 함수(인자 없음)
        """
        # 이미 진행 중인 모션 타이머가 있다면 안전하게 정리
        self._stop_slow_motion()

        self._motion_target_angle = target_angle
        self._motion_direction = 1 if target_angle > self.current_angle else -1
        self._motion_on_complete = on_complete

        # ROS 2 Timer 생성: STEP_PERIOD_SEC 마다 _motion_timer_callback 실행
        self._motion_timer = self.create_timer(
            self.STEP_PERIOD_SEC,
            self._motion_timer_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"저속 구동 시작: {self.current_angle:.1f}도 -> {target_angle:.1f}도 "
            f"(스텝 {self.STEP_DEGREE}도 / {self.STEP_PERIOD_SEC*1000:.0f}ms)"
        )

    def _motion_timer_callback(self):
        """
        타이머가 주기적으로 호출하는 콜백.
        한 번 호출될 때마다 STEP_DEGREE 만큼만 각도를 이동시켜
        서보가 부드럽게(기어가듯) 움직이도록 합니다.
        목표 각도에 도달하면 타이머를 해제하고 on_complete 콜백을 실행합니다.
        """
        next_angle = self.current_angle + (self._motion_direction * self.STEP_DEGREE)

        # 목표 각도를 초과/미달하지 않도록 오버슈트 방지 처리
        reached = False
        if self._motion_direction > 0 and next_angle >= self._motion_target_angle:
            next_angle = self._motion_target_angle
            reached = True
        elif self._motion_direction < 0 and next_angle <= self._motion_target_angle:
            next_angle = self._motion_target_angle
            reached = True

        self._set_servo_angle(next_angle)

        if reached:
            self.get_logger().info(f"목표 각도 {self._motion_target_angle:.1f}도 도달.")
            self._stop_slow_motion()
            # 목표 도달 후 실행할 후속 처리(상태 전이 등) 콜백 실행
            if self._motion_on_complete is not None:
                self._motion_on_complete()

    def _stop_slow_motion(self):
        """진행 중인 저속 구동 타이머가 있다면 안전하게 파괴(해제)합니다."""
        if self._motion_timer is not None:
            self._motion_timer.cancel()
            self.destroy_timer(self._motion_timer)
            self._motion_timer = None

    # =========================================================================
    # 미션 트리거 콜백: IDLE -> GRASPING 진입
    # =========================================================================
    def mission_trigger_callback(self, msg: Bool):
        """
        외부 미션 플래너로부터 '조난자 위치 도달' 신호를 수신했을 때 호출됩니다.
        현재 상태가 IDLE 일 때만 GRASPING 시퀀스를 시작합니다.
        (이미 시퀀스가 진행 중일 때 중복 트리거가 들어와도 무시하여 안전성 확보)
        """
        if not msg.data:
            return  # False 는 무시

        if self.state != RescueState.IDLE:
            self.get_logger().warn(
                f"미션 트리거 수신했지만 현재 상태가 {self.state.name} 이므로 무시합니다."
            )
            return

        self.get_logger().info("미션 트리거 수신: GRASPING(파지) 시퀀스를 시작합니다.")
        self.state = RescueState.GRASPING
        self._publish_state()

        # 0도 -> ANGLE_GRASP(60도) 까지 저속 구동 시작.
        # 목표 도달 시 _on_grasp_complete 콜백이 자동 호출되어 WAITING_USER로 전이됩니다.
        self._start_slow_motion(self.ANGLE_GRASP, self._on_grasp_complete)

    def _on_grasp_complete(self):
        """
        그리퍼가 60도(파지 완료 각도)에 도달했을 때 호출되는 콜백.
        모터 구동을 멈춘 상태(타이머는 이미 _stop_slow_motion 에서 해제됨)에서
        사용자 승인을 기다리는 WAITING_USER 상태로 전이합니다.
        """
        self.state = RescueState.WAITING_USER
        self._publish_state()
        self.get_logger().info(
            "그리퍼 파지 완료. 사용자 승인 대기 중... "
            "(/rescue_system/user_input 토픽으로 'Y' 또는 'N' 입력 대기)"
        )

    # =========================================================================
    # 사용자 입력(HITL) 콜백: WAITING_USER -> ASCENDING / RELEASING
    # =========================================================================
    def user_input_callback(self, msg: String):
        """
        GCS(RViz 패널 또는 게임패드 브리지 노드 등)에서 발행하는
        /rescue_system/user_input 토픽을 수신하는 콜백입니다.

        - 'Y' 수신 시: 파지가 정상적으로 이루어졌다고 판단 -> ASCENDING(승강) 시작
        - 'N' 수신 시: 파지 실패/취소로 판단 -> RELEASING(해제) 후 IDLE 복귀

        WAITING_USER 상태가 아닐 때 수신된 입력은 안전을 위해 무시합니다.
        """
        if self.state != RescueState.WAITING_USER:
            self.get_logger().warn(
                f"사용자 입력 '{msg.data}' 수신했지만 현재 상태가 "
                f"{self.state.name} 이므로 무시합니다 (WAITING_USER 상태에서만 유효)."
            )
            return

        user_cmd = msg.data.strip().upper()

        if user_cmd == 'Y':
            self.get_logger().info("사용자 승인('Y') 수신: ASCENDING(연직 레일 승강) 시퀀스로 전환합니다.")
            self.state = RescueState.ASCENDING
            self._publish_state()
            self._start_ascending()

        elif user_cmd == 'N':
            self.get_logger().info("사용자 거부('N') 수신: 그리퍼를 해제하고 RELEASING 시퀀스를 시작합니다.")
            self.state = RescueState.RELEASING
            self._publish_state()
            # 60도 -> 0도(ANGLE_OPEN) 로 저속 구동. 완료 시 _on_release_complete 호출.
            self._start_slow_motion(self.ANGLE_OPEN, self._on_release_complete)

        else:
            self.get_logger().warn(
                f"알 수 없는 사용자 입력 '{msg.data}' 을(를) 수신했습니다. "
                f"'Y' 또는 'N' 만 유효합니다."
            )

    def _on_release_complete(self):
        """
        그리퍼 해제(0도) 완료 시 호출되는 콜백.
        상태를 IDLE 로 되돌려 다음 미션 트리거를 받을 수 있도록 준비합니다.
        """
        self.state = RescueState.IDLE
        self._publish_state()
        self.get_logger().info("그리퍼 해제 완료. IDLE 상태로 복귀하여 다음 미션 트리거를 대기합니다.")

    # =========================================================================
    # 픽스호크 연동 스텁 (연직 레일 승강 시스템)
    #   - 포트 번호 등 통신 파라미터가 아직 확정되지 않아 PIXHAWK_SERIAL_PORT /
    #     PIXHAWK_BAUDRATE 를 클래스 상단에 변수(현재 None)로만 선언해 두었습니다.
    #   - 추후 MAVLink(pymavlink) 또는 MAVROS 연동 코드를 이 함수 내부에
    #     구현하면 됩니다. 지금은 상태 전이와 로그만 수행하는 스텁입니다.
    # =========================================================================
    def _start_ascending(self):
        """
        연직 레일 승강 시스템 구동 스텁 함수.
        실제 구현 시 아래와 같은 작업이 필요할 것으로 예상됩니다:
          1) PIXHAWK_SERIAL_PORT / PIXHAWK_BAUDRATE 확정 후 시리얼 포트 연결
          2) MAVLink 명령(예: 특정 서보 출력 채널 PWM 제어, 혹은 커스텀 릴레이 제어)으로
             레일 승강 모터 구동
          3) 레일이 완전히 상승 완료되었음을 알리는 센서(리밋 스위치 등) 피드백 처리
        지금 단계에서는 하드웨어가 아직 정해지지 않았으므로 로그만 출력합니다.
        """
        self.get_logger().info(
            "[STUB] Pixhawk 연동 미구현 상태입니다. "
            f"PIXHAWK_SERIAL_PORT={self.PIXHAWK_SERIAL_PORT}, "
            f"PIXHAWK_BAUDRATE={self.PIXHAWK_BAUDRATE} "
            "(추후 포트/보드레이트 확정 후 실제 MAVLink 연동 코드로 대체 예정)"
        )
        # TODO: 실제 Pixhawk/MAVLink 연동 코드 구현 위치
        # 예) self._pixhawk_conn = mavutil.mavlink_connection(self.PIXHAWK_SERIAL_PORT,
        #                                                     baud=self.PIXHAWK_BAUDRATE)
        #     self._pixhawk_conn.mav.command_long_send(...)


def main(args=None):
    rclpy.init(args=args)

    node = RescueSequenceNode()

    # MultiThreadedExecutor 를 사용하면 여러 콜백 그룹이 진짜로 병렬 처리되어
    # 타이머 콜백과 토픽 콜백이 서로를 기다리지 않고 동작할 수 있습니다.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
