#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rescue_sequence_node.py

자율 비행 조난자 구조용 고정익 비행기 - 그리퍼(2서보) 파지/승강 시퀀스 제어 노드

[하드웨어 / 제어 구조]
- 컴패니언 컴퓨터: Jetson Orin Nano (Ubuntu 22.04, ROS 2 Humble, Python 3)
- 비행 컨트롤러: Pixhawk (PX4) - uXRCE-DDS 브리지로 ROS 2 와 직접 통신
- 그리퍼 구동: 서보 2개(양쪽 조), 서로 반대 방향으로 움직임

  ★ 오린은 저수준 서보 제어(PWM/PCA9685)를 하지 않습니다.
    오린은 "이 각도로 가라" 라는 액추에이터 값(-1~1)만 PX4 에 보내고,
    실제 서보 출력/PWM 은 PX4 의 control allocation(출력 함수 매핑)이 담당합니다.

  ★ 2개 서보가 반대로 움직이는 것은 PX4 출력 설정에서 처리합니다.
    한쪽 채널(서보 B)의 출력 min/max 를 뒤집어(reverse) 두었기 때문에,
    이 노드는 두 채널에 **같은 값**을 보내면 물리적으로 반대로 움직입니다.

[왜 DO_GRIPPER 가 아니라 DO_SET_ACTUATOR 인가]
- PX4 내장 Gripper 기능은 완전개방↔완전폐쇄 2상태(binary)뿐이라
  "조금 짚기(중간 각도)" 같은 2단 각도를 낼 수 없습니다.
- DO_SET_ACTUATOR 는 -1~1 연속값이라 각 단계의 각도를 자유롭게 지정할 수 있습니다.
- DO_SET_ACTUATOR 는 "Offboard Actuator Set N" 으로 지정한 **보조 출력핀만** 움직이며,
  모터/조종면 출력은 계속 비행 컨트롤러가 제어합니다. Offboard 모드 진입도, 믹서
  대체도 아니므로 비행 안전에 영향이 없습니다. (⚠️ direct_actuator 방식과 혼동 금지)

[통신 방식]
- 명령 송신: /fmu/in/vehicle_command  (px4_msgs/VehicleCommand)
             command = VEHICLE_CMD_DO_SET_ACTUATOR(187)
             param1  = 액추에이터 1 값 (-1~1)  -> QGC "Offboard Actuator Set 1" (서보 A)
             param2  = 액추에이터 2 값 (-1~1)  -> QGC "Offboard Actuator Set 2" (서보 B, PX4 에서 리버스)
             param7  = index (0 = 액추에이터 세트 1~6 그룹)
- 결과 수신: /fmu/out/vehicle_command_ack (px4_msgs/VehicleCommandAck)

[QGC 사전 설정 (Actuators 출력)]
  - 서보 A 를 연결한 출력핀 함수 = "Offboard Actuator Set 1"
  - 서보 B 를 연결한 출력핀 함수 = "Offboard Actuator Set 2"  (+ 출력 방향 리버스)
  - 두 핀 모두 모터/조종면과 겹치지 않는 여분 AUX 출력이어야 함.

[상태 머신]
IDLE -> (자동 시작 / 미션 트리거) -> GRASP_PARTIAL(조금 짚기) -> WAITING_USER
        -> (Y) -> ASCENDING(완전 파지/승강) -> HOLDING(유지)
                                              -> (Y/N) -> RELEASING(개방) -> IDLE
        -> (N) -> RELEASING(개방) -> IDLE

[단독 실행 모드]
- 노드를 실행하면 auto_start_delay_sec(기본 2초) 뒤에 스스로 1단계를 시작합니다.
- WAITING_USER 에서는 터미널에 y 또는 n 을 치고 Enter 를 누르면 됩니다.
- HOLDING(유지) 에서 다시 y 를 치면 그리퍼를 개방하고 IDLE 로 돌아갑니다.
  (지상 테스트에서 파지 -> 개방을 반복 확인하거나, 하차 지점에서 내려놓을 때 사용)
- 외부 토픽(mission_trigger / user_input) 구독은 주석 처리해 두었습니다.
  미션 플래너와 연동할 때 주석을 되살리고 아래 파라미터를 false 로 주세요:
      ros2 run imagery_processing rescue_sequence_node --ros-args \
          -p auto_start:=false -p keyboard_input:=false
"""

import sys
import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool

from px4_msgs.msg import VehicleCommand, VehicleCommandAck


# =============================================================================
# 상태(State) 정의
# =============================================================================
class RescueState(Enum):
    IDLE = auto()           # 대기 상태: 그리퍼 개방
    GRASP_PARTIAL = auto()  # 1단계: 조금 짚기 (중간 각도) 명령 후 완료 대기
    WAITING_USER = auto()   # 사용자(HITL) 승인 대기
    ASCENDING = auto()      # 2단계: 완전 파지/승강 (Y 승인 이후) 명령 후 완료 대기
    HOLDING = auto()        # 승강 완료 후 대상 유지 (여기서 Y/N 을 치면 개방)
    RELEASING = auto()      # 개방 명령 후 완료 대기 (거부(N) 또는 HOLDING 에서 개방)


class RescueSequenceNode(Node):
    """
    조난자 구조 시퀀스를 관리하는 ROS 2 노드.

    핵심 설계 포인트:
    - time.sleep() 을 사용하지 않습니다. 명령 ACK 대기/서보 이동 대기는 모두
      rclpy 의 타이머로 처리하여, 대기 중에도 다른 콜백(토픽 수신 등)이
      블로킹되지 않습니다.
    - 각 단계의 각도는 -1~1 액추에이터 값(파라미터)으로 지정합니다. 두 서보가
      반대로 움직이는 것은 PX4 출력 리버스로 처리하므로, 두 채널에 같은 값을
      보냅니다.
    """

    def __init__(self):
        super().__init__('rescue_sequence_node')

        # 콜백 그룹: 타이머와 구독자가 서로를 블로킹하지 않도록 재진입 허용
        self._cb_group = ReentrantCallbackGroup()

        # -----------------------------------------------------------
        # 1) ROS 2 파라미터 (launch 파일에서 오버라이드 가능)
        # -----------------------------------------------------------
        # DO_SET_ACTUATOR 의 param7(액추에이터 세트 그룹 index). 0 이면
        # param1~param6 이 "Offboard Actuator Set 1~6" 에 매핑됩니다.
        self.declare_parameter('actuator_set_index', 0)
        # 각 단계의 서보 목표값 (-1.0 ~ +1.0). 현장에서 QGC 없이 각도 튜닝 가능.
        self.declare_parameter('grip_partial', 0.4)   # 1단계: 조금 짚기
        self.declare_parameter('grip_full', 1.0)      # 2단계: 완전 파지/승강
        self.declare_parameter('grip_open', -1.0)     # 개방
        # PX4 로부터 명령 ACK 를 기다리는 최대 시간(초).
        self.declare_parameter('ack_timeout_sec', 2.0)
        # ACK 수신 후, 서보가 목표 각도까지 실제로 이동하는 데 걸리는 시간(초).
        self.declare_parameter('move_settle_sec', 2.0)
        # 노드를 실행하면 외부 트리거 없이 스스로 1단계를 시작할지 여부.
        self.declare_parameter('auto_start', True)
        # 자동 시작까지의 지연(초). PX4 연결과 DDS 디스커버리가 자리잡을 시간.
        self.declare_parameter('auto_start_delay_sec', 2.0)
        # 터미널에서 Y/N 을 직접 입력받을지 여부.
        self.declare_parameter('keyboard_input', True)

        self.actuator_set_index = self.get_parameter('actuator_set_index').value
        self.grip_partial = self.get_parameter('grip_partial').value
        self.grip_full = self.get_parameter('grip_full').value
        self.grip_open = self.get_parameter('grip_open').value
        self.ack_timeout_sec = self.get_parameter('ack_timeout_sec').value
        self.move_settle_sec = self.get_parameter('move_settle_sec').value
        self.auto_start = self.get_parameter('auto_start').value
        self.auto_start_delay_sec = self.get_parameter('auto_start_delay_sec').value
        self.keyboard_input = self.get_parameter('keyboard_input').value

        # -----------------------------------------------------------
        # 2) PX4 uXRCE-DDS 통신용 QoS
        #    PX4 가 발행/구독하는 /fmu/* 토픽은 BEST_EFFORT + TRANSIENT_LOCAL 을
        #    사용합니다. 기본 QoS 로 두면 메시지가 오가지 않으니 반드시 맞춰야 합니다.
        # -----------------------------------------------------------
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -----------------------------------------------------------
        # 3) 상태 변수 초기화
        # -----------------------------------------------------------
        self.state = RescueState.IDLE

        # 현재 ACK 를 기다리고 있는 명령 (None 이면 대기 중 아님)
        self._pending_command = None     # 방금 보낸 VehicleCommand.command 값
        self._ack_timer = None           # ACK 타임아웃 감시 타이머
        self._settle_timer = None        # 서보 이동 완료 대기 타이머
        self._on_move_done = None        # 이동 완료 시 실행할 콜백

        # -----------------------------------------------------------
        # 4) PX4 인터페이스
        # -----------------------------------------------------------
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos
        )
        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.command_ack_callback,
            px4_qos,
            callback_group=self._cb_group,
        )

        # -----------------------------------------------------------
        # 5) 미션/GCS 인터페이스
        # -----------------------------------------------------------

        # ┌─────────────────────────────────────────────────────────────┐
        # │ [단독 실행 모드로 전환하면서 주석 처리]                      │
        # │  아래 두 구독자는 외부 노드/터미널에서 토픽을 쏴줘야만       │
        # │  시퀀스가 진행되는 방식입니다. 지금은 노드를 실행하면        │
        # │  자동으로 시작하고 키보드로 Y/N 을 받으므로 꺼둡니다.        │
        # │  미션 플래너와 연동할 때 아래 주석을 되살리고,               │
        # │  파라미터 auto_start / keyboard_input 을 false 로 주면 됩니다.│
        # └─────────────────────────────────────────────────────────────┘

        # # (a) 외부 미션 트리거: 조난자 위치 도달 신호 (True 수신 시 GRASP_PARTIAL 시작)
        # self.mission_trigger_sub = self.create_subscription(
        #     Bool,
        #     '/rescue_system/mission_trigger',
        #     self.mission_trigger_callback,
        #     10,
        #     callback_group=self._cb_group,
        # )
        #
        # # (b) 지상 통제소(GCS) 사용자 입력 토픽: 'Y' 또는 'N'
        # self.user_input_sub = self.create_subscription(
        #     String,
        #     '/rescue_system/user_input',
        #     self.user_input_callback,
        #     10,
        #     callback_group=self._cb_group,
        # )

        # (c) 현재 상태를 외부(GCS/RViz)에 알리기 위한 상태 퍼블리셔
        self.state_pub = self.create_publisher(String, '/rescue_system/state', 10)
        self._publish_state()

        self.get_logger().info(
            f"구조 시퀀스 제어 노드 준비 완료 "
            f"(세트 index={self.actuator_set_index}, "
            f"조금짚기={self.grip_partial}, 완전={self.grip_full}, 개방={self.grip_open})."
        )

        # -----------------------------------------------------------
        # 6) 단독 실행 모드: 자동 시작 + 키보드 입력
        # -----------------------------------------------------------

        # 키보드 입력 스레드를 먼저 띄워야, 자동 시작이 빨라도 Y 를 놓치지 않습니다.
        self._stdin_thread = None
        if self.keyboard_input:
            self._start_keyboard_thread()

        if self.auto_start:
            self.get_logger().info(
                f"{self.auto_start_delay_sec:.1f}초 후 1단계 '조금 짚기' 를 자동으로 시작합니다."
            )
            # 일회성 타이머: PX4 연결/DDS 디스커버리가 자리잡을 시간을 준 뒤 시작
            self._auto_start_timer = self.create_timer(
                self.auto_start_delay_sec,
                self._on_auto_start,
                callback_group=self._cb_group,
            )
        else:
            self._auto_start_timer = None
            self.get_logger().info("자동 시작 꺼짐. 미션 트리거 대기 중...")

    # =========================================================================
    # PX4 명령 송신
    # =========================================================================
    def _publish_vehicle_command(self, command: int, param1: float = 0.0,
                                 param2: float = 0.0, param7: float = 0.0):
        """
        PX4 로 VehicleCommand 를 발행합니다.
        multirotor 패키지의 C++ 노드와 동일한 필드 규약을 사용합니다.
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def _set_actuators(self, value: float, on_done, label: str):
        """
        PX4 에 그리퍼 서보 목표값을 보내고 ACK 를 기다립니다.

        두 서보 채널(Offboard Actuator Set 1/2)에 같은 값을 보냅니다.
        서보 B 의 반대 방향은 PX4 출력 리버스가 처리합니다.

        :param value:  액추에이터 목표값 (-1.0 ~ +1.0)
        :param on_done: 서보 이동이 완료된 것으로 판단됐을 때 호출할 콜백(인자 없음)
        :param label:  로그용 동작 이름 (예: '조금 짚기')
        """
        self._cancel_gripper_timers()

        self._pending_command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        self._on_move_done = on_done

        self.get_logger().info(
            f"PX4 로 그리퍼 서보 명령 전송: {label} (값={value:+.2f}, DO_SET_ACTUATOR)."
        )

        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR,
            param1=value,                       # 서보 A (Offboard Actuator Set 1)
            param2=value,                       # 서보 B (Offboard Actuator Set 2, PX4 리버스)
            param7=float(self.actuator_set_index),
        )

        # ACK 가 정해진 시간 안에 오지 않으면 실패로 처리
        self._ack_timer = self.create_timer(
            self.ack_timeout_sec,
            self._on_ack_timeout,
            callback_group=self._cb_group,
        )

    def command_ack_callback(self, msg: VehicleCommandAck):
        """
        PX4 가 보내는 명령 ACK 를 처리합니다.
        우리가 방금 보낸 명령(DO_SET_ACTUATOR)에 대한 ACK 만 골라서 봅니다.
        """
        if self._pending_command is None:
            return  # 대기 중인 명령 없음
        if msg.command != self._pending_command:
            return  # 다른 명령(ARM 등)의 ACK 는 무시

        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS:
            # 아직 진행 중 - 최종 ACK 를 계속 기다립니다.
            self.get_logger().info("PX4: 액추에이터 명령 진행 중(IN_PROGRESS)...")
            return

        # 최종 결과가 왔으므로 ACK 타임아웃 감시는 중단
        self._cancel_timer('_ack_timer')

        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info(
                f"PX4: 액추에이터 명령 수락(ACCEPTED). 서보 이동 완료까지 "
                f"{self.move_settle_sec:.1f}초 대기합니다."
            )
            # PX4 는 명령 수락만 알려줄 뿐 "서보가 실제로 다 움직였다"는 피드백은
            # 기본 uXRCE-DDS 토픽으로 오지 않습니다. 그래서 서보 이동 시간만큼
            # 기다린 뒤 완료로 간주합니다.
            self._settle_timer = self.create_timer(
                self.move_settle_sec,
                self._on_settle_complete,
                callback_group=self._cb_group,
            )
        else:
            # DENIED/UNSUPPORTED 는 대부분 PX4 출력 설정 문제입니다.
            self.get_logger().error(
                f"PX4 가 액추에이터 명령을 거부했습니다 (result={msg.result}). "
                "QGC Actuators 에서 해당 출력핀이 'Offboard Actuator Set 1/2' 로 "
                "지정돼 있는지 확인하세요."
            )
            self._abort_to_idle()

    def _on_ack_timeout(self):
        """정해진 시간 안에 PX4 ACK 가 오지 않은 경우."""
        self._cancel_timer('_ack_timer')
        self.get_logger().error(
            f"PX4 로부터 명령 ACK 를 {self.ack_timeout_sec:.1f}초 안에 받지 못했습니다. "
            "uXRCE-DDS Agent 실행 여부와 /fmu/out/vehicle_command_ack 수신을 확인하세요."
        )
        self._abort_to_idle()

    def _on_settle_complete(self):
        """서보 이동 대기 시간이 끝나 동작이 완료된 것으로 간주."""
        self._cancel_timer('_settle_timer')

        done_cb = self._on_move_done
        self._pending_command = None
        self._on_move_done = None

        if done_cb is not None:
            done_cb()

    # =========================================================================
    # 타이머 정리 유틸
    # =========================================================================
    def _cancel_timer(self, attr_name: str):
        """지정한 타이머 속성이 살아 있으면 취소/파괴합니다."""
        timer = getattr(self, attr_name, None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            setattr(self, attr_name, None)

    def _cancel_gripper_timers(self):
        self._cancel_timer('_ack_timer')
        self._cancel_timer('_settle_timer')

    def _abort_to_idle(self):
        """명령 실패 시 대기 상태를 정리하고 IDLE 로 복귀합니다."""
        self._cancel_gripper_timers()
        self._pending_command = None
        self._on_move_done = None
        self.state = RescueState.IDLE
        self._publish_state()
        self.get_logger().warn("시퀀스를 중단하고 IDLE 상태로 복귀했습니다.")

    def _publish_state(self):
        """현재 상태 머신 상태를 토픽으로 퍼블리시 (GCS/RViz 모니터링용)."""
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    # =========================================================================
    # 미션 트리거 콜백: IDLE -> GRASP_PARTIAL (1단계: 조금 짚기)
    # =========================================================================
    def mission_trigger_callback(self, msg: Bool):
        """
        외부 미션 플래너로부터 '조난자 위치 도달' 신호를 수신했을 때 호출됩니다.
        (현재 구독자는 단독 실행 모드라 주석 처리되어 있습니다. 미션 연동 시 복구)
        """
        if not msg.data:
            return  # False 는 무시

        self._start_sequence('미션 트리거 수신')

    def _on_auto_start(self):
        """자동 시작 타이머(일회성)가 만료됐을 때 호출됩니다."""
        self._cancel_timer('_auto_start_timer')
        self._start_sequence('자동 시작')

    def _start_sequence(self, source_label: str):
        """
        IDLE -> GRASP_PARTIAL 전이를 수행합니다. 미션 트리거와 자동 시작이
        공유하는 단일 진입점입니다. IDLE 일 때만 시작하여, 시퀀스 진행 중
        중복 트리거가 들어와도 무시합니다.
        """
        if self.state != RescueState.IDLE:
            self.get_logger().warn(
                f"{source_label}: 현재 상태가 {self.state.name} 이므로 무시합니다."
            )
            return

        self.get_logger().info(f"{source_label}: 1단계 '조금 짚기' 시퀀스를 시작합니다.")
        self.state = RescueState.GRASP_PARTIAL
        self._publish_state()

        self._set_actuators(self.grip_partial, self._on_partial_grasp_complete, '조금 짚기')

    def _on_partial_grasp_complete(self):
        """
        1단계(조금 짚기)가 완료된 것으로 판단됐을 때 호출되는 콜백.
        사용자 승인을 기다리는 WAITING_USER 상태로 전이합니다.
        """
        self.state = RescueState.WAITING_USER
        self._publish_state()

        if self.keyboard_input:
            prompt = "터미널에 y(승인) 또는 n(개방) 입력 후 Enter"
        else:
            prompt = "/rescue_system/user_input 토픽으로 'Y' 또는 'N' 입력 대기"

        self.get_logger().info(f"1단계 '조금 짚기' 완료. 사용자 승인 대기 중... ({prompt})")

    # =========================================================================
    # 사용자 입력(HITL) 콜백: WAITING_USER -> ASCENDING / RELEASING
    # =========================================================================
    def user_input_callback(self, msg: String):
        """
        GCS(RViz 패널 또는 게임패드 브리지 노드 등)에서 발행하는
        /rescue_system/user_input 토픽을 수신하는 콜백입니다.

        [WAITING_USER 상태]
        - 'Y' 수신 시: 파지 성공으로 판단 -> ASCENDING(2단계: 완전 파지/승강)
        - 'N' 수신 시: 파지 실패/취소로 판단 -> RELEASING(개방) 후 IDLE 복귀

        [HOLDING 상태]
        - 'Y'/'N' 수신 시: 대상을 내려놓음 -> RELEASING(개방) 후 IDLE 복귀

        그 외 상태(명령 전송/서보 이동 중)에서 수신된 입력은 안전을 위해 무시합니다.
        """
        self._handle_user_input(msg.data, '토픽')

    def _handle_user_input(self, raw_input: str, source_label: str):
        """
        Y/N 판정과 상태 전이를 수행합니다. 토픽 콜백과 키보드 입력 스레드가
        공유하는 단일 진입점입니다.
        """
        user_cmd = raw_input.strip().upper()

        # HOLDING(파지 유지) 에서는 어떤 키든 '개방' 으로 처리합니다.
        # 완전 파지 후 그리퍼를 다시 열 수 있는 유일한 경로입니다.
        if self.state == RescueState.HOLDING:
            if user_cmd in ('Y', 'N'):
                self.get_logger().info(
                    f"HOLDING 상태에서 사용자 입력({source_label}) '{user_cmd}' 수신: "
                    "그리퍼를 개방합니다."
                )
                self._start_release()
            else:
                self.get_logger().warn(
                    f"알 수 없는 사용자 입력 '{raw_input}' 입니다. "
                    "HOLDING 상태에서는 'Y' 또는 'N' 을 치면 그리퍼가 개방됩니다."
                )
            return

        if self.state != RescueState.WAITING_USER:
            self.get_logger().warn(
                f"사용자 입력({source_label}) '{raw_input}' 수신했지만 현재 상태가 "
                f"{self.state.name} 이므로 무시합니다 (WAITING_USER/HOLDING 에서만 유효)."
            )
            return

        if user_cmd == 'Y':
            self.get_logger().info("사용자 승인('Y') 수신: 2단계 '완전 파지/승강' 으로 전환합니다.")
            self.state = RescueState.ASCENDING
            self._publish_state()
            self._set_actuators(self.grip_full, self._on_ascend_complete, '완전 파지/승강')

        elif user_cmd == 'N':
            self.get_logger().info("사용자 거부('N') 수신: 그리퍼를 개방하고 RELEASING 시퀀스를 시작합니다.")
            self._start_release()

        else:
            self.get_logger().warn(
                f"알 수 없는 사용자 입력 '{raw_input}' 을(를) 수신했습니다. "
                f"'Y' 또는 'N' 만 유효합니다."
            )

    def _start_release(self):
        """
        RELEASING(개방) 시퀀스를 시작합니다.
        WAITING_USER 의 'N' 과 HOLDING 의 'Y'/'N' 이 공유하는 단일 진입점입니다.
        """
        self.state = RescueState.RELEASING
        self._publish_state()
        self._set_actuators(self.grip_open, self._on_release_complete, '개방')

    # =========================================================================
    # 키보드 입력 (단독 실행 모드)
    # =========================================================================
    def _start_keyboard_thread(self):
        """
        표준입력에서 y/n 을 읽는 데몬 스레드를 띄웁니다.

        executor 가 MultiThreadedExecutor 라 이 스레드에서 노드 메서드를 호출해도
        콜백들과 동일한 수준으로 병렬 실행됩니다. 데몬이라 노드 종료 시 같이 죽습니다.
        """
        self._stdin_thread = threading.Thread(
            target=self._keyboard_loop, name='rescue_keyboard', daemon=True
        )
        self._stdin_thread.start()

    def _keyboard_loop(self):
        """표준입력을 한 줄씩 읽어 _handle_user_input 으로 넘깁니다."""
        # 백그라운드 실행 등으로 stdin 이 없으면 조용히 빠져나갑니다.
        if not sys.stdin or not sys.stdin.isatty():
            self.get_logger().warn(
                "표준입력이 터미널이 아니라 키보드 입력을 사용할 수 없습니다. "
                "터미널에서 직접 실행하거나, /rescue_system/user_input 토픽을 쓰세요."
            )
            return

        while rclpy.ok():
            try:
                line = sys.stdin.readline()
            except (OSError, ValueError):
                return

            if line == '':      # EOF (Ctrl-D 또는 stdin 종료)
                return

            line = line.strip()
            if not line:        # 그냥 Enter 는 무시
                continue

            self._handle_user_input(line, '키보드')

    def _on_ascend_complete(self):
        """
        2단계(완전 파지/승강)가 완료된 것으로 판단됐을 때 호출되는 콜백.
        대상을 유지하는 HOLDING 상태로 전이합니다.
        """
        self.state = RescueState.HOLDING
        self._publish_state()

        if self.keyboard_input:
            prompt = "터미널에 y(또는 n) 입력 후 Enter"
        else:
            prompt = "/rescue_system/user_input 토픽으로 'Y' 또는 'N' 발행"

        self.get_logger().info(
            f"2단계 '완전 파지/승강' 완료. 대상을 유지(HOLDING)합니다. "
            f"내려놓으려면 개방 입력을 주세요. ({prompt})"
        )

    def _on_release_complete(self):
        """
        그리퍼 개방 완료 시 호출되는 콜백.
        상태를 IDLE 로 되돌려 다음 미션 트리거를 받을 수 있도록 준비합니다.
        """
        self.state = RescueState.IDLE
        self._publish_state()
        self.get_logger().info("그리퍼 개방 완료. IDLE 상태로 복귀하여 다음 미션 트리거를 대기합니다.")


def main(args=None):
    rclpy.init(args=args)

    node = RescueSequenceNode()

    # MultiThreadedExecutor 를 사용하면 여러 콜백 그룹이 병렬 처리되어
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
