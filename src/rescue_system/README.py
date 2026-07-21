# 🚁 Rescue System Package (`rescue_system`)

이 패키지는 자율 비행 조난자 구조 고정익 비행기의 미션 장비(그리퍼 파지 및 승강 레일)를 제어하는 ROS 2 패키지입니다. 

인공지능/자율 비행에 의해 조난자 위치에 도달하면 그리퍼를 저속으로 파지(Grasp)한 뒤, 지상 통제소(GCS)의 Human-in-the-Loop(HITL) Y/N 승인 신호를 받아 다음 승강 시퀀스로 안전하게 전이되도록 설계되었습니다.

---

## 📂 노드 구성 (Node Overview)

### 1. `rescue_sequence_node.py` (젯슨 오린 나노 탑재용 메인 제어 노드)
* 역할: 미션 트리거 수신, 서보 모터(MG996R) 논블로킹 저속 구동, 상태 머신 관리, 지상 Y/N 신호 수신 및 후속 시퀀스(승강) 진행
* 특징:
  * `time.sleep()`을 전혀 사용하지 않고 ROS 2 `create_timer()` 기반으로 서보 모터를 0° → 60°까지 부드럽게 저속 제어 (ROS 통신 끊김 방지).
  * `adafruit_servokit` 라이브러리가 없는 개발 PC 환경에서도 로직 테스트가 가능하도록 Mock(더미) 모드 내장.

### 2. `gcs_button_gui.py` (지상 통제소 노트북용 PyQt5 UI 노드)
* 역할: RViz2 화면과 함께 띄워두고 마우스 클릭 한 번으로 `Y` (승강 진행) 또는 `N` (파지 해제) 토픽을 발행하는 GUI 패널.
* 특징:
  * 젯슨의 현재 상태(`IDLE`, `GRASPING`, `WAITING_USER` 등)를 실시간 모니터링.
  * `WAITING_USER` 상태 진입 시 주황색 라벨로 자동 전환되어 조종사의 직관적인 판단 지원.

---

## 📡 인터페이스 명세 (Topics)

| 토픽 이름 | 메시지 타입 | 퍼블리셔 (Pub) | 서브스크라이버 (Sub) | 설명 |
| `/rescue_system/mission_trigger` | `std_msgs/Bool` | 자율 비행 / 미션 플래너 | `rescue_sequence_node` | `true` 수신 시 파지(Grasp) 시퀀스 시작 |
| `/rescue_system/state` | `std_msgs/String` | `rescue_sequence_node` | `gcs_button_gui` / RViz | 현재 미션 장비의 상태 머신 상태 퍼블리시 |
| `/rescue_system/user_input` | `std_msgs/String` | `gcs_button_gui` | `rescue_sequence_node` | 지상 조종사의 승인(`'Y'`) 또는 거부(`'N'`) 신호 |

---

## 🔄 상태 머신 흐름 (State Machine Workflow)

```text
  [IDLE] (0도 개방 대기)
    │
    ▼  <-- /rescue_system/mission_trigger (Bool: true)
[GRASPING] (0도 -> 60도 저속 이동)
    │
    ▼  (60도 도달 시 모터 정지 및 대기)
[WAITING_USER] 
    │
    ├───── 'Y' 수신 ────► [ASCENDING] (승강 레일 구동 시퀀스)
    │
    └───── 'N' 수신 ────► [RELEASING] (60도 -> 0도 저속 해제) ──► [IDLE] 복귀
