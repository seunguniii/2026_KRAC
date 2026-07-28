#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// ros2 run 만으로 실행해도 config/landing_waypoints.yaml 을 스스로 읽기 위한 것들.
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

// 중간 인계 구간에서 조종자의 'y' 키를 논블로킹으로 받기 위한 것들.
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace {

// Timer period. The waypoint setpoint is advanced per cycle, so the speed
// limits below are expressed in m/s and converted with this.
constexpr float kCycleDt = 0.1f;

float clamp_symmetric(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

float clamp_range(float value, float min_value, float max_value) {
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

// 각도차를 -pi..pi 로 접는다. 이걸 안 하면 179 deg -> -179 deg 전환에서
// 기체가 짧은 쪽(2 deg) 대신 긴 쪽(358 deg)으로 돌아버린다.
float wrap_pi(float angle) {
  while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
  while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
  return angle;
}

}  // namespace


class LandingTest : public rclcpp::Node {
public:
  LandingTest() : Node("land_test") {
    odom_sub_ = this->create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const VehicleOdometry::SharedPtr msg) {
        curr_odom_ = *msg;
        has_odom_ = true;
      });

    landed_sub_ = this->create_subscription<VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected",
      rclcpp::SensorDataQoS(),
      [this](const VehicleLandDetected::SharedPtr msg) {
        landed_ = msg->landed;
      });

    desired_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/landing/coordinates",
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        desired_x_ = msg->point.x;   // right(+), [m]
        desired_y_ = msg->point.y;   // forward(+), [m]
        acc_alt_ = -msg->point.z;    // existing convention
        last_setpoint_time_ = this->now();
        has_setpoint_ = true;
      });

    last_setpoint_time_ = this->now();

    declare_parameters();

    // --params-file 없이 `ros2 run` 만으로 띄워도 동작하게, 설치된 기본 설정
    // 파일을 노드가 직접 읽어서 파라미터에 반영한다. 반드시 declare 뒤,
    // 타이머 시작 전에 호출해야 한다.
    load_default_config();

    offboard_control_mode_publisher_ =
      this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_publisher_ =
      this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    vehicle_command_publisher_ =
      this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    mission_mode_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/mission_mode", 10);

    timer_ = this->create_wall_timer(100ms, [this]() { timer_callback(); });
  }

  // PAUSED 에서 터미널을 raw 모드로 바꿔 놓은 채 죽으면 셸이 망가진다.
  ~LandingTest() override { restore_stdin(); }

  // 미션 루프가 성공(0)과 타겟 상실 중단(2)을 구분할 수 있게 한다.
  int exit_code() const { return exit_code_; }

private:
  enum Mission {
    STANDBY,
    FLIGHT,
    PAUSED,
    LANDING,
    FINISHED,
  };

  enum LandingMode {
    POSITION_XY_VELOCITY_Z = 0,
    VELOCITY_XYZ = 1,
  };

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_publisher_;

  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr landed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_sub_;

  // PX4 / mission state
  VehicleOdometry curr_odom_{};

  bool has_odom_ = false;
  bool landed_ = false;
  bool disarm_sent_ = false;
  bool arm_requested_ = false;
  bool offboard_requested_ = false;
  bool nav_land_sent_ = false;

  int preflight_setpoint_count_ = 0;
  int offboard_setpoint_counter_ = 0;

  Mission mission_mode_ = STANDBY;

  // Target state from vision node
  float desired_x_ = 0.0f;  // right(+), [m]
  float desired_y_ = 0.0f;  // forward(+), [m]
  float acc_alt_ = 0.0f;

  bool has_setpoint_ = false;
  rclcpp::Time last_setpoint_time_;

  int lost_count_ = 0;
  int hold_counter_ = 0;

  // 미션 시퀀스 연동
  bool exit_when_done_ = false;
  bool abort_exit_ = false;
  int exit_code_ = 0;
  int exit_delay_ = 0;

  float descent_cmd_ = 0.0f;  // slew-limited descent speed [m/s]
  float v_cmd_ = 0.0f;        // slew-limited horizontal speed [m/s]

  // Parameters
  int start_mode_ = 0;
  int land_mode_ = VELOCITY_XYZ;

  // Legacy approach legs, expressed in BODY/FRD relative to the pose captured
  // at start. NOT absolute NED: the drone always moves relative to where it was
  // and which way it was facing when the node took over.
  // waypoints 파라미터가 비어 있을 때만 쓰이는 폴백이다.
  float start_forward_m_ = 0.0f;  // +x (forward) leg
  float start_right_m_ = 0.0f;    // +y (right) leg
  float start_up_m_ = 0.0f;       // climb, converted to -z (down) internally

  // YAML 로 받는 접근 웨이포인트가 있으면 위 3-leg 대신 이쪽을 쓴다.
  // 최저 고도 가드: 접근 구간에서 이 고도 아래로는 내려가지 않는다. 착륙은
  // LANDING 단계가 담당하므로, FLIGHT 에서 지면까지 찍는 웨이포인트는 오타로 본다.
  float min_alt_m_ = 1.0f;

  float wp_reach_m_ = 0.5f;
  int wp_hold_cycles_ = 20;

  // Approach speed limits. The published setpoint creeps toward the waypoint at
  // these rates instead of jumping, so PX4 never sees a step it wants to chase
  // at MPC_XY_VEL_MAX / MPC_Z_VEL_MAX_UP.
  // YAML 웨이포인트는 고도가 내려갈 수도 있으므로 상승/하강을 따로 제한한다.
  float climb_speed_mps_ = 0.5f;
  float descend_speed_mps_ = 0.5f;
  float cruise_speed_mps_ = 1.2f;

  // Leash: how far the moving setpoint may get ahead of the vehicle. Stops the
  // setpoint running away if the drone falls behind (wind, thrust limit).
  // Must exceed the steady-state tracking error (cruise / MPC_XY_P, roughly
  // 1.2 / 0.95 = 1.3 m) or the setpoint stalls and never reaches the waypoint.
  float max_lead_m_ = 2.5f;

  // The setpoint actually published during FLIGHT, advanced each cycle.
  Eigen::Vector3f setpoint_pos_ = Eigen::Vector3f::Zero();

  // Start pose in NED, captured before the first leg.
  Eigen::Vector3f origin_ = Eigen::Vector3f::Zero();
  float origin_yaw_ = 0.0f;
  int origin_counter_ = 0;
  const int origin_sample_count_ = 10;
  bool origin_done_ = false;

  // Legs converted into absolute NED setpoints, in order.
  std::vector<Eigen::Vector3f> waypoints_ned_;
  size_t wp_idx_ = 0;
  int wp_hold_counter_ = 0;

  // 중간 인계(PAUSED). N 번째 웨이포인트를 찍은 뒤 PX4 를 POSITION 으로 넘기고,
  // 조종자가 'y' 를 누를 때까지 기다렸다가 OFFBOARD 로 복귀해 나머지를 잇는다.
  // 접근 비행 중 기수를 진행 방향으로 돌린다. 목표 yaw 는 "현재 leg 의 방향"
  // (직전 웨이포인트 -> 현재 웨이포인트) 이라 leg 마다 상수다. 기체 위치에서
  // 매번 다시 재면 도착 직전에 벡터가 짧아져 기수가 떨린다.
  bool align_yaw_to_path_ = true;
  float yaw_rate_dps_ = 45.0f;    // 기수 회전 속도 상한 [deg/s]
  float yaw_min_leg_m_ = 0.5f;    // 이보다 짧은 leg 는 방향이 없다고 보고 유지

  float yaw_cmd_ = 0.0f;          // slew 된 yaw 명령 [rad]
  bool yaw_cmd_valid_ = false;    // FLIGHT 진입/재개 때 실제 기수로 다시 잡는다

  // 회전을 끝낸 뒤에 이동한다. 돌면서 가면 경로가 호를 그리고, 카메라도 계속
  // 흔들린다. 기체의 "실제" 기수가 leg 방향에 들어올 때까지 위치 setpoint 를
  // 전진시키지 않는다(명령값이 아니라 실제 기수로 판정해야 의미가 있다).
  float leg_yaw_target_ = 0.0f;   // 현재 leg 의 목표 기수 [rad]
  bool leg_yaw_valid_ = false;    // 현재 leg 에 수평 방향이 있는가
  float yaw_settle_deg_ = 10.0f;  // 이 안에 들어오면 회전 완료로 본다
  float yaw_settle_timeout_s_ = 10.0f;  // 회전이 안 끝나도 이 시간 뒤엔 진행
  int yaw_settle_counter_ = 0;

  int pause_after_wp_ = 0;      // 1-based, 0 이면 인계 없음
  float pause_timeout_s_ = 0.0f;  // 0 이면 무한 대기
  bool pause_done_ = false;
  bool resuming_ = false;
  int pause_counter_ = 0;
  int resume_counter_ = 0;

  bool raw_stdin_active_ = false;
  termios saved_termios_{};

  int lost_abort_ = 700;
  int align_need_ = 5;
  int hold_decay_ = 2;  // hold_counter_ decay per misaligned cycle (soft reset)

  float setpoint_timeout_s_ = 0.5f;  // vision setpoint considered stale after this
  float descent_slew_mps_ = 0.05f;   // max descent-speed change per cycle
  float xy_slew_mps_ = 0.05f;        // max horizontal-speed change per cycle

  float max_xy_ = 0.6f;
  float tol_m_ = 0.8f;
  float deadband_m_ = 0.05f;

  float tanh_min_xy_ = 0.05f;
  float tanh_gain_ = 1.2f;

  float atan_position_gain_ = 1.2f;
  float position_step_max_m_ = 0.50f;
  float position_step_min_m_ = 0.05f;

  float descent_high_mps_ = 0.40f;
  float descent_mid_mps_ = 0.30f;
  float descent_low_mps_ = 0.20f;

  float low_enough_ = -0.7f;

  bool use_q_inverse_ = false;

  const float nan_ = std::numeric_limits<float>::quiet_NaN();

  // Methods
  void declare_parameters();
  void load_default_config();
  void read_parameters();
  void timer_callback();

  void capture_origin();
  std::vector<double> read_waypoint_param() const;
  void build_waypoints();
  void publish_position_setpoint(const Eigen::Vector3f &pos);
  void publish_hold_setpoint();

  void pause_for_operator();
  void enable_raw_stdin();
  void restore_stdin();
  bool resume_key_pressed();
  void advance_setpoint(const Eigen::Vector3f &target, const Eigen::Vector3f &current);

  void arm();
  void disarm();

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f);

  void land();

  float select_descent_speed(float alt_m, bool valid_xy) const;
  float current_yaw() const;
  float path_yaw_setpoint(const Eigen::Vector3f &target);
  Eigen::Quaternionf current_attitude_quaternion() const;
  Eigen::Vector3f body_frd_to_ned(const Eigen::Vector3f &body_frd) const;
};


void LandingTest::declare_parameters() {
  // 0: x/y position setpoint + z velocity setpoint
  // 1: x/y/z velocity setpoint
  this->declare_parameter<int>("land_param", VELOCITY_XYZ);

  // 0: start landing immediately
  // 1: fly the approach waypoints first, then landing
  this->declare_parameter<int>("start_param", 0);

  // 접근 경로 웨이포인트. [forward, right, alt] 를 3 개씩 평탄하게 나열한다.
  // (waypoint_flight / mission_waypoints.yaml 과 같은 형식)
  //
  //   forward / right : 노드가 시작한 시점의 기체 위치·기수(FRD) 기준 [m]
  //                     -> (0, 0, *) 이면 제자리에서 고도만 바꾼다
  //   alt             : EKF 원점(= 이륙 지점) 기준 절대 고도, 위쪽이 양수 [m]
  //                     -> 인계받은 고도가 얼마든 "5 m 상공"은 항상 같은 높이
  //                     -> waypoint_flight / mission_waypoints.yaml 과 같은 기준
  //
  // YAML 예:
  //   landing:
  //     ros__parameters:
  //       start_param: 1
  //       waypoints: [0.0, 0.0, 5.0,
  //                   3.0, 0.0, 5.0,
  //                   3.0, 0.0, 3.0]
  //
  // 값에는 반드시 소수점을 붙인다. [0, 0, 5] 로 쓰면 ROS 가 정수 배열로 파싱한다
  // (아래 dynamic_typing 으로 받아주긴 하지만, 소수/정수를 섞으면 파서가 거부한다).
  //
  // 비어 있으면 예전처럼 start_x/y/z_param 3-leg 로 폴백한다.
  rcl_interfaces::msg::ParameterDescriptor waypoints_desc;
  waypoints_desc.description = "[forward, right, alt] x N, FRD 기준 접근 웨이포인트";
  waypoints_desc.dynamic_typing = true;

  this->declare_parameter(
    "waypoints", rclcpp::ParameterValue(std::vector<double>{}), waypoints_desc);

  // 기본 설정 파일 경로. 비워두면 설치된
  // share/multirotor/config/landing_waypoints.yaml 을 쓴다.
  this->declare_parameter<std::string>("config_file", "");

  // 접근 구간 최저 고도 [m]. 이보다 낮은 웨이포인트는 경고 후 끌어올린다.
  this->declare_parameter<float>("min_alt_m_", 1.0f);

  // 접근 비행 중 기수를 진행 방향으로 맞춘다. false 면 예전처럼 기수를 유지한다.
  this->declare_parameter<bool>("align_yaw_to_path", true);

  // 기수 회전 속도 상한 [deg/s]. 이동하면서 함께 돌기 때문에, 너무 크게 잡으면
  // leg 전환마다 급선회가 된다. MPC_YAWRAUTO_MAX 보다 낮게 두는 것이 안전하다.
  this->declare_parameter<float>("yaw_rate_dps", 45.0f);

  // 회전 완료 판정 [deg]. 기체 실제 기수가 leg 방향에서 이 안에 들어와야
  // 위치 setpoint 가 전진한다. 그 전까지는 제자리에서 돌기만 한다.
  this->declare_parameter<float>("yaw_settle_deg", 10.0f);

  // 회전이 끝나지 않아도 이 시간이 지나면 그냥 진행한다 [s].
  // 바람이나 yaw 제어 한계로 영영 못 들어올 때 미션이 멈추는 걸 막는다.
  this->declare_parameter<float>("yaw_settle_timeout_s", 10.0f);

  // 수평 길이가 이보다 짧은 leg 는 진행 방향이 없다고 본다. 첫 웨이포인트가
  // 제자리 상승(0, 0, 5) 이므로, 이륙/상승 구간에서는 yaw 를 아예 명령하지
  // 않고(NaN) PX4 에 맡긴다. 상승이 끝나고 첫 수평 leg 부터 기수를 잡는다.
  this->declare_parameter<float>("yaw_min_leg_m", 0.5f);

  // 중간 인계: N 번째 웨이포인트(1-based)를 찍은 뒤 PX4 를 POSITION 으로 넘기고
  // 대기한다. 조종자가 실행 터미널에서 'y' 를 누르면 OFFBOARD 로 복귀해
  // 나머지 웨이포인트를 잇는다. 0 이면 인계 없이 끝까지 간다.
  this->declare_parameter<int>("pause_after_wp", 0);

  // 인계 대기 자동 해제 [s]. 0 이면 키를 누를 때까지 무한 대기(기본).
  // 조종 중에 노드가 제멋대로 조종권을 뺏어가지 않게 기본값을 0 으로 둔다.
  this->declare_parameter<float>("pause_timeout_s", 0.0f);

  // launch/백그라운드 실행이라 stdin 이 터미널이 아닐 때의 탈출구.
  //   ros2 param set /land_test resume true
  this->declare_parameter<bool>("resume", false);

  // BODY/FRD offsets relative to the pose captured at start, NOT absolute NED.
  // waypoints 가 비었을 때만 쓰이는 레거시 폴백.
  this->declare_parameter<float>("start_x_param", 0.0f);  // forward leg [m]
  this->declare_parameter<float>("start_y_param", 0.0f);  // right leg [m], negative = left
  this->declare_parameter<float>("start_z_param", 0.0f);  // climb [m], positive = up

  // Waypoint arrival gate. Legs are only a few metres long, so a 3 m threshold
  // would skip them outright and hand over to LANDING while still moving.
  this->declare_parameter<float>("wp_reach_m_", 0.5f);
  this->declare_parameter<int>("wp_hold_cycles_", 20);

  // Approach speeds [m/s]. Keep these well under MPC_Z_VEL_MAX_UP /
  // MPC_XY_VEL_MAX so PX4 can always keep up with the moving setpoint.
  this->declare_parameter<float>("climb_speed_mps_", 0.5f);
  this->declare_parameter<float>("descend_speed_mps_", 0.5f);
  this->declare_parameter<float>("cruise_speed_mps_", 1.2f);
  this->declare_parameter<float>("max_lead_m_", 2.5f);

  // 미션 시퀀스에서 다음 노드로 넘기기 위해 종료 시 프로세스를 내린다.
  this->declare_parameter<bool>("exit_when_done", false);

  this->declare_parameter<int>("lost_abort_", 700);
  this->declare_parameter<float>("max_xy_", 0.4f);
  this->declare_parameter<float>("tol_m_", 0.8f);
  this->declare_parameter<int>("align_need_", 5);
  this->declare_parameter<int>("hold_decay_", 2);

  // Vision setpoint older than this (seconds) is treated as lost. Keep it above
  // the vision publish period so normal message gaps do not flap valid/invalid.
  this->declare_parameter<float>("setpoint_timeout_s_", 0.5f);

  // Speed slew limits [m/s per cycle]. 0.05 per 0.1 s cycle = 0.5 m/s^2.
  // vz 만 제한하고 수평을 열어두면, 모드 전환이나 비전 측정 튐에서 한 사이클
  // 만에 0 -> max_xy_ 계단 입력이 나가 기체가 러칭한다. 둘 다 제한한다.
  this->declare_parameter<float>("descent_slew_mps_", 0.05f);
  this->declare_parameter<float>("xy_slew_mps_", 0.05f);

  this->declare_parameter<float>("deadband_m_", 0.05f);

  // Velocity-based x/y controller.
  this->declare_parameter<float>("tanh_min_xy_", 0.10f);
  this->declare_parameter<float>("tanh_gain_", 1.2f);

  // Position-based x/y controller.
  this->declare_parameter<float>("atan_position_gain_", 1.2f);
  this->declare_parameter<float>("position_step_max_m_", 0.50f);
  this->declare_parameter<float>("position_step_min_m_", 0.05f);

  // z is always velocity-based.
  this->declare_parameter<float>("descent_high_mps_", 0.40f);
  this->declare_parameter<float>("descent_mid_mps_", 0.30f);
  this->declare_parameter<float>("descent_low_mps_", 0.20f);
}


void LandingTest::load_default_config() {
  // 명령행(-p) 이나 --params-file 로 직접 준 값은 항상 우선한다. 자동 로드가
  // 그걸 덮으면 mission_loop 의 `-p start_param:=0` 같은 게 무시돼서 위험하다.
  const auto &overrides =
    this->get_node_parameters_interface()->get_parameter_overrides();

  if (overrides.count("waypoints") > 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "[CONFIG] waypoints 를 명령행에서 받았다. 기본 설정 파일은 읽지 않는다.");
    return;
  }

  std::string path = this->get_parameter("config_file").as_string();

  if (path.empty()) {
    try {
      path = ament_index_cpp::get_package_share_directory("multirotor") +
             "/config/landing_waypoints.yaml";
    } catch (const std::exception &e) {
      RCLCPP_WARN(
        this->get_logger(),
        "[CONFIG] 패키지 share 경로를 찾지 못했다: %s", e.what());
      return;
    }
  }

  YAML::Node root;

  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception &e) {
    RCLCPP_WARN(
      this->get_logger(),
      "[CONFIG] 기본 설정 파일을 읽지 못했다 (%s): %s. 파라미터 기본값으로 진행한다.",
      path.c_str(),
      e.what());

    return;
  }

  // 노드 이름 섹션을 먼저 보고, 없으면 ROS 와일드카드 '/**' 를 본다.
  YAML::Node section = root[this->get_name()];
  if (!section) section = root["/**"];

  if (!section || !section["ros__parameters"]) {
    RCLCPP_WARN(
      this->get_logger(),
      "[CONFIG] %s 에 '%s:' / '/**:' 아래 ros__parameters 섹션이 없다.",
      path.c_str(),
      this->get_name());

    return;
  }

  const YAML::Node params = section["ros__parameters"];

  int applied = 0;
  int kept = 0;

  for (const auto &kv : params) {
    const std::string name = kv.first.as<std::string>();

    if (overrides.count(name) > 0) {
      kept++;  // 명령행이 이겼다
      continue;
    }

    if (!this->has_parameter(name)) {
      RCLCPP_WARN(
        this->get_logger(), "[CONFIG] 선언되지 않은 파라미터 '%s' 무시", name.c_str());
      continue;
    }

    try {
      // 선언된 타입에 맞춰 변환한다. 덕분에 이 경로에서는 [0, 0, 5] 처럼
      // 정수로 써도 되고, ROS YAML 파서의 정수/소수 혼용 제약도 받지 않는다.
      switch (this->get_parameter(name).get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          this->set_parameter(rclcpp::Parameter(name, kv.second.as<bool>()));
          break;

        case rclcpp::ParameterType::PARAMETER_INTEGER:
          this->set_parameter(rclcpp::Parameter(name, kv.second.as<int64_t>()));
          break;

        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          this->set_parameter(rclcpp::Parameter(name, kv.second.as<double>()));
          break;

        case rclcpp::ParameterType::PARAMETER_STRING:
          this->set_parameter(rclcpp::Parameter(name, kv.second.as<std::string>()));
          break;

        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
          std::vector<double> values;
          values.reserve(kv.second.size());

          for (const auto &item : kv.second) {
            values.push_back(item.as<double>());
          }

          this->set_parameter(rclcpp::Parameter(name, values));
          break;
        }

        default:
          RCLCPP_WARN(
            this->get_logger(),
            "[CONFIG] '%s' 는 지원하지 않는 타입이라 건너뛴다", name.c_str());
          continue;
      }

      applied++;
    } catch (const std::exception &e) {
      RCLCPP_WARN(
        this->get_logger(),
        "[CONFIG] '%s' 적용 실패: %s", name.c_str(), e.what());
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[CONFIG] %s 에서 파라미터 %d 개 적용 (명령행 우선 %d 개 유지)",
    path.c_str(),
    applied,
    kept);
}


void LandingTest::read_parameters() {
  land_mode_ = this->get_parameter("land_param").as_int();
  start_mode_ = this->get_parameter("start_param").as_int();

  start_forward_m_ = static_cast<float>(this->get_parameter("start_x_param").as_double());
  start_right_m_ = static_cast<float>(this->get_parameter("start_y_param").as_double());
  start_up_m_ = static_cast<float>(this->get_parameter("start_z_param").as_double());

  min_alt_m_ = static_cast<float>(this->get_parameter("min_alt_m_").as_double());

  align_yaw_to_path_ = this->get_parameter("align_yaw_to_path").as_bool();
  yaw_rate_dps_ = static_cast<float>(this->get_parameter("yaw_rate_dps").as_double());
  yaw_min_leg_m_ = static_cast<float>(this->get_parameter("yaw_min_leg_m").as_double());
  yaw_settle_deg_ = static_cast<float>(this->get_parameter("yaw_settle_deg").as_double());
  yaw_settle_timeout_s_ =
    static_cast<float>(this->get_parameter("yaw_settle_timeout_s").as_double());

  pause_after_wp_ = static_cast<int>(this->get_parameter("pause_after_wp").as_int());
  pause_timeout_s_ =
    static_cast<float>(this->get_parameter("pause_timeout_s").as_double());

  wp_reach_m_ = static_cast<float>(this->get_parameter("wp_reach_m_").as_double());
  wp_hold_cycles_ = this->get_parameter("wp_hold_cycles_").as_int();

  climb_speed_mps_ = static_cast<float>(this->get_parameter("climb_speed_mps_").as_double());
  descend_speed_mps_ =
    static_cast<float>(this->get_parameter("descend_speed_mps_").as_double());
  cruise_speed_mps_ = static_cast<float>(this->get_parameter("cruise_speed_mps_").as_double());
  max_lead_m_ = static_cast<float>(this->get_parameter("max_lead_m_").as_double());

  exit_when_done_ = this->get_parameter("exit_when_done").as_bool();

  lost_abort_ = this->get_parameter("lost_abort_").as_int();
  align_need_ = this->get_parameter("align_need_").as_int();
  hold_decay_ = this->get_parameter("hold_decay_").as_int();

  setpoint_timeout_s_ =
    static_cast<float>(this->get_parameter("setpoint_timeout_s_").as_double());
  descent_slew_mps_ =
    static_cast<float>(this->get_parameter("descent_slew_mps_").as_double());
  xy_slew_mps_ =
    static_cast<float>(this->get_parameter("xy_slew_mps_").as_double());

  max_xy_ = static_cast<float>(this->get_parameter("max_xy_").as_double());
  tol_m_ = static_cast<float>(this->get_parameter("tol_m_").as_double());
  deadband_m_ = static_cast<float>(this->get_parameter("deadband_m_").as_double());

  tanh_min_xy_ = static_cast<float>(this->get_parameter("tanh_min_xy_").as_double());
  tanh_gain_ = static_cast<float>(this->get_parameter("tanh_gain_").as_double());

  atan_position_gain_ =
    static_cast<float>(this->get_parameter("atan_position_gain_").as_double());

  position_step_max_m_ =
    static_cast<float>(this->get_parameter("position_step_max_m_").as_double());

  position_step_min_m_ =
    static_cast<float>(this->get_parameter("position_step_min_m_").as_double());

  descent_high_mps_ =
    static_cast<float>(this->get_parameter("descent_high_mps_").as_double());

  descent_mid_mps_ =
    static_cast<float>(this->get_parameter("descent_mid_mps_").as_double());

  descent_low_mps_ =
    static_cast<float>(this->get_parameter("descent_low_mps_").as_double());
}


void LandingTest::timer_callback() {
  if (!has_odom_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  read_parameters();

  // start_param=0 이면 접근 비행이 없으므로 STANDBY 를 거치지 않는다.
  // STANDBY 를 거치면 PX4 가 OFFBOARD 위치홀드로 먼저 진입한 뒤 곧바로
  // position -> velocity 로 제어 모드가 바뀌면서 전환 충격이 생긴다.
  // 처음부터 LANDING 으로 두면 OFFBOARD 진입 시점에 이미 속도제어 스트림이
  // 흐르고 있어서 모드 전환이 POSITION -> OFFBOARD 한 번으로 끝난다.
  if (start_mode_ == 0 && mission_mode_ == STANDBY) {
    mission_mode_ = LANDING;
  }

  publish_offboard_control_mode();

  std_msgs::msg::String mission_msg;

  switch (mission_mode_) {
    case STANDBY:
      // Freeze on the captured start pose so PX4 gets a valid setpoint stream
      // before OFFBOARD/ARM. Arming under a far-away setpoint is what made the
      // old version lunge the moment offboard engaged.
      capture_origin();
      publish_hold_setpoint();
      mission_msg.data = "STANDBY";
      break;

    case FLIGHT:
      publish_trajectory_setpoint();
      mission_msg.data = "FLIGHT";
      break;

    case PAUSED:
      pause_for_operator();
      mission_msg.data = "PAUSED";
      break;

    case LANDING:
      land();
      mission_msg.data = "LANDING";
      break;

    case FINISHED:
    default:
      if (landed_ && !disarm_sent_) {
        disarm();
        disarm_sent_ = true;
      }

      mission_msg.data = "FINISHED";
      mission_mode_publisher_->publish(mission_msg);

      // 미션 시퀀스에서는 이 노드가 끝나야 다음 단계로 넘어간다.
      // disarm이 반영될 시간을 조금 준 뒤 내려간다.
      if (exit_when_done_) {
        if (disarm_sent_ || abort_exit_) {
          if (++exit_delay_ > 10) {
            RCLCPP_INFO(
              this->get_logger(),
              "[EXIT] 착륙 시퀀스 종료 (code=%d)", exit_code_);
            rclcpp::shutdown();
          }
        }
      }

      return;
  }

  mission_mode_publisher_->publish(mission_msg);

  // PX4 Offboard requires a short setpoint stream before mode switch.
  if (!offboard_requested_) {
    preflight_setpoint_count_++;

    if (preflight_setpoint_count_ > 20) {
      RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode");
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      offboard_requested_ = true;
    }

    return;
  }

  if (!arm_requested_) {
    RCLCPP_INFO(this->get_logger(), "Requesting ARM");
    arm();
    arm_requested_ = true;
    return;
  }

  // Armed and holding on the start pose: commit to the mission.
  // start_param=0 은 위에서 이미 LANDING 으로 넘어갔으므로 여기는 접근 비행 전용.
  if (mission_mode_ == STANDBY && origin_done_) {
    build_waypoints();
    mission_mode_ = FLIGHT;
  }

  offboard_setpoint_counter_++;
}


void LandingTest::capture_origin() {
  if (origin_done_) return;

  if (origin_counter_ < origin_sample_count_) {
    origin_ += Eigen::Vector3f(
      curr_odom_.position[0],
      curr_odom_.position[1],
      curr_odom_.position[2]);

    origin_counter_++;
    return;
  }

  origin_ /= static_cast<float>(origin_sample_count_);

  const float w = curr_odom_.q[0];
  const float x = curr_odom_.q[1];
  const float y = curr_odom_.q[2];
  const float z = curr_odom_.q[3];

  origin_yaw_ = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

  origin_done_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "[ORIGIN] NED=(%.2f, %.2f, %.2f) yaw=%.1f deg",
    origin_[0],
    origin_[1],
    origin_[2],
    origin_yaw_ * 180.0f / static_cast<float>(M_PI));
}


std::vector<double> LandingTest::read_waypoint_param() const {
  // dynamic_typing 으로 선언했으므로 [0.0, ...] 도 [0, ...] 도 받아준다.
  // (정수로만 쓴 YAML 때문에 노드가 뜨자마자 죽는 일이 없게)
  const rclcpp::Parameter param = this->get_parameter("waypoints");

  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return param.as_double_array();

    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      const std::vector<int64_t> raw = param.as_integer_array();
      return std::vector<double>(raw.begin(), raw.end());
    }

    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      return {};

    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "[FLIGHT] waypoints 파라미터 타입이 배열이 아니다 (%s). 무시한다.",
        param.get_type_name().c_str());
      return {};
  }
}


void LandingTest::build_waypoints() {
  const float cos_yaw = std::cos(origin_yaw_);
  const float sin_yaw = std::sin(origin_yaw_);

  // Rotate an FRD offset by the start yaw, then anchor it on the start pose.
  const auto frd_to_ned_xy = [&](float forward, float right) {
    return Eigen::Vector2f(
      origin_[0] + forward * cos_yaw - right * sin_yaw,
      origin_[1] + forward * sin_yaw + right * cos_yaw);
  };

  waypoints_ned_.clear();

  const std::vector<double> wp = read_waypoint_param();
  const bool use_yaml = !wp.empty() && (wp.size() % 3 == 0);

  if (!wp.empty() && !use_yaml) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[FLIGHT] waypoints 길이가 %zu 로 3 의 배수가 아니다. "
      "[forward, right, alt] 를 3 개씩 나열해야 한다. "
      "레거시 start_*_param 경로로 대체한다.",
      wp.size());
  }

  if (use_yaml) {
    waypoints_ned_.reserve(wp.size() / 3);

    for (size_t i = 0; i < wp.size(); i += 3) {
      const float forward = static_cast<float>(wp[i]);
      const float right = static_cast<float>(wp[i + 1]);
      float alt = static_cast<float>(wp[i + 2]);

      // FLIGHT 는 접근 구간이다. 지면까지 찍힌 웨이포인트는 오타로 보고 끌어올린다.
      if (alt < min_alt_m_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[FLIGHT] wp %zu 의 alt %.2f m 가 min_alt_m_(%.2f m) 보다 낮다 -> %.2f m 로 올림",
          waypoints_ned_.size() + 1,
          alt,
          min_alt_m_,
          min_alt_m_);

        alt = min_alt_m_;
      }

      const Eigen::Vector2f ned_xy = frd_to_ned_xy(forward, right);

      // alt 는 EKF 원점 기준 절대 고도라서 NED z 는 부호만 뒤집으면 된다.
      // origin_[2] 를 더하지 않는 것이 핵심: 인계 고도가 달라도 결과가 같다.
      waypoints_ned_.emplace_back(ned_xy[0], ned_xy[1], -alt);
    }
  } else {
    // 레거시 폴백: 누적 BODY/FRD leg (상승 -> 우측 -> 전진).
    // 이쪽 고도는 절대 고도가 아니라 시작 위치 기준 상대값이다.
    const std::vector<Eigen::Vector3f> legs_frd = {
      {0.0f,              0.0f,             -start_up_m_},  // 1) climb
      {0.0f,              start_right_m_,   -start_up_m_},  // 2) sideways
      {start_forward_m_,  start_right_m_,   -start_up_m_},  // 3) forward
    };

    waypoints_ned_.reserve(legs_frd.size());

    for (const auto &leg : legs_frd) {
      const Eigen::Vector2f ned_xy = frd_to_ned_xy(leg[0], leg[1]);
      waypoints_ned_.emplace_back(ned_xy[0], ned_xy[1], origin_[2] + leg[2]);
    }
  }

  wp_idx_ = 0;
  wp_hold_counter_ = 0;

  // 기수 명령은 FLIGHT 첫 사이클에 실제 기수에서 잡는다.
  yaw_cmd_valid_ = false;

  // Start the moving setpoint on the vehicle, not on the first waypoint.
  setpoint_pos_ = origin_;

  RCLCPP_INFO(
    this->get_logger(),
    "[FLIGHT] %s 웨이포인트 %zu 개 "
    "(cruise %.2f, climb %.2f, descend %.2f m/s | reach %.2f m, hold %d cycles)",
    use_yaml ? "YAML" : "레거시 start_*_param",
    waypoints_ned_.size(),
    cruise_speed_mps_,
    climb_speed_mps_,
    descend_speed_mps_,
    wp_reach_m_,
    wp_hold_cycles_);

  for (size_t i = 0; i < waypoints_ned_.size(); ++i) {
    RCLCPP_INFO(
      this->get_logger(),
      "  wp %zu/%zu  NED=(%.2f, %.2f, %.2f)  alt=%.2f m",
      i + 1,
      waypoints_ned_.size(),
      waypoints_ned_[i][0],
      waypoints_ned_[i][1],
      waypoints_ned_[i][2],
      -waypoints_ned_[i][2]);
  }
}


void LandingTest::advance_setpoint(
  const Eigen::Vector3f &target,
  const Eigen::Vector3f &current) {
  const Eigen::Vector3f to_target = target - setpoint_pos_;

  // Horizontal and vertical are rate-limited separately so a diagonal leg does
  // not climb faster than climb_speed_mps_.
  Eigen::Vector2f step_xy(to_target[0], to_target[1]);
  const float dist_xy = step_xy.norm();
  const float max_step_xy = cruise_speed_mps_ * kCycleDt;

  if (dist_xy > max_step_xy) {
    step_xy *= max_step_xy / dist_xy;
  }

  // NED z 는 아래가 양수다. to_target[2] < 0 이면 올라가는 방향.
  const float z_speed = (to_target[2] < 0.0f) ? climb_speed_mps_ : descend_speed_mps_;
  const float step_z = clamp_symmetric(to_target[2], z_speed * kCycleDt);

  const Eigen::Vector3f candidate =
    setpoint_pos_ + Eigen::Vector3f(step_xy[0], step_xy[1], step_z);

  // Hold the setpoint if the vehicle has not caught up yet, so the error (and
  // therefore the commanded speed) stays bounded.
  if ((candidate - current).norm() <= max_lead_m_) {
    setpoint_pos_ = candidate;
  }
}


void LandingTest::publish_position_setpoint(const Eigen::Vector3f &pos) {
  TrajectorySetpoint msg{};

  msg.position = {pos[0], pos[1], pos[2]};
  msg.velocity = {nan_, nan_, nan_};
  msg.yaw = nan_;
  msg.yawspeed = nan_;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  trajectory_setpoint_publisher_->publish(msg);
}


void LandingTest::publish_hold_setpoint() {
  // Before the origin average completes there is nothing to hold but the
  // current position.
  const Eigen::Vector3f hold = origin_done_
    ? origin_
    : Eigen::Vector3f(
        curr_odom_.position[0],
        curr_odom_.position[1],
        curr_odom_.position[2]);

  publish_position_setpoint(hold);
}


void LandingTest::enable_raw_stdin() {
  if (raw_stdin_active_) return;

  if (!isatty(STDIN_FILENO)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[PAUSE] stdin 이 터미널이 아니다 (launch/백그라운드 실행). 키 입력을 받을 수 "
      "없으니 'ros2 param set /land_test resume true' 로 재개해라.");
    return;
  }

  if (tcgetattr(STDIN_FILENO, &saved_termios_) != 0) {
    RCLCPP_ERROR(this->get_logger(), "[PAUSE] tcgetattr 실패. 키 입력 비활성.");
    return;
  }

  termios raw = saved_termios_;

  // 엔터 없이 한 글자씩 받고, 화면에 에코하지 않는다.
  raw.c_lflag &= ~(static_cast<tcflag_t>(ICANON) | static_cast<tcflag_t>(ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;

  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
    RCLCPP_ERROR(this->get_logger(), "[PAUSE] tcsetattr 실패. 키 입력 비활성.");
    return;
  }

  raw_stdin_active_ = true;
}


void LandingTest::restore_stdin() {
  if (!raw_stdin_active_) return;

  tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios_);
  raw_stdin_active_ = false;
}


bool LandingTest::resume_key_pressed() {
  if (!raw_stdin_active_) return false;

  bool pressed = false;

  // 쌓인 입력을 모두 비운다. 그래야 이전에 눌린 엔터 같은 게 남지 않는다.
  for (;;) {
    pollfd pfd{};
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;

    if (poll(&pfd, 1, 0) <= 0) break;
    if (!(pfd.revents & POLLIN)) break;

    char c = 0;
    if (read(STDIN_FILENO, &c, 1) != 1) break;

    if (c == 'y' || c == 'Y') pressed = true;
  }

  return pressed;
}


void LandingTest::pause_for_operator() {
  const Eigen::Vector3f current(
    curr_odom_.position[0],
    curr_odom_.position[1],
    curr_odom_.position[2]);

  if (pause_counter_ == 0) {
    enable_raw_stdin();

    // POSITION 으로 넘겨서 조종자가 직접 몰 수 있게 한다.
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f);

    RCLCPP_INFO(
      this->get_logger(),
      "[PAUSE] 웨이포인트 %d 개 완료 -> PX4 POSITION 모드로 인계. "
      "조종이 끝나면 이 터미널에서 'y' 를 눌러라 "
      "(또는 ros2 param set /land_test resume true).",
      pause_after_wp_);
  }

  pause_counter_++;

  // OFFBOARD 재진입에는 살아있는 setpoint 스트림이 필요하다. 조종자가 기체를
  // 옮겨도 재진입 순간 제자리 홀드가 되도록 "현재 위치"를 계속 흘린다.
  // 인계 직전 좌표를 그대로 흘리면 OFFBOARD 로 돌아오는 순간 그리로 튄다.
  setpoint_pos_ = current;
  publish_position_setpoint(current);

  if (!resuming_) {
    const bool timed_out =
      pause_timeout_s_ > 0.0f &&
      (static_cast<float>(pause_counter_) * kCycleDt) > pause_timeout_s_;

    if (resume_key_pressed() || this->get_parameter("resume").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "[PAUSE] 재개 요청 수신 -> OFFBOARD 재진입");
      resuming_ = true;
      resume_counter_ = 0;
    } else if (timed_out) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PAUSE] pause_timeout_s(%.1f s) 초과 -> 자동 재개", pause_timeout_s_);
      resuming_ = true;
      resume_counter_ = 0;
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "[PAUSE] 대기 중 (%.1f s). 'y' 를 누르면 OFFBOARD 로 복귀한다.",
        static_cast<float>(pause_counter_) * kCycleDt);

      return;
    }
  }

  // 재진입. setpoint 를 계속 흘리면서 OFFBOARD 명령을 1 초 간격으로 두 번 보낸다.
  // vehicle_status 를 구독하지 않으므로 확인 대신 재전송으로 확실히 한다.
  if (resume_counter_ % 10 == 0) {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }

  resume_counter_++;

  if (resume_counter_ < 20) return;

  restore_stdin();

  pause_done_ = true;
  resuming_ = false;
  wp_hold_counter_ = 0;

  if (wp_idx_ >= waypoints_ned_.size()) {
    hold_counter_ = 0;
    mission_mode_ = LANDING;
    RCLCPP_INFO(this->get_logger(), "[LANDING] 재개 후 남은 웨이포인트 없음 -> 착륙 시퀀스");
    return;
  }

  mission_mode_ = FLIGHT;

  RCLCPP_INFO(
    this->get_logger(),
    "[FLIGHT] 재개 -> wp %zu/%zu",
    wp_idx_ + 1,
    waypoints_ned_.size());
}


void LandingTest::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void LandingTest::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


void LandingTest::publish_offboard_control_mode() {
  OffboardControlMode msg{};

  if (mission_mode_ == LANDING) {
    if (land_mode_ == POSITION_XY_VELOCITY_Z) {
      msg.position = true;   // x/y
      msg.velocity = true;   // z
    } else {
      msg.position = false;
      msg.velocity = true;   // x/y/z
    }
  } else {
    msg.position = true;
    msg.velocity = false;
  }

  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  offboard_control_mode_publisher_->publish(msg);
}


void LandingTest::publish_trajectory_setpoint() {
  if (curr_odom_.timestamp == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  if (waypoints_ned_.empty() || wp_idx_ >= waypoints_ned_.size()) {
    return;
  }

  TrajectorySetpoint msg{};

  Eigen::Vector3f current(
    curr_odom_.position[0],
    curr_odom_.position[1],
    curr_odom_.position[2]);

  const Eigen::Vector3f &target = waypoints_ned_[wp_idx_];
  const float dist = (target - current).norm();

  // 기수를 진행 방향으로. align_yaw_to_path 가 꺼져 있으면 NaN 이 나가서
  // 예전처럼 기수를 유지한다. yaw 를 0 으로 두면 북쪽으로 돌라는 명령이 된다.
  // 위치보다 먼저 계산해야 아래 회전 완료 판정에 쓸 수 있다.
  const float yaw_setpoint = path_yaw_setpoint(target);

  // 회전을 끝내고 이동한다. 명령값(yaw_cmd_)이 아니라 기체의 실제 기수로
  // 판정한다. 명령값은 slew 라 먼저 도착하므로 그걸로 재면 아직 돌고 있는데
  // 출발해 버린다.
  bool yaw_settled = true;

  if (align_yaw_to_path_ && leg_yaw_valid_) {
    const float yaw_err =
      std::fabs(wrap_pi(leg_yaw_target_ - current_yaw())) * 180.0f /
      static_cast<float>(M_PI);

    yaw_settled = yaw_err <= yaw_settle_deg_;

    if (yaw_settled) {
      yaw_settle_counter_ = 0;
    } else {
      yaw_settle_counter_++;

      // 바람이나 yaw 제어 한계로 영영 못 들어오면 미션이 멈춘다. 그건 더 나쁘다.
      if (static_cast<float>(yaw_settle_counter_) * kCycleDt > yaw_settle_timeout_s_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[FLIGHT] 회전이 %.1f s 안에 안 끝났다 (오차 %.1f deg). 그대로 진행한다.",
          yaw_settle_timeout_s_,
          yaw_err);

        yaw_settled = true;
        yaw_settle_counter_ = 0;
      } else {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "[FLIGHT] wp %zu/%zu 회전 중 (오차 %.1f deg > %.1f). 이동 대기.",
          wp_idx_ + 1,
          waypoints_ned_.size(),
          yaw_err,
          yaw_settle_deg_);
      }
    }
  }

  // Creep the setpoint toward the waypoint instead of commanding it directly.
  // 회전이 끝나기 전에는 setpoint 를 전진시키지 않는다 -> 제자리에서 돌기만 한다.
  if (yaw_settled) {
    advance_setpoint(target, current);
  }

  msg.position = {setpoint_pos_[0], setpoint_pos_[1], setpoint_pos_[2]};
  msg.velocity = {nan_, nan_, nan_};

  msg.yaw = yaw_setpoint;
  msg.yawspeed = nan_;

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);

  // yaw 를 실제로 명령하고 있는지(각도) 아니면 손대지 않는지(none) 구분해서 찍는다.
  const std::string yaw_cmd_str = std::isfinite(msg.yaw)
    ? std::to_string(static_cast<int>(std::lround(msg.yaw * 180.0f / M_PI))) + " deg"
    : std::string("none");

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "[FLIGHT] wp %zu/%zu dist=%.2f m lead=%.2f m alt=%.2f->%.2f m "
    "yaw=%.0f deg (cmd %s) hold=%d/%d",
    wp_idx_ + 1,
    waypoints_ned_.size(),
    dist,
    (setpoint_pos_ - current).norm(),
    -current[2],
    -target[2],
    current_yaw() * 180.0f / static_cast<float>(M_PI),
    yaw_cmd_str.c_str(),
    wp_hold_counter_,
    wp_hold_cycles_);

  if (dist >= wp_reach_m_) {
    wp_hold_counter_ = 0;
    return;
  }

  wp_hold_counter_++;

  if (wp_hold_counter_ <= wp_hold_cycles_) {
    return;
  }

  wp_hold_counter_ = 0;
  wp_idx_++;

  // 새 leg 는 새 회전이다. 이전 leg 에서 쌓인 대기 시간을 물려주지 않는다.
  yaw_settle_counter_ = 0;

  // 중간 인계 지점에 도달했으면 조종자에게 넘긴다.
  // 남은 웨이포인트가 없더라도(= 마지막 wp 뒤 인계) PAUSED 를 거친다.
  if (!pause_done_ && pause_after_wp_ > 0 &&
      wp_idx_ == static_cast<size_t>(pause_after_wp_)) {
    pause_counter_ = 0;
    resume_counter_ = 0;
    resuming_ = false;

    // 인계 중 조종자가 기체를 돌릴 수 있다. 재개할 때 실제 기수에서 다시 잡는다.
    yaw_cmd_valid_ = false;

    mission_mode_ = PAUSED;
    return;
  }

  if (wp_idx_ < waypoints_ned_.size()) {
    RCLCPP_INFO(
      this->get_logger(),
      "[FLIGHT] waypoint reached, heading to %zu/%zu",
      wp_idx_ + 1,
      waypoints_ned_.size());

    return;
  }

  // hold_counter_ is shared with the landing alignment gate.
  hold_counter_ = 0;
  mission_mode_ = LANDING;

  RCLCPP_INFO(this->get_logger(), "[LANDING] Initiating landing sequence");
}


float LandingTest::select_descent_speed(float alt_m, bool valid_xy) const {
  if (!valid_xy || hold_counter_ < align_need_) {
    return 0.0f;
  }

  if (alt_m > 2.0f) {
    return descent_high_mps_;
  }

  if (alt_m > 0.8f) {
    return descent_mid_mps_;
  }

  return descent_low_mps_;
}


float LandingTest::current_yaw() const {
  const float w = curr_odom_.q[0];
  const float x = curr_odom_.q[1];
  const float y = curr_odom_.q[2];
  const float z = curr_odom_.q[3];

  return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}


float LandingTest::path_yaw_setpoint(const Eigen::Vector3f &target) {
  if (!align_yaw_to_path_) return nan_;

  // 진행 방향은 "leg 방향" (직전 웨이포인트 -> 현재 목표) 으로 잡는다.
  // leg 마다 상수라서 도착 직전에도 기수가 떨리지 않는다.
  const Eigen::Vector3f &leg_start =
    (wp_idx_ == 0) ? origin_ : waypoints_ned_[wp_idx_ - 1];

  const Eigen::Vector2f leg(target[0] - leg_start[0], target[1] - leg_start[1]);

  leg_yaw_valid_ = leg.norm() >= yaw_min_leg_m_;

  // 수평 방향이 없는 leg (제자리 상승/하강).
  if (!leg_yaw_valid_) {
    // 아직 한 번도 기수를 잡은 적이 없으면 = 이륙/상승 구간이다. NaN 을 내보내
    // PX4 의 yaw 제어를 건드리지 않는다. 여기서 current_yaw() 를 "유지하라"고
    // 실어 보내면, 그것도 엄연히 그 각도를 잡으라는 명령이라 기체가 미세하게
    // 돌아간다. 상승이 끝나고 첫 수평 leg 가 시작될 때부터만 기수를 잡는다.
    return yaw_cmd_valid_ ? yaw_cmd_ : nan_;
  }

  // 첫 수평 leg. 지금 향하고 있는 실제 기수에서 출발해 부드럽게 돌린다.
  // 0 이나 origin_yaw_ 에서 출발하면 그 각도로 한 번 튀고 나서 돌게 된다.
  if (!yaw_cmd_valid_) {
    yaw_cmd_ = current_yaw();
    yaw_cmd_valid_ = true;
  }

  // NED 이므로 x = north, y = east. PX4 yaw 도 북쪽 기준 절대각이다.
  leg_yaw_target_ = std::atan2(leg[1], leg[0]);

  const float max_step =
    yaw_rate_dps_ * static_cast<float>(M_PI) / 180.0f * kCycleDt;

  yaw_cmd_ = wrap_pi(
    yaw_cmd_ + clamp_symmetric(wrap_pi(leg_yaw_target_ - yaw_cmd_), max_step));

  return yaw_cmd_;
}


Eigen::Quaternionf LandingTest::current_attitude_quaternion() const {
  Eigen::Quaternionf q(
    curr_odom_.q[0],
    curr_odom_.q[1],
    curr_odom_.q[2],
    curr_odom_.q[3]);

  q.normalize();
  return q;
}


Eigen::Vector3f LandingTest::body_frd_to_ned(const Eigen::Vector3f &body_frd) const {
  const Eigen::Quaternionf q = current_attitude_quaternion();

  if (use_q_inverse_) {
    return q.conjugate() * body_frd;
  }

  return q * body_frd;
}


void LandingTest::land() {
  TrajectorySetpoint msg{};

  // Hold current heading throughout landing; yaw=0 (the msg{} default) would
  // command a spin to North on every setpoint.
  msg.yaw = nan_;
  msg.yawspeed = nan_;

  const float alt_m = -acc_alt_;

  // Fix: staleness. isfinite() alone stays true when the vision node dies
  // silently (last values freeze), so the lost-target safety never fires.
  // Require a fresh setpoint as well.
  const double setpoint_age =
    has_setpoint_ ? (this->now() - last_setpoint_time_).seconds() : 1.0e9;
  const bool fresh = setpoint_age < setpoint_timeout_s_;

  const bool valid_xy =
    fresh &&
    std::isfinite(desired_x_) &&
    std::isfinite(desired_y_);

  const bool aligned =
    valid_xy &&
    std::fabs(desired_x_) < tol_m_ &&
    std::fabs(desired_y_) < tol_m_;

  if (!valid_xy) {
    lost_count_++;
    hold_counter_ = 0;
  } else {
    lost_count_ = 0;
    // Soft decay instead of hard reset so a brief excursion past tol_m_ does
    // not instantly zero the descent gate (avoids stutter descent).
    hold_counter_ = aligned ? hold_counter_ + 1 : std::max(0, hold_counter_ - hold_decay_);
  }

  if (lost_count_ > lost_abort_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[LANDING] target lost too long -> switch PX4 to POSITION mode");

    publish_vehicle_command(
      VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0f,
      3.0f);

    mission_mode_ = FINISHED;
    abort_exit_ = true;
    exit_code_ = 2;  // 타겟 상실 중단. 미션 루프가 성공과 구분할 수 있게.
    return;
  }

  const float ex = valid_xy ? desired_x_ : 0.0f;  // right(+)
  const float ey = valid_xy ? desired_y_ : 0.0f;  // forward(+)
  const float err_dist = std::sqrt(ex * ex + ey * ey);

  // OFFBOARD 진입 전 약 2초 동안 PX4 는 setpoint 를 무시한다(모드 전환 전에
  // setpoint 스트림이 먼저 흘러야 하기 때문). 그 사이에 램프가 미리 쌓이면
  // 진입하는 순간 최대 속도 명령이 그대로 들어가므로, 진입 전까지 0 으로 묶는다.
  const bool control_engaged = offboard_requested_ && arm_requested_;

  const float descent_target =
    control_engaged ? select_descent_speed(alt_m, valid_xy) : 0.0f;

  // Slew-limit the descent speed so vz never steps abruptly (including the
  // drop to 0 when alignment is briefly lost).
  if (descent_target > descent_cmd_) {
    descent_cmd_ = std::min(descent_target, descent_cmd_ + descent_slew_mps_);
  } else {
    descent_cmd_ = std::max(descent_target, descent_cmd_ - descent_slew_mps_);
  }
  const float descent_mps = descent_cmd_;

  if (land_mode_ == POSITION_XY_VELOCITY_Z) {
    Eigen::Vector3f current_ned(
      curr_odom_.position[0],
      curr_odom_.position[1],
      curr_odom_.position[2]);

    float xy_step = 0.0f;
    Eigen::Vector3f target_body_frd(0.0f, 0.0f, 0.0f);

    if (control_engaged && valid_xy && err_dist >= deadband_m_) {
      const float ux = ex / err_dist;  // right ratio
      const float uy = ey / err_dist;  // forward ratio

      xy_step =
        position_step_max_m_ *
        (2.0f / static_cast<float>(M_PI)) *
        std::atan(atan_position_gain_ * err_dist);

      xy_step = clamp_range(xy_step, position_step_min_m_, position_step_max_m_);

      const float step_right = xy_step * ux;
      const float step_forward = xy_step * uy;

      // BODY/FRD: x = forward, y = right, z = down
      target_body_frd = Eigen::Vector3f(step_forward, step_right, 0.0f);
    }

    Eigen::Vector3f target_ned = current_ned + body_frd_to_ned(target_body_frd);

    // x/y position-based, z velocity-based.
    msg.position = {target_ned[0], target_ned[1], nan_};
    msg.velocity = {nan_, nan_, descent_mps};

    RCLCPP_INFO(
      this->get_logger(),
      "[POS_XY_VEL_Z] valid=%d aligned=%d hold=%d/%d dx=%.3f dy=%.3f err=%.3f tol=%.3f xy_step=%.3f vz=%.3f alt=%.3f",
      valid_xy,
      aligned,
      hold_counter_,
      align_need_,
      desired_x_,
      desired_y_,
      err_dist,
      tol_m_,
      xy_step,
      descent_mps,
      alt_m);
  } else {
    float ux = 0.0f;  // right ratio
    float uy = 0.0f;  // forward ratio
    float v_target = 0.0f;

    if (control_engaged && valid_xy && err_dist >= deadband_m_) {
      ux = ex / err_dist;
      uy = ey / err_dist;

      v_target = max_xy_ * std::tanh(tanh_gain_ * err_dist);
      v_target = clamp_range(v_target, tanh_min_xy_, max_xy_);
    }

    // vz 와 동일하게 수평 속도 "크기"만 slew 한다. 방향(ux, uy)은 제한하지
    // 않으므로 타겟을 놓치면(ux=uy=0) 출력은 즉시 0 이 된다 - 감속은 늦추지 않는다.
    if (v_target > v_cmd_) {
      v_cmd_ = std::min(v_target, v_cmd_ + xy_slew_mps_);
    } else {
      v_cmd_ = std::max(v_target, v_cmd_ - xy_slew_mps_);
    }

    const float v_close = v_cmd_;

    const float v_right = clamp_symmetric(v_close * ux, max_xy_);
    const float v_forward = clamp_symmetric(v_close * uy, max_xy_);

    Eigen::Vector3f v_body(v_forward, v_right, 0.0f);
    Eigen::Vector3f v_ned = body_frd_to_ned(v_body);

    // x/y/z velocity-based.
    msg.position = {nan_, nan_, nan_};
    msg.velocity = {v_ned[0], v_ned[1], descent_mps};

    RCLCPP_INFO(
      this->get_logger(),
      "[VEL_XYZ] eng=%d dx=%.3f dy=%.3f err=%.3f tol=%.3f v_tgt=%.3f v_close=%.3f vz=%.3f alt=%.3f",
      control_engaged,
      desired_x_,
      desired_y_,
      err_dist,
      tol_m_,
      v_target,
      v_close,
      descent_mps,
      alt_m);
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);

  if (
    valid_xy &&
    hold_counter_ >= align_need_ &&
    acc_alt_ > low_enough_ &&
    !nav_land_sent_) {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_INFO(
      this->get_logger(),
      "[LANDING] aligned & low enough (alt=%.2f m). NAV_LAND.",
      alt_m);

    nav_land_sent_ = true;
    mission_mode_ = FINISHED;
  }
}


void LandingTest::publish_vehicle_command(
  uint16_t command,
  float param1,
  float param2,
  float param3,
  float param4) {
  VehicleCommand msg{};

  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.target_system = 1;
  msg.command = command;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  vehicle_command_publisher_->publish(msg);
}


int main(int argc, char *argv[]) {
  std::cout << "Starting landing test" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<LandingTest>();
  rclcpp::spin(node);

  const int code = node->exit_code();
  rclcpp::shutdown();

  return code;
}
