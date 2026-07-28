#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
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

}  // namespace


class LandingTest : public rclcpp::Node {
public:
  LandingTest() : Node("landing") {
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

  // 미션 루프가 성공(0)과 타겟 상실 중단(2)을 구분할 수 있게 한다.
  int exit_code() const { return exit_code_; }

private:
  enum Mission {
    FLIGHT,
    LANDING,
    FINISHED,
  };

  enum LandingMode {
    POSITION_XY_VELOCITY_Z = 0,
    VELOCITY_XYZ = 1,
  };

  // start_mode_ == 1일 때 FLIGHT 단계를 둘로 쪼갠다.
  enum FlightPhase {
    CLIMB,      // 제자리에서 start_z_까지 수직 상승
    TRANSLATE,  // 그 고도를 유지한 채 start_x_/start_y_로 수평 이동
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

  Mission mission_mode_ = FLIGHT;

  // Target state from vision node
  float desired_x_ = 0.0f;  // right(+), [m]
  float desired_y_ = 0.0f;  // forward(+), [m]
  float acc_alt_ = 0.0f;

  bool has_setpoint_ = false;
  rclcpp::Time last_setpoint_time_;

  int lost_count_ = 0;
  int hold_counter_ = 0;

  // FLIGHT 2단계(CLIMB -> TRANSLATE) 상태
  FlightPhase flight_phase_ = CLIMB;
  bool climb_origin_set_ = false;
  float climb_x_ = 0.0f;
  float climb_y_ = 0.0f;
  float climb_z_ = 0.0f;    // FLIGHT 진입 시점 고도 (NED down)
  float climb_yaw_ = 0.0f;  // FLIGHT 진입 시점 yaw [rad]
  float climb_tol_m_ = 0.5f;
  int climb_hold_need_ = 10;

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

  // BODY/FRD 오프셋. 절대 NED 좌표가 아니라 FLIGHT 진입 시점의
  // 기체 위치/기수 방향 기준 상대 이동량이다.
  float start_x_ = 0.0f;  // 전방(+) [m]
  float start_y_ = 0.0f;  // 우측(+) [m]
  float start_z_ = 0.0f;  // 하강(+) [m] = -start_z_param

  int lost_abort_ = 700;
  int align_need_ = 5;
  int hold_decay_ = 2;  // hold_counter_ decay per misaligned cycle (soft reset)

  float setpoint_timeout_s_ = 0.3f;  // vision setpoint considered stale after this
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
  void read_parameters();
  void timer_callback();

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
  Eigen::Quaternionf current_attitude_quaternion() const;
  Eigen::Vector3f body_frd_to_ned(const Eigen::Vector3f &body_frd) const;
};


void LandingTest::declare_parameters() {
  // 0: x/y position setpoint + z velocity setpoint
  // 1: x/y/z velocity setpoint
  this->declare_parameter<int>("land_param", VELOCITY_XYZ);

  // 0: start landing immediately
  // 1: FRD 접근 비행 후 착륙. start_x/y/z 는 절대 NED 가 아니라
  //    FLIGHT 진입 시점의 기체 자세 기준 상대 오프셋이다.
  this->declare_parameter<int>("start_param", 0);

  this->declare_parameter<float>("start_x_param", 0.0f);  // 전방(+) [m]
  this->declare_parameter<float>("start_y_param", 0.0f);  // 우측(+) [m], 음수 = 좌측
  this->declare_parameter<float>("start_z_param", 0.0f);  // 상승(+) [m]

  // start_param == 1일 때: 먼저 수직 상승, 그 다음 수평 이동.
  this->declare_parameter<float>("climb_tol_m_", 0.5f);
  this->declare_parameter<int>("climb_hold_need_", 10);

  // 미션 시퀀스에서 다음 노드로 넘기기 위해 종료 시 프로세스를 내린다.
  this->declare_parameter<bool>("exit_when_done", false);

  this->declare_parameter<int>("lost_abort_", 700);
  this->declare_parameter<float>("max_xy_", 0.4f);
  this->declare_parameter<float>("tol_m_", 0.8f);
  this->declare_parameter<int>("align_need_", 5);
  this->declare_parameter<int>("hold_decay_", 2);

  // Vision setpoint older than this (seconds) is treated as lost. Keep it above
  // the vision publish period so normal message gaps do not flap valid/invalid.
  this->declare_parameter<float>("setpoint_timeout_s_", 0.3f);

  // Speed slew limits [m/s per cycle]. 0.05 per 0.1 s cycle = 0.5 m/s^2.
  // vz 만 제한하고 수평을 열어두면, OFFBOARD 진입이나 비전 측정 튐에서 한 사이클
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


void LandingTest::read_parameters() {
  land_mode_ = this->get_parameter("land_param").as_int();
  start_mode_ = this->get_parameter("start_param").as_int();

  start_x_ = static_cast<float>(this->get_parameter("start_x_param").as_double());
  start_y_ = static_cast<float>(this->get_parameter("start_y_param").as_double());
  start_z_ = -static_cast<float>(this->get_parameter("start_z_param").as_double());

  climb_tol_m_ = static_cast<float>(this->get_parameter("climb_tol_m_").as_double());
  climb_hold_need_ = static_cast<int>(this->get_parameter("climb_hold_need_").as_int());
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

  if (start_mode_ == 0 && mission_mode_ == FLIGHT) {
    mission_mode_ = LANDING;
  }

  publish_offboard_control_mode();

  std_msgs::msg::String mission_msg;

  switch (mission_mode_) {
    case FLIGHT:
      publish_trajectory_setpoint();
      mission_msg.data = "FLIGHT";
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

  offboard_setpoint_counter_++;
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

  TrajectorySetpoint msg{};

  Eigen::Vector3f current(
    curr_odom_.position[0],
    curr_odom_.position[1],
    curr_odom_.position[2]);

  // 처음부터 (x,y,z)를 한 번에 주면 대각선으로 급기동한다. 먼저 제자리에서
  // start_z_까지 수직 상승한 뒤에 x/y로 수평 이동한다.
  // 기준점(위치 + yaw)은 FLIGHT 에 진입한 첫 사이클에 한 번만 잡는다. 매 사이클
  // 갱신하면 기체가 회전할 때 목표까지 같이 돌아 발산한다.
  if (!climb_origin_set_) {
    climb_x_ = current[0];
    climb_y_ = current[1];
    climb_z_ = current[2];

    const float qw = curr_odom_.q[0];
    const float qx = curr_odom_.q[1];
    const float qy = curr_odom_.q[2];
    const float qz = curr_odom_.q[3];

    climb_yaw_ = std::atan2(
      2.0f * (qw * qz + qx * qy),
      1.0f - 2.0f * (qy * qy + qz * qz));

    climb_origin_set_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "[FLIGHT] CLIMB: 제자리(%.2f, %.2f)에서 %.2f m 상승 (기준 yaw=%.1f deg)",
      climb_x_, climb_y_, -start_z_,
      climb_yaw_ * 180.0f / static_cast<float>(M_PI));
  }

  // FRD 오프셋을 진입 시점 yaw 로 회전해 절대 NED 목표로 변환한다.
  const float cos_yaw = std::cos(climb_yaw_);
  const float sin_yaw = std::sin(climb_yaw_);

  const float north = start_x_ * cos_yaw - start_y_ * sin_yaw;
  const float east = start_x_ * sin_yaw + start_y_ * cos_yaw;

  const Eigen::Vector3f target =
    (flight_phase_ == CLIMB)
      ? Eigen::Vector3f(climb_x_, climb_y_, climb_z_ + start_z_)
      : Eigen::Vector3f(climb_x_ + north, climb_y_ + east, climb_z_ + start_z_);

  msg.position = {target[0], target[1], target[2]};

  // Hold current heading; leaving yaw at 0 would command a spin to North.
  msg.yaw = nan_;
  msg.yawspeed = nan_;

  if (flight_phase_ == CLIMB) {
    // 고도만 본다. x/y는 아직 건드리지 않으므로 3D 거리로 판정하면 안 된다.
    if (std::fabs(current[2] - target[2]) < climb_tol_m_) {
      hold_counter_++;

      if (hold_counter_ > climb_hold_need_) {
        hold_counter_ = 0;
        flight_phase_ = TRANSLATE;

        RCLCPP_INFO(
          this->get_logger(),
          "[FLIGHT] 상승 완료 (alt=%.2f m) -> TRANSLATE: 전방 %.2f m / 우측 %.2f m 이동",
          -current[2], start_x_, start_y_);
      }
    } else {
      hold_counter_ = 0;
    }
  } else {
    const float dist = (target - current).norm();

    if (dist < 3.0f) {
      hold_counter_++;

      if (hold_counter_ > 20) {
        hold_counter_ = 0;
        mission_mode_ = LANDING;
        RCLCPP_INFO(this->get_logger(), "[LANDING] Initiating landing sequence");
        return;
      }
    } else {
      hold_counter_ = 0;
    }
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
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

  // 고도(z)도 유효해야 한다. alt_m 이 NaN 이면 select_descent_speed() 의 비교가
  // 전부 false 로 빠져 최저 하강속도(descent_low_mps_)로 고도를 모른 채 계속
  // 내려가고, NAV_LAND 게이트(acc_alt_ > low_enough_)도 항상 false 라 착륙이
  // 끝나지 않는다. 타겟 상실과 동일하게 취급해서 하강을 0 으로 slew 시키고
  // lost_abort_ 후 POSITION 모드로 abort 되게 한다.
  const bool valid_xy =
    fresh &&
    std::isfinite(desired_x_) &&
    std::isfinite(desired_y_) &&
    std::isfinite(alt_m);

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
