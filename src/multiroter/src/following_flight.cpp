#include <chrono>
#include <cmath>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include <Eigen/Geometry>

using namespace std::chrono_literals;

class PersonFollower : public rclcpp::Node
{
public:
  PersonFollower()
  : Node("person_follower"),
    target_visible_(false)
  {
    // ---- 파라미터 ----
    K_y_         = this->declare_parameter("K_y", 5.0);
    K_z_         = this->declare_parameter("K_z", 1.5);
    K_size_      = this->declare_parameter("K_size", 2.0);   // sqrt law면 너무 크지 않게 시작
    size_target_ = this->declare_parameter("size_target", 0.03);

    // 데드존/속도 제한 (m/s)
    dead_xy_   = this->declare_parameter("dead_xy", 0.03);
    dead_size_ = this->declare_parameter("dead_size", 0.003);

    vx_max_ = this->declare_parameter("vx_max", 1.0);
    vy_max_ = this->declare_parameter("vy_max", 0.8);
    vz_max_ = this->declare_parameter("vz_max", 0.6);

    // ---- PX4 target/source id ----
    target_system_    = this->declare_parameter("target_system", 1);
    target_component_ = this->declare_parameter("target_component", 1);
    source_system_    = this->declare_parameter("source_system", 1);
    source_component_ = this->declare_parameter("source_component", 1);

    // ---- QoS ----
    auto qos_px4 = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_px4.best_effort();
    qos_px4.durability_volatile();

    // ---- 구독자 ----
    error_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/person_tracker/error", 10,
      std::bind(&PersonFollower::errorCallback, this, std::placeholders::_1));

    visible_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/person_tracker/visible", 10,
      std::bind(&PersonFollower::visibleCallback, this, std::placeholders::_1));

    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos_px4,
      std::bind(&PersonFollower::vehicleStatusCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos_px4,
      std::bind(&PersonFollower::odomCb, this, std::placeholders::_1));

    // ---- 퍼블리셔 ----
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos_px4);

    traj_sp_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos_px4);

    cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos_px4);

    // ---- 타이머 (20 Hz) ----
    timer_ = this->create_wall_timer(50ms, std::bind(&PersonFollower::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "PersonFollower node started.");
  }

private:
  static float qnan()
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  static float clampf(float v, float lo, float hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  uint64_t nowUs()
  {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
  }

  // ---- 콜백 ----
  void errorCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    last_error_ = *msg;
    have_error_ = true;
  }

  void visibleCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    target_visible_ = msg->data;
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
    have_vehicle_status_ = true;
  }

  void odomCb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // VehicleOdometry.q: [w, x, y, z]
    q_body_to_ref_ = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    q_body_to_ref_.normalize();
    have_odom_ = true;
  }

  // ---- PX4 명령 ----
  void sendVehicleCommand(uint16_t command, float param1, float param2)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = nowUs();
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;

    cmd.target_system = static_cast<uint8_t>(target_system_);
    cmd.target_component = static_cast<uint8_t>(target_component_);
    cmd.source_system = static_cast<uint8_t>(source_system_);
    cmd.source_component = static_cast<uint8_t>(source_component_);
    cmd.from_external = true;

    cmd_pub_->publish(cmd);
  }

  void requestArmAndOffboard()
  {
    sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
    sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }

  // ---- Offboard stream ----
  void publishOffboardMode(uint64_t t_us)
  {
    px4_msgs::msg::OffboardControlMode ctrl{};
    ctrl.timestamp = t_us;
    ctrl.position = false;
    ctrl.velocity = true;
    ctrl.acceleration = false;
    ctrl.attitude = false;
    ctrl.body_rate = false;
    offboard_mode_pub_->publish(ctrl);
  }

  void publishVelocitySetpoint(uint64_t t_us, float vN, float vE, float vD)
  {
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = t_us;

    // velocity (NED)
    sp.velocity[0] = vN;
    sp.velocity[1] = vE;
    sp.velocity[2] = vD;

    // unused -> NaN
    sp.position[0] = qnan(); sp.position[1] = qnan(); sp.position[2] = qnan();
    sp.acceleration[0] = qnan(); sp.acceleration[1] = qnan(); sp.acceleration[2] = qnan();
    sp.jerk[0] = qnan(); sp.jerk[1] = qnan(); sp.jerk[2] = qnan();
    sp.yaw = qnan();
    sp.yawspeed = qnan();

    traj_sp_pub_->publish(sp);
  }

  void timerCallback()
  {
    const uint64_t t_us = nowUs();

    // 1) 항상 스트리밍
    publishOffboardMode(t_us);

    float vx_body = 0.0f, vy_body = 0.0f, vz_body = 0.0f; // body(F,R,D)로 만들고 NED로 회전할 것

    if (target_visible_ && have_error_) {
      float dx   = static_cast<float>(last_error_.x); // 오른쪽 +
      float dy   = static_cast<float>(last_error_.y); // 위쪽 +
      float size = static_cast<float>(last_error_.z); // bbox area ratio (0~1)

      // deadzone
      if (std::fabs(dx) < dead_xy_) dx = 0.0f;
      if (std::fabs(dy) < dead_xy_) dy = 0.0f;

      // ---- 전후(거리) 제어: sqrt law (면적은 거리^(-2)라 더 자연스럽게)
      const float eps = 1e-6f;
      float size_clamped = std::max(size, eps);

      // 목표보다 멀면 ratio>1 -> +vx, 가까우면 ratio<1 -> -vx
      float ratio = std::sqrt(static_cast<float>(size_target_) / size_clamped);
      float e_size = ratio - 1.0f;
      if (std::fabs(e_size) < dead_size_) e_size = 0.0f;

      vx_body = clampf(static_cast<float>(K_size_) * e_size, -static_cast<float>(vx_max_), static_cast<float>(vx_max_));

      // 좌우: dx>0(오른쪽)이면 Right로 + 가도록
      vy_body = clampf(static_cast<float>(K_y_) * dx, -static_cast<float>(vy_max_), static_cast<float>(vy_max_));

      // 상하: dy>0(위쪽)면 올라가야 함. NED는 Down(+)라서 body D로는 음수(=up)
      vz_body = clampf(static_cast<float>(-K_z_) * dy, -static_cast<float>(vz_max_), static_cast<float>(vz_max_));

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "dx=%.3f dy=%.3f size=%.4f ratio=%.3f | v_body(F,R,D)=(%.2f,%.2f,%.2f)",
        last_error_.x, last_error_.y, last_error_.z, ratio, vx_body, vy_body, vz_body);
    }

    // 2) body(FRD) -> NED 회전
    Eigen::Vector3f v_body(vx_body, vy_body, vz_body);
    Eigen::Vector3f v_ned(0,0,0);

    if (have_odom_) {
      v_ned = q_body_to_ref_ * v_body;
    } else {
      v_ned.setZero();
    }

    publishVelocitySetpoint(t_us, v_ned.x(), v_ned.y(), v_ned.z());

    // 3) 워밍업 후 arm+offboard 재요청
    if (warmup_counter_ < WARMUP_COUNT) {
      warmup_counter_++;
      return;
    }

    bool is_offboard = false;
    bool is_armed = false;
    if (have_vehicle_status_) {
      is_offboard = (vehicle_status_.nav_state ==
                     px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
      is_armed = (vehicle_status_.arming_state ==
                  px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    }

    if ((!is_offboard || !is_armed) && (t_us - last_request_us_ > 1'000'000ULL)) {
      requestArmAndOffboard();
      last_request_us_ = t_us;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Requesting ARM + OFFBOARD...");
    }
  }

private:
  // 구독자
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr error_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

  // 퍼블리셔
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

  // 타이머
  rclcpp::TimerBase::SharedPtr timer_;

  // 상태
  geometry_msgs::msg::Vector3 last_error_{};
  bool have_error_{false};
  bool target_visible_;

  px4_msgs::msg::VehicleStatus vehicle_status_{};
  bool have_vehicle_status_{false};

  Eigen::Quaternionf q_body_to_ref_{1,0,0,0};
  bool have_odom_{false};

  // 제어 파라미터
  double K_y_;
  double K_z_;
  double K_size_;
  double size_target_;

  double dead_xy_;
  double dead_size_;
  double vx_max_;
  double vy_max_;
  double vz_max_;

  // ids
  int target_system_;
  int target_component_;
  int source_system_;
  int source_component_;

  // offboard 안정화
  uint64_t warmup_counter_{0};
  static constexpr uint64_t WARMUP_COUNT = 20;
  uint64_t last_request_us_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonFollower>());
  rclcpp::shutdown();
  return 0;
}

