#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "stack_cpp/mission_manager.h"

using namespace std::chrono_literals;

using namespace std_msgs::msg;
using namespace geometry_msgs::msg;
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
}

class Target : public rclcpp::Node {
  public:
    Target() : Node("target") {
      status_publisher = this->create_publisher<UInt32>("nodes/target/status", 10);
      
      trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
      
      mission_mode_subscriber = this->create_subscription<UInt32>("mission/mode", 10,
        [this](const UInt32::SharedPtr msg) {
          mission_mode = static_cast<MissionMode>(msg->data);
        });
      
      command_subscriber = this->create_subscription<UInt32>("mission/command", 10,
        [this](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;
          NodeState command_state = manager.get_command(cmd);
          if(manager.get_node(cmd) == NodeName::TARGET && self_state != command_state) {
            self_state = command_state;
            RCLCPP_INFO(get_logger(), "Command recieved from MISSION.");
          }
        });
      
      odometry_subscriber = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        [this](const VehicleOdometry::SharedPtr msg) {
          curr_odom_ = *msg;
        });
      target_subscriber = this->create_subscription<Quaternion>(
        "/nodes/marker/target",
        10,
        [this](const Quaternion::SharedPtr msg) {
          desired_x_ = msg->x;   // right(+), [m]
          desired_y_ = msg->y;   // forward(+), [m]
          acc_alt_ = msg->z;     // existing convention
          desired_yaw_ = msg->w; // use for RESCUE
        });

      declare_parameters();


      timer_ = this->create_wall_timer(100ms, [this]() {
        reportNodeStatus(self_state);
        
        if(self_state != NodeState::BUSY && self_state != NodeState::SUCCESS)
          return;
          
        read_parameters();
        timer_callback(); 
      });
    }

  private:
    enum LandingMode {
      POSITION_XY_VELOCITY_Z = 0,
      VELOCITY_XYZ = 1,
    };

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<UInt32>::SharedPtr status_publisher;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;

    rclcpp::Subscription<UInt32>::SharedPtr command_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr mission_mode_subscriber;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr target_subscriber;

  
    VehicleOdometry curr_odom_;

    bool landed_ = false;
    bool nav_land_sent_ = false;

    int preflight_setpoint_count_ = 0;
    int offboard_setpoint_counter_ = 0;

    // Target state from vision node
    float desired_x_ = 0.0f;  // right(+), [m]
    float desired_y_ = 0.0f;  // forward(+), [m]
    float desired_yaw_ = 0.0f; //North 0, CCW(+), [rad]
    float acc_alt_ = 0.0f;

    int lost_count_ = 0;
    int hold_counter_ = 0;

    // Parameters
    int start_mode_ = 0;
    int land_mode_ = VELOCITY_XYZ;

    int lost_abort_ = 700;
    int align_need_ = 5;

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
    
    MissionManager manager;
    NodeState self_state = NodeState::IDLE;
    MissionMode mission_mode = MissionMode::IDLE;
    void reportNodeStatus(NodeState state);
};

void Target::reportNodeStatus(NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(NodeName::TARGET, state);
  status_publisher -> publish(msg);
}

void Target::declare_parameters() {
  // 0: x/y position setpoint + z velocity setpoint
  // 1: x/y/z velocity setpoint
  this->declare_parameter<int>("land_param", VELOCITY_XYZ);
  
  this->declare_parameter<int>("lost_abort_", 700);
  this->declare_parameter<float>("max_xy_", 0.4f);
  this->declare_parameter<float>("tol_m_", 0.8f);
  this->declare_parameter<int>("align_need_", 5);

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


void Target::read_parameters() {
  land_mode_ = this->get_parameter("land_param").as_int();
  
  lost_abort_ = this->get_parameter("lost_abort_").as_int();
  align_need_ = this->get_parameter("align_need_").as_int();

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


//TODO: add rescue/drop logics
//
//suggestion: planar guidance uses same logic
//            altitude control uses different logic
//            for an overall shorter code & avoids duplication.
//
//i.e.
//
// float target_planar_state[3];
// float target_altitude;
// float target_yaw;
// target_planar_coordinate = planar_guidance(weights);
// switch(mission_mode){
//   case LANDING:
//     target_altitude = land_altitude_control(); break;
//   case RESCUE:
//      target_altitude = rescue_altitude_control(); break;
//   case DROP:
//      target_altitude = drop_altitude_control(); break;
// }
// msg.x = target_planar_state[0];
// msg.y = target_planar_state[1];
// msg.z = target_altitude;
// msg.yaw = target_planar_state[2]; <- nan for other modes, valid value for RESCUE
// msg.timestamp = ...;
// trajectory_setpoint_publisher -> publish(msg);
//
//another suggestion: build different classes for different mission modes
//
//i.e.
// mode = control_mode(mission_mode);
// mode.planar_coordinate();
// mode.altitude();
// mode.publish_trajectory_stepoint();

void Target::timer_callback() {
  switch(mission_mode){
    case MissionMode::LANDING:
      RCLCPP_WARN(this->get_logger(),"landing");
      land();
      break;
      
    case MissionMode::RESCUE:
    case MissionMode::DROP:
    default:
      self_state = NodeState::SUCCESS;
      break;
  }
  offboard_setpoint_counter_++;
}


float Target::select_descent_speed(float alt_m, bool valid_xy) const {
  if (!valid_xy || hold_counter_ < align_need_)
    return 0.0f;

  if (alt_m > 2.0f)
    return descent_high_mps_;

  if (alt_m > 0.8f)
    return descent_mid_mps_;

  return descent_low_mps_;
}


Eigen::Quaternionf Target::current_attitude_quaternion() const {
  Eigen::Quaternionf q(
    curr_odom_.q[0],
    curr_odom_.q[1],
    curr_odom_.q[2],
    curr_odom_.q[3]);

  q.normalize();
  return q;
}


Eigen::Vector3f Target::body_frd_to_ned(const Eigen::Vector3f &body_frd) const {
  const Eigen::Quaternionf q = current_attitude_quaternion();

  if (use_q_inverse_)
    return q.conjugate() * body_frd;

  return q * body_frd;
}


void Target::land() {
  TrajectorySetpoint msg;

  const float alt_m = -acc_alt_;

  const bool valid_xy =
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
    hold_counter_ = aligned ? hold_counter_ + 1 : 0;
  }

  if (lost_count_ > lost_abort_) {
    //TODO: suggestion; land at current position?
    RCLCPP_WARN(
      this->get_logger(),
      "[LANDING] target lost too long -> switch PX4 to POSITION mode");

    publish_vehicle_command(
      VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f
    );

    self_state = NodeState::SUCCESS;
    return;
  }

  const float ex = valid_xy ? desired_x_ : 0.0f;  // right(+)
  const float ey = valid_xy ? desired_y_ : 0.0f;  // forward(+)
  const float err_dist = std::sqrt(ex * ex + ey * ey);

  const float descent_mps = select_descent_speed(alt_m, valid_xy);

  if (land_mode_ == POSITION_XY_VELOCITY_Z) {
    Eigen::Vector3f current_ned(
      curr_odom_.position[0],
      curr_odom_.position[1],
      curr_odom_.position[2]);

    float xy_step = 0.0f;
    Eigen::Vector3f target_body_frd(0.0f, 0.0f, 0.0f);

    if (valid_xy && err_dist >= deadband_m_) {
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
  } else {
    float v_forward = 0.0f;
    float v_right = 0.0f;
    float v_close = 0.0f;

    if (valid_xy && err_dist >= deadband_m_) {
      const float ux = ex / err_dist;  // right ratio
      const float uy = ey / err_dist;  // forward ratio

      v_close = max_xy_ * std::tanh(tanh_gain_ * err_dist);
      v_close = clamp_range(v_close, tanh_min_xy_, max_xy_);

      v_right = clamp_symmetric(v_close * ux, max_xy_);
      v_forward = clamp_symmetric(v_close * uy, max_xy_);
    }

    Eigen::Vector3f v_body(v_forward, v_right, 0.0f);
    Eigen::Vector3f v_ned = body_frd_to_ned(v_body);

    // x/y/z velocity-based.
    msg.position = {nan_, nan_, nan_};
    msg.velocity = {v_ned[0], v_ned[1], descent_mps};
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher->publish(msg);

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
    self_state = NodeState::SUCCESS;
  }
}


void Target::publish_vehicle_command(
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

  vehicle_command_publisher->publish(msg);
}


int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Target>());
  rclcpp::shutdown();

  return 0;
}
