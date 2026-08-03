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
      
      command_subscriber = this->create_subscription<UInt32>("mission/command", 10,
        [this](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;
        if(manager.get_node(cmd) != NodeName::TARGET) return; 
          mission_mode = manager.get_mode(cmd);
          NodeState command_state = manager.get_command(cmd);
          if(self_state != command_state) {
            self_state = command_state;
            if(command_state == NodeState::IDLE){
               need_init = true;
               nav_land_sent_ = false;
               hold_counter_ = 0;
               lost_count_ = 0;
            }
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
          acc_alt_ = msg->z;     // up(+), [m]
          desired_yaw_ = msg->w; // use for RESCUE
        });
        
      yolo_subscriber = this->create_subscription<Quaternion>(
        "/nodes/yolo/target",
        10,
        [this](const Quaternion::SharedPtr msg) {
          desired_x_ = msg->x;   // right(+), [m]
          desired_y_ = msg->y;   // forward(+), [m]
          acc_alt_ = msg->z;     // up(+), [m]
          desired_yaw_ = 0; //msg->w; // use for RESCUE
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
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<UInt32>::SharedPtr status_publisher;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;

    rclcpp::Subscription<UInt32>::SharedPtr command_subscriber;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr target_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr yolo_subscriber;

  
    VehicleOdometry curr_odom_;

    bool landed_ = false;
    bool nav_land_sent_ = false;
    
    bool need_init = true;

    int preflight_setpoint_count_ = 0;
    int offboard_setpoint_counter_ = 0;

    // Target state from vision node
    float desired_x_ = 0.0f;  // right(+), [m]
    float desired_y_ = 0.0f;  // forward(+), [m]
    float desired_yaw_ = 0.0f; //North 0, CCW(+), [rad]
    float acc_alt_ = 0.0f;    // up(+), [m]

    int lost_count_ = 0;
    int hold_counter_ = 0;

    // Parameters
    int start_mode_ = 0;

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

    float low_enough_ = 0.7f; //up (+), [m]

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
    
    float init_distance_threshold = 0.3; //m
};

void Target::reportNodeStatus(NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(NodeName::TARGET, state);
  status_publisher -> publish(msg);
}

void Target::declare_parameters() {  
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


void Target::timer_callback() {
  offboard_setpoint_counter_++;
  
  if(self_state == NodeState::IDLE ||
     self_state == NodeState::SUCCESS ||
     self_state == NodeState::ABORT) return;
  land();
}


float Target::select_descent_speed(float alt_m, bool valid_xy) const {
  if (!valid_xy || hold_counter_ < align_need_)
    return 0.0f;

  if (alt_m > 4.0f)
    return descent_high_mps_;

  if (alt_m > 1.5f)
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
  
  const float alt_m = acc_alt_;
  Eigen::Vector3f target;
  Eigen::Vector3f curr_p(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  
  //TODO: parameterize initial coordinates for these mission modes
  if(need_init){
    switch(mission_mode){
      case MissionMode::RESCUE:
        target = {0.0, -20.0, -8.0};
        break;
      case MissionMode::DROP:
        target = {0.0, 20.0, -8.0};
        break;
      case MissionMode::LANDING:
        target = {0.0, 0.0, -8.0};
        break;
      default:
        self_state = NodeState::ABORT;
        break;
    }    
    Eigen::Vector3f to_sp = target - curr_p;
    float dist_to_sp = to_sp.norm();
    if(acc_alt_ < 8.0 && dist_to_sp < init_distance_threshold)
      need_init = false;
    
    msg.position = {target[0], target[1], target[2]};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher->publish(msg);
    return;
  }

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

  if (lost_count_ > lost_abort_ && self_state != NodeState::SUCCESS) {
    //TODO: suggestion; land at current position?
    RCLCPP_WARN(
      this->get_logger(),
      "[LANDING] Lost Target. Setting node state to ABORT.");
    self_state = NodeState::ABORT;
    return;
  }

  const float ex = valid_xy ? desired_x_ : 0.0f;  // right(+)
  const float ey = valid_xy ? desired_y_ : 0.0f;  // forward(+)
  const float err_dist = std::sqrt(ex * ex + ey * ey);

  const float descent_mps = select_descent_speed(alt_m, valid_xy);

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

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher->publish(msg);

  //if (
  //  valid_xy &&
  //  hold_counter_ >= align_need_ &&
  //  acc_alt_ < low_enough_ &&
  //  !nav_land_sent_) {
  if(acc_alt_ < low_enough_ && !nav_land_sent_) {
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
