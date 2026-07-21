#include <iostream>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
  public:
    OffboardControl() : Node("test") {
      odom_sub_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
                           [this](const VehicleOdometry::SharedPtr msg) {
                           curr_odom_ = *msg; has_odom_ = true;});

      landed_sub_ = this->create_subscription<VehicleLandDetected>("/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
                           [this](const VehicleLandDetected::SharedPtr msg) {
                           landed_ = msg->landed;});

      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

      auto timer_callback = [this]() -> void {
        if(!has_odom_){
          RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
          return;
        }

        if(!armed_ && mission_mode_ == STANDBY) {
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          this->arm();
        }
        
        if(mission_mode_ == STANDBY) {
          this->set_origin();
          if(set_origin_done) mission_mode_ = FLIGHT;
        }

        publish_offboard_control_mode();

        switch(mission_mode_) {
          case FLIGHT:
            publish_trajectory_setpoint();
            break;

          case LANDING:
            land();
            break;

          case FINISHED:
            if(landed_ && armed_) disarm();
            if(!armed_) return;
            break;
        }

        offboard_setpoint_counter_++;
      };

      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<VehicleLandDetected>::SharedPtr  landed_sub_;

    px4_msgs::msg::VehicleOdometry curr_odom_;

    enum MissionMode {
        STANDBY,
        FLIGHT,
        LANDING,
        FINISHED
    };

    MissionMode mission_mode_ = STANDBY;

    std::vector<std::array<float,3>> waypoints_ = {
      {0.0f, 0.0f, -2.0f},
      {2.0f, 0.0f, -2.0f},
      {0.0f, 0.0f, -2.0f},
      {0.0f, 2.0f, -2.0f},
      {0.0f, 0.0f, -2.0f},
      {0.0f, 0.0f, -3.0f},
    };

    uint64_t offboard_setpoint_counter_ {0};
    size_t wp_idx_ {0};

    bool has_odom_ = false;
    bool armed_ = false;
    bool landed_ = false;

    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 20;

    void arm();
    void disarm();
    void land();

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void publish_vehicle_command(int command, float value);
    
    void set_origin();
    float origin[3] = {0, 0, 0};
    bool set_origin_done = false;
    int origin_counter = 0;
    int origin_count_threshold = 10;
};

void OffboardControl::set_origin(){
  if(origin_counter < origin_count_threshold){
    origin[0] += curr_odom_.position[0];
    origin[1] += curr_odom_.position[1];
    origin[2] += curr_odom_.position[2];
    origin_counter ++;
  }
  else {    
    origin[0] /= origin_count_threshold;
    origin[1] /= origin_count_threshold;
    origin[2] /= origin_count_threshold;

    set_origin_done = true;
    RCLCPP_INFO(this->get_logger(), "Origin set to (%f, %f, %f)", origin[0], origin[1], origin[2]);
    for(int i = 0; i < waypoints_.size(); i++){
      waypoints_[i][0] += origin[0];
      waypoints_[i][1] += origin[1];
      waypoints_[i][2] += origin[2];
    }
  }  
}

void OffboardControl::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
  armed_ = true;
}

void OffboardControl::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
  armed_ = false;
}

void OffboardControl::publish_offboard_control_mode() {
  OffboardControlMode msg {};
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() {
  if (curr_odom_.timestamp == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  TrajectorySetpoint msg {};

  auto &wp = waypoints_[wp_idx_];

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  Eigen::Vector3f target(wp[0], wp[1], wp[2]);
  Eigen::Vector3f to_wp = target - current;
  float dist_to_wp = to_wp.norm();
  Eigen::Vector3f direction_v(to_wp.x()/dist_to_wp, to_wp.y()/dist_to_wp, to_wp.z()/dist_to_wp);

  if (mission_mode_ == FLIGHT && armed_) {
    msg.position = {wp[0], wp[1], wp[2]};

    if (dist_to_wp < 0.7f) {
      hold_counter_++;
      if (hold_counter_ > HOLD_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "[MULTIROTOR] Heading to waypoint %ld", wp_idx_ + 1);
        hold_counter_ = 0;
        wp_idx_++;
        if(wp_idx_ >= waypoints_.size()) {
          RCLCPP_INFO(this->get_logger(), "[MULTIROTOR] Flight mode : LANDING");
          mission_mode_ = LANDING;
        }
      }
    }
  }
  else return;

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::land() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  RCLCPP_INFO(this->get_logger(), "[Landing] Starting landing sequence");
  mission_mode_ = FINISHED;
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
  VehicleCommand msg {};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  std::cout << "Starting offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());

  rclcpp::shutdown();
  return 0;
}
