#include <iostream>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "stack_cpp/mission_manager.h"

using namespace std::chrono;
using namespace std::chrono_literals;

using namespace std_msgs::msg;
using namespace px4_msgs::msg;

class Flight : public rclcpp::Node {
  public:
    Flight() : Node("Flight") {
      status_publisher = this->create_publisher<UInt32>("nodes/flight/status", 10);
      
      trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

      
      vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        [this](const VehicleOdometry::SharedPtr msg) {
        curr_odom_ = *msg;});

      command_subscriber = this->create_subscription<UInt32>("mission/command", 10,
        [this](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;
          NodeState command_state = manager.get_command(cmd);
          if(manager.get_node(cmd) == NodeName::FLIGHT && self_state != command_state) {
            self_state = command_state;
            RCLCPP_INFO(get_logger(), "Command recieved from MISSION.");
          }
        });


      auto timer_callback = [this]() -> void {
        reportNodeStatus(self_state);
        
        if(self_state != NodeState::BUSY)
          return;

        publishTrajectorySetpoint();

        offboard_setpoint_counter_++;
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;
    
    rclcpp::Publisher<UInt32>::SharedPtr status_publisher;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;

    rclcpp::Subscription<UInt32>::SharedPtr command_subscriber;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;

    VehicleOdometry curr_odom_;

    enum FlightMode {
        MULTIROTOR = 3,
        FIXED_WING = 4,
        FINISHED
    };

    FlightMode flight_mode_ = MULTIROTOR;

    std::vector<std::array<float,3>> waypoints_ = {
      {0.0f, 0.0f, -25.0f},
      {100.0f, 0.0f, -25.0f},
      {200.0f, 100.0f, -25.0f},
      {200.0f, -100.0f, -25.0f},
      {100.0f, 0.0f, -25.0f},
      {0.0f, 0.0f, -25.0f},
      {0.0f, 0.0f, 0.0f}
    };

    uint64_t offboard_setpoint_counter_ {0};
    size_t wp_idx_ {0};

    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 20;

    void reportNodeStatus(NodeState state);

    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void transition(FlightMode mode = MULTIROTOR);

    float k = 1;
    
    
    MissionManager manager;
    NodeState self_state = NodeState::IDLE;
};


void Flight::reportNodeStatus(NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(NodeName::FLIGHT, state);
  status_publisher -> publish(msg);
}


//main logic
void Flight::publishTrajectorySetpoint() {
  TrajectorySetpoint msg {};

  auto &wp = waypoints_[wp_idx_];

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  Eigen::Vector3f target(wp[0], wp[1], wp[2]);
  Eigen::Vector3f to_wp = target - current;
  float dist_to_wp = to_wp.norm();
  Eigen::Vector3f direction_v(to_wp.x()/dist_to_wp, to_wp.y()/dist_to_wp, to_wp.z()/dist_to_wp);

  if (flight_mode_ == MULTIROTOR) {
    msg.position = {wp[0], wp[1], wp[2]};

    if (dist_to_wp < 3.0f) {
      hold_counter_++;
      if (hold_counter_ > HOLD_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "[MULTIROTOR] Heading to waypoint %ld", wp_idx_ + 1);
        hold_counter_ = 0;
        wp_idx_++;
        if(wp_idx_ == 1) transition(FIXED_WING);
        if(wp_idx_ >= waypoints_.size()) {
          RCLCPP_INFO(this->get_logger(), "Finished waypoint flight successfully.");
          self_state = NodeState::SUCCESS;
        } 
      }
    }
  }

  else if (flight_mode_ == FIXED_WING) {
    msg.position = {wp[0], wp[1], wp[2]};
    msg.velocity = {k*direction_v.x(), k*direction_v.y(), 0};

    if (dist_to_wp < 10.0f) {
      wp_idx_++;
      RCLCPP_INFO(this->get_logger(), "[FIXED_WING] Heading to waypoint %ld", wp_idx_);
    }

    if (wp_idx_ + 2> waypoints_.size()) transition(MULTIROTOR);
  }

  else if (flight_mode_ == FINISHED) return;

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher->publish(msg);
}


void Flight::publishVehicleCommand(uint16_t command, float param1, float param2) {
  VehicleCommand msg {};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher->publish(msg);
}


void Flight::transition(FlightMode mode) {
  if (mode == flight_mode_) {
    RCLCPP_INFO(this->get_logger(), "[TRANSITION] Already in desired flight mode, no command sent.");
    return;
  }

  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, static_cast<float>(mode));
  std::string mode_str = (mode == FIXED_WING) ? "Fixed-Wing" : "Multicopter";
  RCLCPP_INFO(this->get_logger(), "[TRANSITION] Sent VTOL transition command: %s", mode_str.c_str());
  flight_mode_ = mode;
}


int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Flight>());

  rclcpp::shutdown();
  return 0;
}
