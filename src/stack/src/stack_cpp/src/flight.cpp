#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int32.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "stack_cpp/mission_manager.h"

using namespace std::chrono;
using namespace std::chrono_literals;

using namespace std_msgs::msg;
using namespace px4_msgs::msg;

//TODO: GPS navigation
//      PX4 supports VehicleGlobalPosition : Fused WGS84
//      Use VehicleGlobalPosition & WGS84 waypoints
//      instead of VehicleOdometry & local NED waypoints
//      -> set_origin() logic might not be needed
//      -> TrajectorySetpoint uses NED coordinates
//      OR
//      Calculate local NED waypoints according to current WGS84
//      and given WGS84 waypoints and use current waypoint finding logic
//
//TODO: Initial report shows PRISM trajectory planning logic
//      Current code feeds discrete waypoint coordinates whereas
//      for the aircraft to follow the planned trajectory
//      continuous coordinates should be fed and thus needs change in flight logic
//      Suggestion:
//      Use ROS Service/Client before flight, similar with set_origin()
//      and save it as current forward_waypoints.
//      Publish trajectory coordinates without evaluating
//      if the aircraft has arrived at the desired coordinate.
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
          if(manager.get_node(cmd) != NodeName::FLIGHT) return;
          mission_mode = manager.get_mode(cmd);
          NodeState command_state = manager.get_command(cmd);
          if(self_state == command_state) return;
          if(command_state == NodeState::BUSY) {
            flight_mode_ = STANDBY;
            hold_counter_ = 0;
            wp_idx_ = 0;
            holding_last_wp_ = false;
              
            setWaypointOrder(mission_mode);
            hold_position_ = waypoints_.back();
          }
          self_state = command_state;
          RCLCPP_INFO(get_logger(), "Command recieved from MISSION.");
        });
        
      //main logic
      auto timer_callback = [this]() -> void {
        reportNodeStatus(self_state);
        
        if(self_state != NodeState::BUSY && self_state != NodeState::SUCCESS)
          return;
          
        if(flight_mode_ == STANDBY) {
          if(set_origin_done){
            flight_mode_ = MULTIROTOR;
            return;
          }
          this->set_origin();
        }

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
      STANDBY,
      MULTIROTOR = 3,
      FIXED_WING = 4,
      FINISHED
    };

    FlightMode flight_mode_ = STANDBY;
    
    std::vector<std::array<float,3>> forward_waypoints = {
      {0.0f, 0.0f, -10.0f},
      {100.0f, 0.0f, -10.0f},
      {200.0f, 100.0f, -10.0f},
      {200.0f, -100.0f, -10.0f},
      {100.0f, 0.0f, -10.0f},
      {0.0f, 0.0f, -10.0f}
    };
    
    std::vector<std::array<float,3>> waypoints_ = forward_waypoints;
    
    void setWaypointOrder(MissionMode mode);
    
    std::array<float,3>hold_position_ = forward_waypoints.back();
    bool holding_last_wp_ = false;

    uint64_t offboard_setpoint_counter_ {0};
    size_t wp_idx_ {0};

    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 20;

    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void transition(FlightMode mode = MULTIROTOR);

    float k = 1;
    
    void set_origin();
    float origin[3] = {0, 0, 0};
    bool set_origin_done = false;
    int origin_counter = 0;
    int origin_count_threshold = 10;
    
    MissionManager manager;
    NodeState self_state = NodeState::IDLE;
    MissionMode mission_mode = MissionMode::IDLE;
    void reportNodeStatus(NodeState state);
};


void Flight::reportNodeStatus(NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(NodeName::FLIGHT, state);
  status_publisher -> publish(msg);
}


void Flight::setWaypointOrder(MissionMode mode) {
    if (mode == MissionMode::WP_FLIGHT) waypoints_ = forward_waypoints;
    else if (mode == MissionMode::INVERSE_WP_FLIGHT) {
        holding_last_wp_ = false;
        waypoints_ = forward_waypoints;
        std::reverse(waypoints_.begin(), waypoints_.end());
    }
    else self_state = NodeState::ABORT;
}


//main logic
void Flight::publishTrajectorySetpoint() {
  TrajectorySetpoint msg {};

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  
  std::array<float,3> target_wp = holding_last_wp_? hold_position_ : waypoints_[wp_idx_];
  
  Eigen::Vector3f target(target_wp[0], target_wp[1], target_wp[2]);
  msg.position = {target_wp[0], target_wp[1], target_wp[2]};
  
  Eigen::Vector3f to_wp = target - current;
  float dist_to_wp = to_wp.norm();
  
  //normalize to_wp
  if (dist_to_wp > 1e-3f)
    to_wp /= dist_to_wp;
  switch(flight_mode_){
    case STANDBY:
      break;
      
    case MULTIROTOR:
      if(holding_last_wp_) {
        if(dist_to_wp < 3.0f) {
          hold_counter_++;
          
          if(hold_counter_ > HOLD_THRESHOLD) {
            RCLCPP_INFO(this->get_logger(), "Finished waypoint flight successfully.");
            self_state = NodeState::SUCCESS;
            flight_mode_ = FINISHED;
          }
        } else
          hold_counter_ = 0;
      }
      else { //not holding last waypoint
        if(dist_to_wp < 3.0f) {
          hold_counter_++;
          
          if(hold_counter_ > HOLD_THRESHOLD) {
            hold_counter_ = 0;
            wp_idx_++;
            
            if(wp_idx_ == 1) transition(FIXED_WING);
            
            RCLCPP_INFO(get_logger(), "[MULTIROTOR] Heading to waypoint %zu", wp_idx_);
          }
        } else
          hold_counter_ = 0;
      }
      break;
    
    case FIXED_WING:
      msg.velocity = {k*to_wp.x(), k*to_wp.y(), 0.0f};
      if (dist_to_wp < 10.0f) {
        wp_idx_++;
        if(wp_idx_ >= waypoints_.size()) {
          holding_last_wp_ = true;
          transition(MULTIROTOR);
        }
        else {
          RCLCPP_INFO(this->get_logger(), "[FIXED_WING] Heading to waypoint %ld", wp_idx_);
        }
      }
      break;
      
    case FINISHED:
      msg.position = {hold_position_[0], hold_position_[1], hold_position_[2]};
      break;
  }
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher->publish(msg);
}

void Flight::set_origin(){
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
