#include <iostream>
#include <chrono>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "stack_cpp/mission_manager.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace px4_msgs::msg;

class Mission : public rclcpp::Node {
  public:
    Mission() : Node("Mission") {
      mission_command_publisher = this->create_publisher<UInt32>("mission/command", 10);
      mission_summary_publisher = this->create_publisher<UInt32>("mission/summary", 10);

      offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
      
      
      ground_command_subscriber = this->create_subscription<UInt32>("ground/command", 10,
        [this](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;
          if(manager.get_node(cmd) == NodeName::MISSION) {
            self_state = manager.get_command(cmd);
            manager.set(NodeName::MISSION, self_state);
            RCLCPP_INFO(get_logger(), "Command recieved from GROUND.");
          }});
      
      vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        [this](const VehicleOdometry::SharedPtr msg) {
          if(!has_odom)
            RCLCPP_INFO(this->get_logger(), "Odometry recieved.");
          
	  has_odom = true;
        });
        
        
      vehicle_status_subscriber = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
        [this](const VehicleStatus::SharedPtr msg) {
          armed = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
        });
      
      //helper
      auto makeStatusCallback = [this](NodeName expected_node) {
        return [this, expected_node](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;

          if (manager.get_node(cmd) == expected_node) {
            manager.set(expected_node, manager.get_command(cmd));
          }
        };
      };
      
      state_flight_subscriber = this->create_subscription<UInt32>(
        "nodes/flight/status", 10,
        makeStatusCallback(NodeName::FLIGHT));

      state_target_subscriber = this->create_subscription<UInt32>(
        "nodes/target/status", 10,
        makeStatusCallback(NodeName::TARGET));
        
      state_gripper_subscriber = this->create_subscription<UInt32>(
        "nodes/gripper/status", 10,
        makeStatusCallback(NodeName::GRIPPER));

      state_vision_subscriber = this->create_subscription<UInt32>(
        "nodes/vision/status", 10,
        makeStatusCallback(NodeName::VISION));

      state_marker_subscriber = this->create_subscription<UInt32>(
        "nodes/marker/status", 10,
        makeStatusCallback(NodeName::MARKER));
        
      state_yolo_subscriber = this->create_subscription<UInt32>(
        "nodes/yolo/status", 10,
        makeStatusCallback(NodeName::YOLO));          

      state_logger_subscriber = this->create_subscription<UInt32>(
        "nodes/logger/status", 10,
        makeStatusCallback(NodeName::LOGGER));
            
      //main logic
      auto timer_callback = [this]() -> void {
        if(counter_ == 0) manager.clear();
        counter_ ++;
        publishMissionSummary();
        
        if(self_state == NodeState::IDLE
           || mission_mode == MissionMode::FINISHED)
          return;
          
        //mission mode abort handeled in switch()
        if(self_state == NodeState::ABORT) {
           abort();
           return;
        }
        
        if(!has_odom
           && mission_mode != MissionMode::FINISHED
           && mission_mode != MissionMode::ABORT) {           
          RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
          return;
        }
        
        publishOffboardControlMode();
        switch(mission_mode){
          case MissionMode::IDLE:
            if(!armed){
              this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
              arm();
            }
            
            if(manager.get(NodeName::LOGGER) == NodeState::IDLE) {
              publishMissionCommand(NodeName::LOGGER, NodeState::BUSY);
              RCLCPP_INFO(this->get_logger(), "Sending activation command to LOGGER.");
            }
            
            if(manager.get(NodeName::VISION) == NodeState::IDLE) {
              publishMissionCommand(NodeName::VISION, NodeState::BUSY);
              RCLCPP_INFO(this->get_logger(), "Sending activation command to VISION.");
            }
            
            all_go = (manager.get(NodeName::MISSION) == NodeState::BUSY) &&
              (manager.get(NodeName::FLIGHT) == NodeState::IDLE) &&
              (manager.get(NodeName::TARGET) == NodeState::IDLE) &&
              (manager.get(NodeName::GRIPPER) == NodeState::IDLE) &&
              (manager.get(NodeName::VISION) == NodeState::BUSY) &&
              (manager.get(NodeName::MARKER) == NodeState::IDLE) &&
              (manager.get(NodeName::YOLO) == NodeState::IDLE) &&
              (manager.get(NodeName::LOGGER) == NodeState::BUSY);
            
            if(all_go) {
              mission_mode = MissionMode::TAKEOFF;
              RCLCPP_INFO(this->get_logger(), "MissionMode: Takeoff");
            }
            break;
            
          case MissionMode::TAKEOFF:
            publishMissionCommand(NodeName::FLIGHT, NodeState::BUSY);
              
            //if(manager.get(NodeName::FLIGHT) == NodeState::SUCCESS)
            //  mission_mode = MissionMode::TRANSITION_2_FW;
                
            break;
            //takeoff
            //if done altitude mission_mode++
          case MissionMode::TRANSITION_2_FW:
            //transition to fixed wing
            //if done mission_mode++ 
            //or if rescued or mission_mode = 6
          case MissionMode::WP_FLIGHT:
            //fw flight through waypoints
            //if done mission_mode++
          case MissionMode::TRANSITION_2_MC:
            //reverse transition
            //if done mission_mode++
            //or if rescued mission_mode = 7
          case MissionMode::RESCUE:
            //find and rescue
            //if done mission_mode = 1
          case MissionMode::INVERSE_WP_FLIGHT:
            //go back to start
            //if done mission_mode = 4
          case MissionMode::DROP:
            //drop rescued personel
            //if done mission_mode++
          case MissionMode::LANDING:
            //precision landing
            //if successful mission_mode++
          case MissionMode::FINISHED:
            //disarm vehicle
            
          default:
            RCLCPP_INFO(this->get_logger(), "ABORTING MISSION");
            abort();
            break;
        }
    };
    timer = this->create_wall_timer(100ms, timer_callback);
  }
    
  private:    
    rclcpp::TimerBase::SharedPtr timer;
    std::atomic<uint64_t> timestamp;
    
    rclcpp::Publisher<UInt32>::SharedPtr mission_command_publisher;
    rclcpp::Publisher<UInt32>::SharedPtr mission_summary_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
    
    rclcpp::Subscription<UInt32>::SharedPtr ground_command_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_flight_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_target_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_gripper_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_vision_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_marker_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_yolo_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_logger_subscriber;
    
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber;
    
    MissionManager manager;
    
    MissionMode mission_mode = MissionMode::IDLE;
    NodeState self_state = NodeState::IDLE;
    
    int counter_ = 0;
    int num_of_nodes = static_cast<int>(NodeName::LOGGER);
    
    bool has_odom = false;
    bool all_go = false;
    bool aborted = false;
    bool armed = false;

    void publishMissionSummary();
    void publishMissionCommand(NodeName node, NodeState state);
    
    void publishOffboardControlMode();
    void publishVehicleCommand(uint16_t command, float param1, float param2);
    
    void arm();
    void disarm();
    
    void abort();
};

void Mission::abort() {
  if(aborted) return;
  
  mission_mode = MissionMode::ABORT;
  
  for(int i = 0; i < num_of_nodes; i++) {
    //mission node's state is directly modified by ground/command callback
    NodeName node = static_cast<NodeName>(i + 1);
    publishMissionCommand(node, NodeState::ABORT);
  }
  
  for(int i = 0; i < num_of_nodes + 1; i++) {
    NodeName node = static_cast<NodeName>(i);
    if(manager.get(node) != NodeState::ABORT){
      RCLCPP_WARN(this->get_logger(), "Some nodes might still be active.");
      RCLCPP_WARN(this->get_logger(), "Trying to kill all remaining nodes...");
      return;
    }
  }
  aborted = true;
  RCLCPP_WARN(this->get_logger(), "Mission successfully aborted.");    
}

void Mission::publishMissionSummary() {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.raw();
  mission_summary_publisher -> publish(msg);
}

void Mission::publishMissionCommand(NodeName node, NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(node, state);
  mission_command_publisher -> publish(msg);
}

void Mission::publishOffboardControlMode() {
  OffboardControlMode msg {};
  //TODO: set position to "false" if landing / target guidance is done via velocity
  //      should be "true" for waypoint flights
  msg.position = true;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher->publish(msg);
}

void Mission::publishVehicleCommand(uint16_t command, float param1, float param2) {
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

//check vehicle status before updating armed tag
void Mission::arm() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
  //armed = true;
}

void Mission::disarm() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
  //armed = false;
}

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission>());
  
  rclcpp::shutdown();
  return 0;
}
