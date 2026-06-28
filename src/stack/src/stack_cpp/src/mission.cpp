#include <iostream>
#include <chrono>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
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
      //node command & state summary
      mission_command_publisher = this->create_publisher<UInt32>("mission/command", 10);
      mission_summary_publisher = this->create_publisher<UInt32>("mission/summary", 10);
      
      //mission mode
      mission_mode_publisher = this->create_publisher<UInt32>("mission/mode", 10);

      offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
      
      
      ground_command_subscriber = this->create_subscription<UInt32>("ground/command", 10,
        [this](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;
          if(manager.get_node(cmd) == NodeName::MISSION) {
            self_state = manager.get_command(cmd);
            manager.set(NodeName::MISSION, self_state);
            RCLCPP_INFO(get_logger(), "Command recieved from GROUND.");
          }
        });
      
      vehicle_land_detected_subscriber = this->create_subscription<VehicleLandDetected>("/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
        [this](const VehicleLandDetected::SharedPtr msg) {
          landed = msg->landed;
        });
      
      vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        [this](const VehicleOdometry::SharedPtr msg) {
          if(!has_odom)
            RCLCPP_INFO(this->get_logger(), "Odometry recieved.");
          
	  has_odom = true;
        });
        
      //TODO: topic might not exist for different firmware versions
      vehicle_status_subscriber = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
        [this](const VehicleStatus::SharedPtr msg) {
          armed = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
        });
      
      //helper
      auto makeStatusCallback = [this](NodeName expected_node) {
        return [this, expected_node](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;

          if (manager.get_node(cmd) == expected_node) {
            auto state = manager.get_command(cmd);
            manager.set(expected_node, state);
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
        publishMissionMode();
        
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
        //main logic
        switch(mission_mode){
          case MissionMode::IDLE:
            if(!armed){
              this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
              arm();
              break;
            }
            
            if(manager.get(NodeName::LOGGER) == NodeState::IDLE) {
              RCLCPP_INFO(this->get_logger(), "Sending activation command to LOGGER.");
              publishMissionCommand(NodeName::LOGGER, NodeState::BUSY);
            }
            
            if(manager.get(NodeName::VISION) == NodeState::IDLE) {
              RCLCPP_INFO(this->get_logger(), "Sending activation command to VISION.");
              publishMissionCommand(NodeName::VISION, NodeState::BUSY);
            }
            
            if(manager.get(NodeName::FLIGHT) == NodeState::BUSY) {
              RCLCPP_INFO(this->get_logger(), "All desired nodes active.");
              RCLCPP_INFO(this->get_logger(), "MissionMode: Waypoint Flight");
              mission_mode = MissionMode::WP_FLIGHT;
              break;
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
              RCLCPP_WARN(this->get_logger(), "All green. Proceeding mission.");
              
              RCLCPP_INFO(this->get_logger(), "Sending activation command to FLIGHT.");
              publishMissionCommand(NodeName::FLIGHT, NodeState::BUSY);
            }
            break;
          
          //TODO:SUCCESS tags are for test/debug.
          //     rid SUCCESS tags once node is fully implemented
          case MissionMode::WP_FLIGHT:
            if(((manager.get(NodeName::TARGET) == NodeState::BUSY) || (manager.get(NodeName::TARGET) == NodeState::SUCCESS))
                && ((manager.get(NodeName::GRIPPER) == NodeState::BUSY) || (manager.get(NodeName::GRIPPER) == NodeState::SUCCESS))
                && (manager.get(NodeName::YOLO) == NodeState::BUSY))
            {
              RCLCPP_INFO(this->get_logger(), "All desired nodes active.");
              mission_mode = MissionMode::RESCUE;
              RCLCPP_INFO(this->get_logger(), "MissionMode: Rescue");
              break;
            }
            
            if(manager.get(NodeName::FLIGHT) == NodeState::SUCCESS) {
              RCLCPP_INFO(this->get_logger(), "Command FLIGHT to IDLE.");
              publishMissionCommand(NodeName::FLIGHT, NodeState::IDLE);
              
              RCLCPP_INFO(this->get_logger(), "Sending activation command to TARGET.");
              publishMissionCommand(NodeName::TARGET, NodeState::BUSY);
              RCLCPP_INFO(this->get_logger(), "Sending activation command to GRIPPER.");
              publishMissionCommand(NodeName::GRIPPER, NodeState::BUSY);
              RCLCPP_INFO(this->get_logger(), "Sending activation command to YOLO.");
              publishMissionCommand(NodeName::YOLO, NodeState::BUSY);
              
              //TODO: find appropriate system/component id.
              publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE, 1, 1);
              publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW, -90.0, 0.0);
            }
            break;
          
          
          case MissionMode::RESCUE:
            if(manager.get(NodeName::FLIGHT) == NodeState::BUSY)
            {
              RCLCPP_INFO(this->get_logger(), "All desired nodes active.");
              mission_mode = MissionMode::INVERSE_WP_FLIGHT;
              RCLCPP_INFO(this->get_logger(), "MissionMode: Inverse Waypoint Flight.");
              //TODO: set gimbal pitch according to desired view
              publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0.0, 0.0);
              break;
            }
            if(manager.get(NodeName::TARGET) == NodeState::SUCCESS &&
               manager.get(NodeName::GRIPPER) == NodeState::SUCCESS)
            {
              RCLCPP_INFO(this->get_logger(), "Command TARGET to IDLE.");
              publishMissionCommand(NodeName::TARGET, NodeState::IDLE);
              RCLCPP_INFO(this->get_logger(), "Command GRIPPER to IDLE.");
              publishMissionCommand(NodeName::GRIPPER, NodeState::IDLE);
              RCLCPP_INFO(this->get_logger(), "Command YOLO to IDLE.");
              publishMissionCommand(NodeName::YOLO, NodeState::IDLE);
          
              RCLCPP_INFO(this->get_logger(), "Sending activation command to FLIGHT.");
              publishMissionCommand(NodeName::FLIGHT, NodeState::BUSY);
            }
            break;
          
          //TODO:SUCCESS tags are for test/debug.
          //     rid SUCCESS tags once node is fully implemented
          case MissionMode::INVERSE_WP_FLIGHT:
            if(((manager.get(NodeName::TARGET) == NodeState::BUSY) || (manager.get(NodeName::TARGET) == NodeState::SUCCESS))
                && ((manager.get(NodeName::GRIPPER) == NodeState::BUSY) || (manager.get(NodeName::GRIPPER) == NodeState::SUCCESS))
                && (manager.get(NodeName::YOLO) == NodeState::BUSY))
            {
              RCLCPP_INFO(this->get_logger(), "All desired nodes active.");
              mission_mode = MissionMode::DROP;
              RCLCPP_INFO(this->get_logger(), "MissionMode: Drop");
              break;
            }
            if(manager.get(NodeName::FLIGHT) == NodeState::SUCCESS) {
              RCLCPP_INFO(this->get_logger(), "Command FLIGHT to IDLE.");
              publishMissionCommand(NodeName::FLIGHT, NodeState::IDLE);
              
              RCLCPP_INFO(this->get_logger(), "Sending activation command to TARGET.");
              publishMissionCommand(NodeName::TARGET, NodeState::BUSY);
              RCLCPP_INFO(this->get_logger(), "Sending activation command to GRIPPER.");
              publishMissionCommand(NodeName::GRIPPER, NodeState::BUSY);
              RCLCPP_INFO(this->get_logger(), "Sending activation command to YOLO.");
              publishMissionCommand(NodeName::YOLO, NodeState::BUSY);
              
              //TODO: use appropriate id for system/component for actual aircraft
              //primary control: 1/1, offboard node
              publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE, 1, 1);
              publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW, -90.0, 0.0);
            }
            break;
          
          //TODO: SUCCESS tags are for test/debug
          //     rid SUCCESS tags once node is fully implemented
          case MissionMode::DROP:            
            if(((manager.get(NodeName::TARGET) == NodeState::BUSY) || (manager.get(NodeName::TARGET) == NodeState::SUCCESS))
                && (manager.get(NodeName::MARKER) == NodeState::BUSY))
            {
              RCLCPP_INFO(this->get_logger(), "All desired nodes active.");
              mission_mode = MissionMode::LANDING;
              RCLCPP_INFO(this->get_logger(), "MissionMode: Landing");
              //TODO: set gimbal pitch according to desired view
              publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0.0, 0.0);
              break;
            }
            if(manager.get(NodeName::TARGET) == NodeState::SUCCESS &&
               manager.get(NodeName::GRIPPER) == NodeState::SUCCESS)
            {
              RCLCPP_INFO(this->get_logger(), "Command TARGET to IDLE.");
              publishMissionCommand(NodeName::TARGET, NodeState::IDLE);
              RCLCPP_INFO(this->get_logger(), "Command GRIPPER to IDLE.");
              publishMissionCommand(NodeName::GRIPPER, NodeState::IDLE);
              RCLCPP_INFO(this->get_logger(), "Command YOLO to IDLE.");
              publishMissionCommand(NodeName::YOLO, NodeState::IDLE);
          
              RCLCPP_INFO(this->get_logger(), "Sending activation command to FLIGHT.");
              publishMissionCommand(NodeName::FLIGHT, NodeState::BUSY);
            }
            break;
          
          case MissionMode::LANDING:
            if(manager.get(NodeName::TARGET) == NodeState::SUCCESS) {
              RCLCPP_INFO(this->get_logger(), "Command MARKER to IDLE.");
              publishMissionCommand(NodeName::MARKER, NodeState::IDLE);
              RCLCPP_INFO(this->get_logger(), "Command TARGET to IDLE.");
              publishMissionCommand(NodeName::TARGET, NodeState::IDLE);
              mission_mode = MissionMode::FINISHED;
              RCLCPP_INFO(this->get_logger(), "MissionMode: Finished");
              
            }
            break;

          case MissionMode::FINISHED:
            if(landed){
              RCLCPP_INFO(this->get_logger(), "Disarming vehicle.");
              disarm();
              publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f);
              mission_mode = MissionMode::IDLE;
            }
            idle();
            break;

          case MissionMode::ABORT:
          default:
            RCLCPP_ERROR(this->get_logger(), "ABORTING MISSION");
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
    rclcpp::Publisher<UInt32>::SharedPtr mission_mode_publisher;
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
    rclcpp::Subscription<VehicleLandDetected>::SharedPtr vehicle_land_detected_subscriber;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber;
    
    MissionManager manager;
    
    MissionMode mission_mode = MissionMode::IDLE;
    NodeState self_state = NodeState::IDLE;
    
    int counter_ = 0;
    int num_of_nodes = static_cast<int>(NodeName::LOGGER);
    
    bool has_odom = false;
    bool all_go = false;

    void publishMissionSummary();
    void publishMissionCommand(NodeName node, NodeState state);
    void publishMissionMode();
    
    void publishOffboardControlMode();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0);
    
    bool armed = false;
    bool landed = false;
    void arm();
    void disarm();
    
    bool aborted = false;
    void abort();

    void idle();
    
    bool needVelocityControl();
    
    float nan = std::numeric_limits<float>::quiet_NaN();
};

void Mission::idle() {
  for(int i = 0; i < num_of_nodes; i++) {
    //mission node's state is directly modified by ground/command callback
    NodeName node = static_cast<NodeName>(i + 1);
    publishMissionCommand(node, NodeState::IDLE);
  }
  
  for(int i = 0; i < num_of_nodes + 1; i++) {
    NodeName node = static_cast<NodeName>(i);
    if(manager.get(node) != NodeState::IDLE){
      RCLCPP_WARN(this->get_logger(), "Some nodes might still be active.");
      RCLCPP_WARN(this->get_logger(), "Trying to kill all remaining nodes...");
      return;
    }
  }
  manager.set(NodeName::MISSION, NodeState::IDLE);
  RCLCPP_WARN(this->get_logger(), "All nodes set to IDLE.");
}

void Mission::abort() {
  if(aborted) return;
  
  mission_mode = MissionMode::ABORT;
  
  for(int i = 0; i < num_of_nodes; i++) {
    //mission node's state is directly modified by ground/command callback
    NodeName node = static_cast<NodeName>(i + 1);
    
    //leave video on
    if(node != NodeName::VISION)
      publishMissionCommand(node, NodeState::ABORT);
  }
  
  for(int i = 0; i < num_of_nodes + 1; i++) {
    NodeName node = static_cast<NodeName>(i);
    if(node != NodeName::VISION) {
      if(manager.get(node) != NodeState::ABORT){
        RCLCPP_WARN(this->get_logger(), "Some nodes might still be active.");
        RCLCPP_WARN(this->get_logger(), "Trying to kill all remaining nodes...");
        return;
      }
    }
  }
  //if aborted set as position mode
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f);

  aborted = true;
  RCLCPP_WARN(this->get_logger(), "Mission successfully aborted.");    
}

void Mission::publishMissionMode() {
  std_msgs::msg::UInt32 msg;
  msg.data = static_cast<int>(mission_mode);
  mission_mode_publisher -> publish(msg);
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

bool Mission::needVelocityControl() {
  return (mission_mode == MissionMode::LANDING
          || mission_mode == MissionMode::RESCUE
          || mission_mode == MissionMode::DROP);
}

void Mission::publishOffboardControlMode() {
  OffboardControlMode msg {};
  //TODO: set position to "false" if landing / target guidance is done via velocity
  //      should be "true" for waypoint flights
  msg.position = needVelocityControl()? false:true;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher->publish(msg);
}

void Mission::publishVehicleCommand(uint16_t command, float param1, float param2, float param3, float param4) {
  VehicleCommand msg {};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher->publish(msg);
}

//check vehicle status before updating armed tag
void Mission::arm() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void Mission::disarm() {
  publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission>());
  
  rclcpp::shutdown();
  return 0;
}
