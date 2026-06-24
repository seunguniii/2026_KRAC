#include <iostream>
#include <chrono>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "stack_cpp/mission_manager.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace px4_msgs::msg;

class Master : public rclcpp::Node {
  public:
    Master() : Node("Master") {
      master_command_publisher = this->create_publisher<UInt32>("mission/command", 10);
      master_summary_publisher = this->create_publisher<UInt32>("mission/summary", 10);
      
      
      vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        [this](const VehicleOdometry::SharedPtr msg) {
	has_odom = true;});
      
      
      state_wp_navigator_subscriber = this->create_subscription<UInt32>("nodes/wp_navigator/status", 10,
        [this](const UInt32::SharedPtr msg) {
          NodeState state = static_cast<NodeState>(msg->data);
          manager.set(NodeName::WP_NAVIGATOR, state);});
          
      state_target_guide_subscriber = this->create_subscription<UInt32>("nodes/target_guide/status", 10,
        [this](const UInt32::SharedPtr msg) {
          NodeState state = static_cast<NodeState>(msg->data);
          manager.set(NodeName::TARGET_GUIDE, state);});
          
      state_gripper_subscriber = this->create_subscription<UInt32>("nodes/gripper/status", 10,
        [this](const UInt32::SharedPtr msg) {
          NodeState state = static_cast<NodeState>(msg->data);
          manager.set(NodeName::GRIPPER, state);});
          
      state_vision_subscriber = this->create_subscription<UInt32>("nodes/vision/status", 10,
        [this](const UInt32::SharedPtr msg) {
          NodeState state = static_cast<NodeState>(msg->data);
          manager.set(NodeName::VISION, state);});
          
      state_logger_subscriber = this->create_subscription<UInt32>("nodes/logger/status", 10,
        [this](const UInt32::SharedPtr msg) {
          NodeState state = static_cast<NodeState>(msg->data);
          manager.set(NodeName::LOGGER, state);});
    
    
      auto timer_callback = [this]() -> void {
        publishMissionSummary();
      
        if(!has_odom){
          RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
          return;
        }
        
        if(manager.get(NodeName::MASTER) == NodeState::IDLE && (mission_mode != MissionMode::FINISHED && mission_mode != MissionMode::ABORT)) {
          RCLCPP_INFO(this->get_logger(), "Odometry recieved. Activating master.");
          manager.set(NodeName::MASTER, NodeState::BUSY);
        }
      
        switch(mission_mode){
          case MissionMode::IDLE:
            if(manager.get(NodeName::LOGGER) == NodeState::IDLE) {
              publishMissionCommand(NodeName::LOGGER, NodeState::BUSY);
              RCLCPP_INFO(this->get_logger(), "Sending activation command to LOGGER.");
            }
            
            all_go = (manager.get(NodeName::MASTER) == NodeState::BUSY) &&
              (manager.get(NodeName::WP_NAVIGATOR) == NodeState::IDLE) &&
              (manager.get(NodeName::TARGET_GUIDE) == NodeState::IDLE) &&
              (manager.get(NodeName::GRIPPER) == NodeState::IDLE) &&
              (manager.get(NodeName::VISION) == NodeState::IDLE) &&
              (manager.get(NodeName::LOGGER) == NodeState::BUSY);
            
            if(all_go) mission_mode = MissionMode::TAKEOFF;
            break;
            
          case MissionMode::TAKEOFF:
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
            
          case MissionMode::ABORT:
          default:
            //emergency handling, failsafe
            break;
        }
    
    };
    timer = this->create_wall_timer(100ms, timer_callback);
  }
    
  private:    
    rclcpp::TimerBase::SharedPtr timer;
    std::atomic<uint64_t> timestamp;
    
    rclcpp::Publisher<UInt32>::SharedPtr master_command_publisher;
    rclcpp::Publisher<UInt32>::SharedPtr master_summary_publisher;
    
    rclcpp::Subscription<UInt32>::SharedPtr state_wp_navigator_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_target_guide_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_gripper_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_vision_subscriber;
    rclcpp::Subscription<UInt32>::SharedPtr state_logger_subscriber;
    
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;
    
    MissionManager manager;
    
    MissionMode mission_mode = MissionMode::IDLE;
    
    bool has_odom = false;
    bool all_go = false;

    void publishMissionSummary();
    void publishMissionCommand(NodeName node, NodeState state);
};

void Master::publishMissionSummary() {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.raw();
  master_summary_publisher -> publish(msg);
}

void Master::publishMissionCommand(NodeName node, NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(node, state);
  master_command_publisher -> publish(msg);
}

int main(int argc, char *argv[]) {
  std::cout << "Starting master node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Master>());
  
  rclcpp::shutdown();
  return 0;
}
