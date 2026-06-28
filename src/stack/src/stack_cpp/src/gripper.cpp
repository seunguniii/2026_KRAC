#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int32.hpp"

#include "stack_cpp/mission_manager.h"

using namespace std::chrono;
using namespace std::chrono_literals;

using namespace std_msgs::msg;

class Gripper : public rclcpp::Node {
  public:
    Gripper() : Node("Gripper") {
      status_publisher = this->create_publisher<UInt32>("nodes/gripper/status", 10);


      command_subscriber = this->create_subscription<UInt32>("mission/command", 10,
        [this](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;
          NodeState command_state = manager.get_command(cmd);
          if(manager.get_node(cmd) == NodeName::GRIPPER && self_state != command_state) {
            self_state = command_state;
            RCLCPP_INFO(get_logger(), "Command recieved from MISSION.");
          }
        });


      //main logic
      auto timer_callback = [this]() -> void {
        reportNodeStatus(self_state);

        //TODO: set when the node should run
        if(self_state != NodeState::BUSY)
          return;

        //TODO: main function
        gripper();
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;
    
    //for mission coordination from mission node
    rclcpp::Publisher<UInt32>::SharedPtr status_publisher;
    rclcpp::Subscription<UInt32>::SharedPtr command_subscriber;
    void reportNodeStatus(NodeState state);
    MissionManager manager;
    NodeState self_state = NodeState::IDLE;
    
    void gripper();
};


void Gripper::reportNodeStatus(NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(NodeName::GRIPPER, state);
  status_publisher -> publish(msg);
}


//TODO: main function
void Gripper::gripper() {
  self_state = NodeState::SUCCESS;
  return;
}


int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gripper>());

  rclcpp::shutdown();
  return 0;
}
