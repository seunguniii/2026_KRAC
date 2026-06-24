#include <rclcpp/rclcpp.hpp>

#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"

#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <fstream>
#include <iomanip>
#include <cmath>

#include "stack_cpp/mission_manager.h"

using namespace std::chrono_literals;

using namespace std_msgs::msg;
using namespace px4_msgs::msg;

class Logger : public rclcpp::Node
{
  public:
    Logger() : Node("logger")
    {
      status_publisher = this->create_publisher<UInt32>("nodes/logger/status", 10);
      
      command_subscriber = this->create_subscription<UInt32>("mission/command", 10,
        [this](const UInt32::SharedPtr msg) {
          uint32_t cmd = msg->data;
          if(manager.get_node(cmd) == NodeName::LOGGER) {
            self_state = manager.get_command(cmd);
            RCLCPP_INFO(get_logger(), "Command recieved from Master.");
          }
        });
      
      file_.open("/home/sujin/flight_log.csv");

      if (!file_.is_open()) {
          RCLCPP_FATAL(get_logger(), "Cannot open file.");
          rclcpp::shutdown();
          return;
      }

      file_
      << "flight_mode,"
      << "waypoint,"
      << "gps_time_sec,"
      << "lat_deg,lon_deg,alt_m,"
      << "ax,ay,az,"
      << "roll_deg,pitch_deg,yaw_deg\n";

      auto qos = rclcpp::QoS(20).best_effort();

      flight_mode_sub_ = this->create_subscription<VehicleControlMode>("fmu/out/vehicle_control_mode", rclcpp::SensorDataQoS(),
        [this](const VehicleControlMode::SharedPtr msg) {
        flight_mode_ = msg->flag_control_offboard_enabled;});

      target_wp_sub_ = this->create_subscription<Int32>("nodes/wp_navigator/target_wp", 10,
        [this](const Int32::SharedPtr msg) {
        target_wp_ = msg->data;});

      gps_sub_ = create_subscription<VehicleGlobalPosition>(
          "/fmu/out/vehicle_global_position", qos,
          std::bind(&Logger::gpsCb, this, std::placeholders::_1));

      acc_sub_ = create_subscription<SensorCombined>(
          "/fmu/out/sensor_combined", qos,
          std::bind(&Logger::imuCb, this, std::placeholders::_1));

      att_sub_ = create_subscription<VehicleAttitude>(
          "/fmu/out/vehicle_attitude", qos,
          std::bind(&Logger::attCb, this, std::placeholders::_1));

      timer_ = create_wall_timer(
          100ms,   // 10 Hz
          std::bind(&Logger::writeRow, this));
    }

    ~Logger()
    {
        file_.flush();
        file_.close();
    }

  private:    
    void gpsCb(px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
    {
      gps_time_ = msg->timestamp;

      lat_ = msg->lat;
      lon_ = msg->lon;
      alt_ = msg->alt;
    }

    void imuCb(px4_msgs::msg::SensorCombined::SharedPtr msg)
    {
      ax_ = msg->accelerometer_m_s2[0];
      ay_ = msg->accelerometer_m_s2[1];
      az_ = msg->accelerometer_m_s2[2];
    }

    void attCb(px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
      double w = msg->q[0];
      double x = msg->q[1];
      double y = msg->q[2];
      double z = msg->q[3];

      roll_ = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
      pitch_ = asin(2*(w*y - z*x));
      yaw_ = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));

      roll_ *= 180.0/M_PI;
      pitch_ *= 180.0/M_PI;
      yaw_ *= 180.0/M_PI;
    }

    void writeRow()
    {
      reportNodeStatus(self_state);
      
      if (self_state != NodeState::BUSY) return;
      
      file_ << std::fixed << std::setprecision(8)
            << flight_mode_ << ","
            << target_wp_ << ","
            << gps_time_ << ","
            << lat_ << ","
            << lon_ << ","
            << alt_ << ","
            << ax_ << ","
            << ay_ << ","
            << az_ << ","
            << roll_ << ","
            << pitch_ << ","
            << yaw_
            << "\n";

      rows_++;

      if (rows_ % 50 == 0)
          file_.flush();
    }
    
    void reportNodeStatus(NodeState state) {
      std_msgs::msg::UInt32 msg;
      msg.data = manager.pack(NodeName::LOGGER, state);
      status_publisher -> publish(msg);
    }

  private:
    std::ofstream file_;
    size_t rows_{0};

    rclcpp::Publisher<UInt32>::SharedPtr status_publisher;

    rclcpp::Subscription<UInt32>::SharedPtr command_subscriber;

    rclcpp::Subscription<VehicleControlMode>::SharedPtr flight_mode_sub_;
    rclcpp::Subscription<Int32>::SharedPtr target_wp_sub_;

    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr gps_sub_;
    rclcpp::Subscription<SensorCombined>::SharedPtr acc_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr att_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool flight_mode_ = 0;
    int target_wp_ = 0;

    uint64_t gps_time_{0};

    double lat_{0}, lon_{0}, alt_{0};
    double ax_{0}, ay_{0}, az_{0};
    double roll_{0}, pitch_{0}, yaw_{0};
    
    MissionManager manager;
    NodeState self_state = NodeState::IDLE;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Logger>());
    rclcpp::shutdown();
    return 0;
}
