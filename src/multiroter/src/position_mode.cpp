#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class LandingTest : public rclcpp::Node {
  public:
    LandingTest() : Node("landing") {
      odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        curr_odom_ = *msg;
        has_odom_ = true;
      });

      landed_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        landed_ = msg->landed;
      });

      desired_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/landing/coordinates", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        desired_x_ = msg->point.x;
        desired_y_ = msg->point.y;
        acc_alt_ = -msg->point.z;
      });

      //0: position based landing  1: velocity based landing
      this->declare_parameter<int>("land_param", 0);
      //0: start from manual setpoint  1: start from setpoint by param  2: land after wp flight
      this->declare_parameter<int>("start_param", 1);

      this->declare_parameter<float>("descent_param", 0.7f);
      this->declare_parameter<float>("start_x_param", 3.0f);
      this->declare_parameter<float>("start_y_param", 4.0f);
      this->declare_parameter<float>("start_z_param", 10.0f);

      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
      mission_mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/mission_mode", 10);

      auto timer_callback = [this]() -> void {
        if(!has_odom_) {
          RCLCPP_WARN(this->get_logger(), "Waiting for...");
          return;
        }

        if(!armed_ && mission_mode_ == STANDBY) {
          land_mode_ = this->get_parameter("land_param").as_int();
          start_mode_ = this->get_parameter("start_param").as_int();

          if(start_mode_ == 0) mission_mode_ = LANDING;
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          this->arm();
        }
        
        if(mission_mode_ == STANDBY) {
          this->set_origin();
          if(set_origin_done) mission_mode_ = FLIGHT;
        }

        descent_step_ = (float)this->get_parameter("descent_param").as_double();
        start_x = (float)this->get_parameter("start_x_param").as_double();
        start_y = (float)this->get_parameter("start_y_param").as_double();
        start_z = - (float)this->get_parameter("start_z_param").as_double();

        publish_offboard_control_mode();

        auto mission_msg = std_msgs::msg::String();
        
        switch (mission_mode_) {
          default:
          case FLIGHT:
            publish_trajectory_setpoint();
            mission_msg.data = "FLIGHT";
            break;

          case LANDING:
            land();
            mission_msg.data = "LANDING";
            break;

          case FINISHED:
            if(landed_ && armed_) disarm();
            mission_msg.data = "FINISHED";
            if(!armed_) return;
            break;
        }
        mission_mode_publisher_->publish(mission_msg);
        offboard_setpoint_counter_ ++;
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_sub_;

    px4_msgs::msg::VehicleOdometry curr_odom_;

    enum Mission {
    	STANDBY,
        FLIGHT,
        LANDING,
        FINISHED,
    };

    bool has_odom_ = false;
    bool armed_ = false;
    bool landed_ = false;

    int hold_counter_ = 0;
    int HOLD_THRESHOLD = 20;

    float k = 1.0f;

    void arm();
    void disarm();

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void land();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0);

    int offboard_setpoint_counter_ = 0;

    float desired_x_ = 0.0f;
    float desired_y_ = 0.0f;
    float acc_alt_ = 0.0f;

    float low_enough_ = -0.7f;
    float descent_step_ = 1.0f;
    float iter_ratio_ = 0.2f;
    float rad_to_deg = 180/M_PI;

    float start_x = 0.0f;
    float start_y = 0.0f;
    float start_z = 0.0f;

    float nan = std::numeric_limits<float>::quiet_NaN();

    int start_mode_ = 0;
    int land_mode_ = 0;
    Mission mission_mode_ = STANDBY;
    
    void set_origin();
    float origin[3] = {0, 0, 0};
    bool set_origin_done = false;
    int origin_counter = 0;
    int origin_count_threshold = 10;
};

void LandingTest::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
  armed_ = true;
}

void LandingTest::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
  armed_ = false;
}

void LandingTest::set_origin(){
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
  }  
}

void LandingTest::publish_offboard_control_mode() {
  OffboardControlMode msg {};
  msg.position = mission_mode_ == LANDING && land_mode_ == 1? false:true;
  msg.velocity = mission_mode_ == LANDING && land_mode_ == 1? true:false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void LandingTest::publish_trajectory_setpoint() {
  if(curr_odom_.timestamp == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  TrajectorySetpoint msg {};

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  Eigen::Vector3f target(start_x, start_y, start_z);
  Eigen::Vector3f to_wp = target - current;
  float dist = to_wp.norm();

  if(armed_) {
    msg.position = {target[0]+origin[0], target[1]+origin[1], target[2]+origin[2]};

    if(dist < 3.0f) {
      hold_counter_++;
      if(hold_counter_ > HOLD_THRESHOLD) {
        hold_counter_ = 0;
        mission_mode_ = LANDING;
        RCLCPP_INFO(this->get_logger(), "[LANDING] Initiating landing sequence");
        return;
      }
    }
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::land() {
  TrajectorySetpoint msg {};

  Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]);

  q.normalize();
  Eigen::Vector3f targetFRD (0, 0, 0);
  iter_ratio_ = (land_mode_ == 0? log10f(descent_step_*100)*0.1:descent_step_*0.15*-acc_alt_);
  float l = -acc_alt_*iter_ratio_;
  if(desired_x_ != 0 || desired_y_ != 0) targetFRD = {desired_y_*l, desired_x_*l, 0};

  switch(land_mode_) {
    default:
	    case 0: {//position based landing
      Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], acc_alt_);
      Eigen::Vector3f targetNED = current + q*targetFRD;

      msg.position= {targetNED[0]+origin[0], targetNED[1]+origin[1], acc_alt_ + descent_step_+origin[2]};
      break;
    }

    case 1: {//velocity based landing
      Eigen::Vector3f target_pos_NED = q*targetFRD;
      target_pos_NED.normalize();
      Eigen::Vector3f target_vel_NED = iter_ratio_*target_pos_NED;

      msg.position = {nan, nan, nan};
      msg.velocity = {target_vel_NED[0], target_vel_NED[1], descent_step_};
    }
  }

  if(acc_alt_ > low_enough_) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "[LANDING] Low enough at altitude %.3f. Sending land command.", -acc_alt_);
    mission_mode_ = FINISHED;
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4) {
  VehicleCommand msg {};
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
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  std::cout << "Starting landing test" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingTest>());

  rclcpp::shutdown();
  return 0;
}
