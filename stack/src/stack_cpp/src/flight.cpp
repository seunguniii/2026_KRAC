#include <iostream>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

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
          commandSubscriberCallback(cmd);
        }
      );
      
      getParameters();
      
      //main logic
      auto timer_callback = [this]() -> void {
        reportNodeStatus(self_state);
        
        if(self_state != NodeState::BUSY && self_state != NodeState::SUCCESS)
          return;

        flight();
        offboard_setpoint_counter_++;
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    //PARAMETERS
    void getParameters();
    int HOLD_THRESHOLD = 20;
    int ORIGIN_THRESHOLD = 10;
    float cruise_speed = 15; //m/s
    float front_yaw = 0.0; //deg
    float back_yaw = 0.0; //deg
    float correction_weight = 0.001;
    float horizon_scale = 0.5;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;
    float period_ = 0.1; //0.1s = 100ms
    
    rclcpp::Publisher<UInt32>::SharedPtr status_publisher;

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;

    rclcpp::Subscription<UInt32>::SharedPtr command_subscriber;
    void commandSubscriberCallback(uint32_t cmd);
    
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;
    VehicleOdometry curr_odom_;


    enum FlightMode {
      STANDBY,
      MULTIROTOR = 3,
      FIXED_WING = 4,
      FINISHED
    };

    FlightMode flight_mode_ = STANDBY;
    
    std::string trajectory_dir_;
    std::vector<std::array<float,3>> parseSetpoints(YAML::Node cf);
    std::vector<std::array<float,3>> forward_setpoints;
    std::vector<std::array<float,3>> setpoints_;
    void initSetpoints(std::string dir);
    bool init_setpoints_done = false;
    bool passedSetpoint(float x1, float y1, float x2, float y2, float x_curr, float y_curr);
    
    Eigen::Vector3f old_target = {0.0f, 0.0f, 0.0f};
    
    void setSetpointOrder(MissionMode mode);
    void setDesiredYaw(MissionMode mode);
    float desired_yaw_ = 0.0;
    
    std::array<float,3>hold_position_ = {0.0f, 0.0f, 0.0f};
    bool holding_last_sp_ = false;

    uint64_t offboard_setpoint_counter_ {0};
    size_t sp_idx_ {0};
    int indexStep();
    int index_step = 1;
    bool coordinatesFarEnough(float x1, float y1, float x2, float y2, float eps);

    int hold_counter_ = 0;

    void flight();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void transition(FlightMode mode = MULTIROTOR);
    
    float e = correction_weight*cruise_speed;
    
    void setOrigin();
    float origin[3] = {0, 0, 0};
    bool set_origin_done = false;
    int origin_counter = 0;
    
    MissionManager manager;
    NodeState self_state = NodeState::IDLE;
    MissionMode mission_mode = MissionMode::IDLE;
    void reportNodeStatus(NodeState state);
    
    float nan = std::numeric_limits<float>::quiet_NaN();
};


void Flight::getParameters() {
  declare_parameter<std::string>("trajectory_dir", "");
  get_parameter("trajectory_dir", trajectory_dir_);
  initSetpoints(trajectory_dir_);

  declare_parameter<int>("hold_threshold", 20);
  get_parameter("hold_threshold", HOLD_THRESHOLD);

  declare_parameter<int>("origin_threshold", 10);
  get_parameter("origin_threshold", ORIGIN_THRESHOLD);

  declare_parameter<float>("cruise_speed", 15);
  get_parameter("cruise_speed", cruise_speed);
  
  float pi = 3.14159265;
  float pi_over_180_ = pi/180.0;
  declare_parameter<float>("front_yaw", 0.0);
  get_parameter("front_yaw", front_yaw);
  front_yaw *= pi_over_180_; //convert to rad
  if(front_yaw > pi || front_yaw < -pi) front_yaw = nan;
  
  declare_parameter<float>("back_yaw", 0.0);
  get_parameter("back_yaw", back_yaw);
  back_yaw *= pi_over_180_; //convert to rad
  if(back_yaw > pi || back_yaw < -pi) back_yaw = nan;
  
  declare_parameter<float>("trajectory_correction_weight", 0.001);
  get_parameter("trajectory_correction_weight", correction_weight);
  
  declare_parameter<float>("trajectory_horizon_scale", 0.5);
  get_parameter("trajectory_horizon_scale", horizon_scale);
}


void Flight::commandSubscriberCallback(uint32_t cmd) {
  if(manager.get_node(cmd) != NodeName::FLIGHT) return;
  mission_mode = manager.get_mode(cmd);
  NodeState command_state = manager.get_command(cmd);
  if(self_state == command_state) return;
  if(command_state == NodeState::BUSY) {
    flight_mode_ = STANDBY;
    hold_counter_ = 0;
    sp_idx_ = 0;
    holding_last_sp_ = false;
              
    setSetpointOrder(mission_mode);
    setDesiredYaw(mission_mode);
    hold_position_ = setpoints_.back();
  }
  self_state = command_state;
  RCLCPP_INFO(get_logger(), "Command recieved from MISSION.");
}


void Flight::reportNodeStatus(NodeState state) {
  std_msgs::msg::UInt32 msg;
  msg.data = manager.pack(NodeName::FLIGHT, state);
  status_publisher -> publish(msg);
}


//helper for initSetpoints()
std::vector<std::array<float, 3>> Flight::parseSetpoints(YAML::Node cf) {
  std::vector<std::array<float, 3>> sps;
  for (const auto& sp : cf["setpoints"]){
    sps.push_back({
      sp["n"].as<float>(), 
      sp["e"].as<float>(),
      sp["d"].as<float>()
    });
  }
  
  return sps;
}

void Flight::initSetpoints(std::string dir) {
  YAML::Node config = YAML::LoadFile(dir);
  
  forward_setpoints.clear();
  forward_setpoints = parseSetpoints(config);
  setpoints_ = forward_setpoints;
  
  hold_position_ = forward_setpoints.back();
  
  init_setpoints_done = true;
  RCLCPP_INFO(this->get_logger(), "Done parsing trajectory setpoints.");
}

void Flight::setSetpointOrder(MissionMode mode) {
    if (mode == MissionMode::WP_FLIGHT) setpoints_ = forward_setpoints;
    else if (mode == MissionMode::INVERSE_WP_FLIGHT) {
        holding_last_sp_ = false;
        setpoints_ = forward_setpoints;
        std::reverse(setpoints_.begin(), setpoints_.end());
    }
    else self_state = NodeState::ABORT;
}

void Flight::setDesiredYaw(MissionMode mode) {
    if (mode == MissionMode::WP_FLIGHT) desired_yaw_ = front_yaw;
    else if (mode == MissionMode::INVERSE_WP_FLIGHT) desired_yaw_ = back_yaw;
    else self_state = NodeState::ABORT;
}


//main logic
void Flight::flight() {
  TrajectorySetpoint msg {};

  Eigen::Vector3f curr_p(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  
  std::array<float,3> target_sp = holding_last_sp_? hold_position_ : setpoints_[sp_idx_];
  
  Eigen::Vector3f target(target_sp[0], target_sp[1], target_sp[2]);
  msg.position = {target_sp[0], target_sp[1], target_sp[2]};
  
  Eigen::Vector3f to_sp = target - curr_p;
  float dist_to_sp = to_sp.norm();
  
  switch(flight_mode_){
    case STANDBY:
      if(set_origin_done) {
        flight_mode_ = MULTIROTOR;
        break;
      }
      if(init_setpoints_done) {
        setOrigin();
        break;
      }
    break;
      
    case MULTIROTOR:
      msg.yaw = desired_yaw_;
      if(holding_last_sp_) {
        if(dist_to_sp < 3.0f) {
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
        if(dist_to_sp < 3.0f) {
          hold_counter_++;
          
          if(hold_counter_ > HOLD_THRESHOLD) {
            hold_counter_ = 0;
            
            old_target = target;
            sp_idx_ ++;
            
            if(sp_idx_ == 1) transition(FIXED_WING);
          }
        } else
          hold_counter_ = 0;
      }
      break;
    
    case FIXED_WING: {
      index_step = indexStep();
      Eigen::Vector3f new_target = {
        setpoints_[sp_idx_ + index_step][0],
        setpoints_[sp_idx_ + index_step][1],
        setpoints_[sp_idx_ + index_step][2]
      };

      //Eigen::Vector3f trajectory_segment = (new_target - target).normalized();
      Eigen::Vector3f trajectory_segment = (target - old_target).normalized();
      msg.velocity = {
        (cruise_speed - e)*trajectory_segment.x() + e*to_sp.x(),
        (cruise_speed - e)*trajectory_segment.y() + e*to_sp.y(),
        0.0f
      };
      
      if (passedSetpoint(
        old_target[0], old_target[1], 
        target[0], target[1], 
        //new_target[0], new_target[1],
        curr_p[0], curr_p[1]
      )) {
        old_target = target;
        sp_idx_ += index_step;
        index_step = 1;
        
        if(sp_idx_ >= setpoints_.size() - 1) {
          holding_last_sp_ = true;
          transition(MULTIROTOR);
        }
      }
      break;
    }
    
    case FINISHED:
      msg.position = {hold_position_[0], hold_position_[1], hold_position_[2]};
      break;
  }
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher->publish(msg);
}


void Flight::setOrigin(){
  if(origin_counter < ORIGIN_THRESHOLD){
    origin[0] += curr_odom_.position[0];
    origin[1] += curr_odom_.position[1];
    origin[2] += curr_odom_.position[2];
    origin_counter ++;
  }
  else {    
    origin[0] /= ORIGIN_THRESHOLD;
    origin[1] /= ORIGIN_THRESHOLD;
    origin[2] /= ORIGIN_THRESHOLD;

    set_origin_done = true;
    RCLCPP_INFO(this->get_logger(), "Origin set to (%f, %f, %f)", origin[0], origin[1], origin[2]);
    for(int i = 0; i < setpoints_.size(); i++){
      setpoints_[i][0] += origin[0];
      setpoints_[i][1] += origin[1];
      setpoints_[i][2] += origin[2];
    }
  }  
}


//checks if (x_curr, y_curr) passed (x1, y1)
bool Flight::passedSetpoint(float x1, float y1, float x2, float y2, float x_curr, float y_curr) {
  float trajectory_vector[2] = {x2 - x1, y2 - y1};
  float current_pos_vector[2] = {x_curr - x1, y_curr - y1};
  
  float inner_product = trajectory_vector[0]*current_pos_vector[0] + trajectory_vector[1]*current_pos_vector[1];
  
  float segment_length_squared = trajectory_vector[0]*trajectory_vector[0] + trajectory_vector[1]*trajectory_vector[1];
  
  //return inner_product > 0; //new target
  return inner_product > segment_length_squared/horizon_scale; //old target
}


//helper for index_step
bool Flight::coordinatesFarEnough(float x1, float y1, float x2, float y2, float eps) {
  return (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) > eps*eps;
}

int Flight::indexStep() {
  int step_ = 1;
  float eps_ = cruise_speed*period_*horizon_scale;
  
  while(sp_idx_ + step_ < setpoints_.size() &&
        !coordinatesFarEnough(
          setpoints_[sp_idx_][0], setpoints_[sp_idx_][1],
          setpoints_[sp_idx_+step_][0], setpoints_[sp_idx_+step_][1],
          eps_    
        )
  ) {step_ ++;}

  return step_;
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
