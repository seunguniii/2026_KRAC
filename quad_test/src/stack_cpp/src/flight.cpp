#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "stack_cpp/mission_manager.h"

using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace px4_msgs::msg;

// Unified struct for all 4D setpoints
struct Setpoint {
    float x;
    float y;
    float z;
    float yaw; // Radians in [-PI, PI]
};

// Helper to wrap yaw to [-PI, PI]
inline float normalize_angle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

class Flight : public rclcpp::Node {
public:
    Flight() : Node("Flight") {
        status_publisher = this->create_publisher<UInt32>("nodes/flight/status", 10);
        
        trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        vehicle_odometry_subscriber = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const VehicleOdometry::SharedPtr msg) {
                curr_odom_ = *msg;
                has_odom_ = true;
            });

        command_subscriber = this->create_subscription<UInt32>(
            "mission/command", 10,
            [this](const UInt32::SharedPtr msg) {
                commandSubscriberCallback(msg->data);
            });
        
        getParameters();
        
        // Main Loop
        auto timer_callback = [this]() -> void {
            reportNodeStatus(self_state);
            
            if (self_state != NodeState::BUSY && self_state != NodeState::SUCCESS)
                return;

            flight();
            offboard_setpoint_counter_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    // Parameters
    void getParameters();
    int HOLD_THRESHOLD = 20;
    int ORIGIN_THRESHOLD = 10;
    float trajectory_step = 0.1;
    float arc_radius = 2.0;
    float straight_line_length = 4.0;
    const float ACCEPTANCE_RADIUS = 0.3f; // 30cm
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Publishers & Subscribers
    rclcpp::Publisher<UInt32>::SharedPtr status_publisher;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
    rclcpp::Subscription<UInt32>::SharedPtr command_subscriber;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;
    
    // State Variables
    VehicleOdometry curr_odom_;
    bool has_odom_ = false;
    uint64_t offboard_setpoint_counter_{0};

    enum FlightMode {
        STANDBY,
        MULTIROTOR = 3,
        FINISHED
    };
    FlightMode flight_mode_ = STANDBY;

    // Trajectory Tracking
    std::vector<Setpoint> generateTrajectory(float step_size, float depth);
    std::vector<Setpoint> setpoints_;
    std::vector<Setpoint> forward_setpoints_; // Cache for forward direction

    void initSetpoints();
    bool init_setpoints_done = false;
    
    size_t sp_idx_{0};
    int hold_counter_ = 0;
    
    Setpoint hold_position_{0.0f, 0.0f, 0.0f, 0.0f};
    bool holding_last_sp_ = false;

    // Origin Tracking
    void setOrigin();
    Eigen::Vector3f origin_{0.0f, 0.0f, 0.0f};
    float initial_yaw_ = 0.0f;
    bool set_origin_done = false;
    int origin_counter_ = 0;

    // Mission Management
    MissionManager manager;
    NodeState self_state = NodeState::IDLE;
    MissionMode mission_mode = MissionMode::IDLE;
    
    void commandSubscriberCallback(uint32_t cmd);
    void reportNodeStatus(NodeState state);
    void setSetpointOrder(MissionMode mode);
    
    void flight();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    
    float nan = std::numeric_limits<float>::quiet_NaN();
};


void Flight::getParameters() {
    declare_parameter<int>("hold_threshold", 20);
    get_parameter("hold_threshold", HOLD_THRESHOLD);

    declare_parameter<int>("origin_threshold", 10);
    get_parameter("origin_threshold", ORIGIN_THRESHOLD);
    
    declare_parameter<float>("trajectory_step", 0.3);
    get_parameter("trajectory_step", trajectory_step);
    
    declare_parameter<float>("arc_radius", 2.0);
    get_parameter("arc_radius", arc_radius);
    
    declare_parameter<float>("straight_line_length", 0.3);
    get_parameter("straight_line_length", straight_line_length);
}


void Flight::commandSubscriberCallback(uint32_t cmd) {
    if (manager.get_node(cmd) != NodeName::FLIGHT) return;
    
    mission_mode = manager.get_mode(cmd);
    NodeState command_state = manager.get_command(cmd);
    
    if (self_state == command_state) return;
    
    if (command_state == NodeState::BUSY) {
        flight_mode_ = STANDBY;
        hold_counter_ = 0;
        sp_idx_ = 0;
        holding_last_sp_ = false;
                
        setSetpointOrder(mission_mode);
        if (!setpoints_.empty()) {
            hold_position_ = setpoints_.back();
        }
    }
    self_state = command_state;
    RCLCPP_INFO(get_logger(), "Command received from MISSION.");
}


void Flight::reportNodeStatus(NodeState state) {
    std_msgs::msg::UInt32 msg;
    msg.data = manager.pack(NodeName::FLIGHT, state);
    status_publisher->publish(msg);
}


std::vector<Setpoint> Flight::generateTrajectory(float step_size = 0.1f, float depth = -3.0f) {
    std::vector<Setpoint> sps;
    const float R = arc_radius; 
    const float L = straight_line_length;
    const float M_PI_F = static_cast<float>(M_PI);

    auto add_setpoint = [&](float x, float y, float raw_yaw) {
        sps.push_back({x, y, depth, normalize_angle(raw_yaw)});
    };

    const float arc_len = (M_PI_F / 2.0f) * R; 

    // Segment 1: Straight 6m
    for (float s = 0; s < L+R; s += step_size) add_setpoint(s, 0.0f, 0.0f);

    // Segment 2: CCW Arc 1
    for (float s = 0; s < arc_len; s += step_size) {
        float theta = (M_PI_F / 2.0f) - (s / R);
        add_setpoint(L+R + R * std::cos(theta), -R + R * std::sin(theta), theta - (M_PI_F / 2.0f));
    }

    // Segment 3: Straight 4m
    for (float s = 0; s < L; s += step_size) add_setpoint(2*R + L, -R - s, -M_PI_F / 2.0f);

    // Segment 4: CCW Arc 2
    for (float s = 0; s < arc_len; s += step_size) {
        float theta = 0.0f - (s / R);
        add_setpoint(R + L + R * std::cos(theta), -(R + L) + R * std::sin(theta), theta - (M_PI_F / 2.0f));
    }

    // Segment 5: Straight 4m
    for (float s = 0; s < L; s += step_size) add_setpoint((R + L) - s, -(2*R + L), -M_PI_F);

    // Segment 6: CCW Arc 3
    for (float s = 0; s < arc_len; s += step_size) {
        float theta = (-M_PI_F / 2.0f) - (s / R);
        add_setpoint(R + R * std::cos(theta), -(R+L) + R * std::sin(theta), theta - (M_PI_F / 2.0f));
    }

    // Segment 7: Straight 6m
    for (float s = 0; s <= (R+L); s += step_size) add_setpoint(0.0f, -(R+L) + s, M_PI_F / 2.0f);

    return sps;
}


void Flight::setOrigin() {
    if (!has_odom_) return;

    if (origin_counter_ < ORIGIN_THRESHOLD) {
        origin_[0] += curr_odom_.position[0];
        origin_[1] += curr_odom_.position[1];
        origin_[2] += curr_odom_.position[2];
        origin_counter_++;
    } 
    else if (!set_origin_done) {  
        origin_ /= static_cast<float>(ORIGIN_THRESHOLD);

        // Direct, robust Quaternion to Yaw calculation (Z-Y-X sequence)
        // PX4 Quaternion format: q[0] = w, q[1] = x, q[2] = y, q[3] = z
        float w = curr_odom_.q[0];
        float x = curr_odom_.q[1];
        float y = curr_odom_.q[2];
        float z = curr_odom_.q[3];

        // Standard Z-Y-X Yaw extraction
        initial_yaw_ = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

        set_origin_done = true;
        RCLCPP_INFO(this->get_logger(), "Origin set to NED: (%f, %f, %f) Yaw: %f rad (%f deg)", 
                    origin_[0], origin_[1], origin_[2], initial_yaw_, initial_yaw_ * 180.0f / M_PI);
        
        initSetpoints();
    }
}


void Flight::initSetpoints() {
    auto frd_trajectory = generateTrajectory(0.3f, -3.0f);
    
    setpoints_.clear();
    setpoints_.reserve(frd_trajectory.size());

    // Transform FRD trajectory into absolute NED
    for (const auto& sp : frd_trajectory) {
        float n = origin_[0] + (sp.x * std::cos(initial_yaw_) - sp.y * std::sin(initial_yaw_));
        float e = origin_[1] + (sp.x * std::sin(initial_yaw_) + sp.y * std::cos(initial_yaw_));
        float d = origin_[2] + sp.z;
        float yaw_ned = normalize_angle(initial_yaw_ + sp.yaw);

        setpoints_.push_back({n, e, d, yaw_ned});
    }

    forward_setpoints_ = setpoints_; // Cache original forward order
    
    if (!setpoints_.empty()) {
        hold_position_ = setpoints_.back();
    }

    init_setpoints_done = true;
    RCLCPP_INFO(this->get_logger(), "Generated and converted %zu trajectory setpoints to NED.", setpoints_.size());
}


void Flight::setSetpointOrder(MissionMode mode) {
    if (mode == MissionMode::WP_FLIGHT) {
        setpoints_ = forward_setpoints_;
    } 
    else if (mode == MissionMode::INVERSE_WP_FLIGHT) {
        setpoints_ = forward_setpoints_;
        std::reverse(setpoints_.begin(), setpoints_.end());
        
        // Reverse required yaw by 180 degrees
        for (auto& sp : setpoints_) {
            sp.yaw = normalize_angle(sp.yaw + M_PI);
        }
    } 
    else {
        self_state = NodeState::ABORT;
    }
}


void Flight::flight() {
    if (setpoints_.empty() && init_setpoints_done) return;

    TrajectorySetpoint msg{};

    Eigen::Vector3f curr_p(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);

    switch (flight_mode_) {
        case STANDBY:
            if (!set_origin_done) {
                setOrigin();
            } else if (init_setpoints_done) {
                sp_idx_ = 0;
                holding_last_sp_ = false;
                flight_mode_ = MULTIROTOR;
                RCLCPP_INFO(this->get_logger(), "Standby complete. Starting multicopter trajectory tracking.");
            }
            break;

        case MULTIROTOR: {
            const Setpoint& target_sp = holding_last_sp_ ? hold_position_ : setpoints_[sp_idx_];
            Eigen::Vector3f target(target_sp.x, target_sp.y, target_sp.z);
            float dist_to_sp = (target - curr_p).norm();

            msg.position = {target_sp.x, target_sp.y, target_sp.z};
            msg.yaw = target_sp.yaw;

            if (holding_last_sp_) {
                // Wait to verify we stabilized at the final waypoint
                if (dist_to_sp < ACCEPTANCE_RADIUS) {
                    hold_counter_++;
                    if (hold_counter_ > HOLD_THRESHOLD) {
                        RCLCPP_INFO(this->get_logger(), "Finished trajectory flight successfully.");
                        self_state = NodeState::SUCCESS;
                        flight_mode_ = FINISHED;
                    }
                } else {
                    hold_counter_ = 0;
                }
            } 
            else {
                // Continuous trajectory tracking without pausing
                if (dist_to_sp < ACCEPTANCE_RADIUS) {
                    sp_idx_++;
                    
                    if (sp_idx_ >= setpoints_.size()) {
                        sp_idx_ = setpoints_.size() - 1;
                        hold_position_ = setpoints_.back();
                        holding_last_sp_ = true;
                        hold_counter_ = 0;
                        RCLCPP_INFO(this->get_logger(), "Reached last setpoint. Holding position...");
                    }
                }
            }
            break;
        }

        case FINISHED:
            msg.position = {hold_position_.x, hold_position_.y, hold_position_.z};
            msg.yaw = hold_position_.yaw;
            break;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher->publish(msg);
}


void Flight::publishVehicleCommand(uint16_t command, float param1, float param2) {
    VehicleCommand msg{};
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


int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Flight>());

    rclcpp::shutdown();
    return 0;
}
