#include <iostream>
#include <chrono>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/*
  수직이착륙 고정익 구조 임무 + 채점 측정 + 비전 정밀착륙 통합.

  비행 흐름:
   - 웨이포인트 미션 (이륙 -> FW 순회 -> REP 구조 -> 복귀)
   - 마지막 복귀 후: NAV_LAND 대신
       (1) 홈 상공 DESCEND_ALT(10m)로 하강 (위치 제어)
       (2) /landing/coordinates(ArUco+라이다) 기반 비전 정밀착륙 (속도 제어)
           정렬되면 NAV_LAND. 저고도+정렬 1회 충족 시 commit 래치(타깃 놓쳐도 착륙 진행).

  인터페이스: marker_recognition_vtol.py 가 /landing/coordinates 발행
    point.x = right(+)[m], point.y = forward(+)[m], point.z = AGL[m]
*/

class FlightTest : public rclcpp::Node {
public:
  FlightTest() : Node("flight_test") {
    odom_sub_ = this->create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](const VehicleOdometry::SharedPtr msg) { curr_odom_ = *msg; has_odom_ = true; });

    lpos_sub_ = this->create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
      [this](const VehicleLocalPosition::SharedPtr msg) { curr_lpos_ = *msg; });

    // 비전 노드(marker_recognition)에서 마커 상대위치 + 라이다 고도 수신
    land_coord_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/landing/coordinates", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        desired_x_  = static_cast<float>(msg->point.x);  // right(+)
        desired_y_  = static_cast<float>(msg->point.y);  // forward(+)
        vision_alt_ = static_cast<float>(msg->point.z);  // AGL [m]
      });

    offboard_control_mode_publisher_ =
      this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
      this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
      this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    build_mission();
    item_scores_.assign(mission_.size(), ItemScore{});

    timer_ = this->create_wall_timer(100ms, [this]() { timer_callback(); });
  }

private:
  enum FlightMode { MULTIROTOR = 3, FIXED_WING = 4, LANDED = 5 };
  enum Phase { FLYING, HOLDING, ALIGNING };
  enum ScoreKind { WAYPOINT, REP_RESCUE, LANDING };
  enum LandStage { DESCEND_10M, VISION_ALIGN };

  struct MissionItem {
    std::array<float, 3> pos;
    FlightMode fly_mode;
    float hold_sec;
    bool  align_to_next_fw;
    bool  land_here;
    std::string name;
    ScoreKind kind;
  };
  struct ItemScore { float min_h = 1e9f; float min_v = 1e9f; };
  struct TransRecord { std::string label; float sink; float peak_g; };

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr land_coord_sub_;

  VehicleOdometry curr_odom_{};
  VehicleLocalPosition curr_lpos_{};
  bool has_odom_ = false, armed_ = false, offboard_enabled_ = false;
  FlightMode flight_mode_ = MULTIROTOR;
  uint64_t offboard_setpoint_counter_ = 0;

  std::vector<MissionItem> mission_;
  std::vector<ItemScore>   item_scores_;
  std::vector<TransRecord> trans_records_;
  size_t item_idx_ = 0;
  Phase  phase_ = FLYING;

  int   hold_counter_ = 0, yaw_hold_counter_ = 0;
  float target_yaw_ = 0.0f;
  float fw_min_dist_ = std::numeric_limits<float>::max();
  float land_error_ = 0.0f;

  // 천이 모니터
  bool  trans_monitoring_ = false;
  int   trans_monitor_left_ = 0;
  float trans_start_alt_ = 0.0f, trans_max_sink_ = 0.0f, trans_peak_g_ = 0.0f;
  std::string trans_label_;

  // ===== 비전 착륙 상태 =====
  bool      landing_active_ = false;
  LandStage land_stage_ = DESCEND_10M;
  float     home_n_ = 0.0f, home_e_ = 0.0f;
  float     desired_x_  = std::numeric_limits<float>::quiet_NaN();  // right(+)
  float     desired_y_  = std::numeric_limits<float>::quiet_NaN();  // forward(+)
  float     vision_alt_ = -1.0f;                                    // AGL [m]
  int       vis_hold_ = 0, vis_lost_ = 0;
  bool      land_committed_ = false, nav_land_sent_ = false;

  // ===== 비행 파라미터 =====
  const float MC_ACCEPT_RADIUS = 3.0f;
  const float FW_GATE          = 40.0f;
  const float FW_RECEDE        = 2.0f;
  const float YAW_ALIGN_TOL    = 0.087f;
  const int   YAW_HOLD_THRESHOLD = 10;
  const float LAND_CENTER_TOL  = 1.0f;
  const int   TRANS_MONITOR_CYCLES = 40;

  // ===== 비전 착륙 파라미터 =====
  const float DESCEND_ALT     = 10.0f;   // 비전 착륙 시작 고도 [m]
  const float DESCEND_TOL     = 1.0f;    // 10m 도달 판정 [m]
  const float LAND_TOL        = 0.8f;    // 정렬 허용 오차 [m]
  const int   ALIGN_NEED      = 5;       // 정렬 연속 프레임
  const float LAND_DEADBAND   = 0.05f;
  const float LAND_MAX_XY     = 0.6f;    // 수평 속도 상한 [m/s]
  const float LAND_MIN_XY     = 0.05f;
  const float LAND_GAIN       = 1.2f;
  const float DESC_HIGH       = 0.40f;   // alt>3m
  const float DESC_MID        = 0.30f;   // 1<alt<=3m
  const float DESC_LOW        = 0.20f;   // alt<=1m
  const float LAND_COMMIT_ALT = 0.4f;    // 이 아래+정렬 -> 착륙 확정(래치)
  const float LAND_LOW_ALT    = 0.7f;    // 이 아래+정렬 -> NAV_LAND

  void build_mission() {
    mission_ = {
      {{{  0.0f,   0.0f,-25.0f}}, MULTIROTOR,  3.0f, true,  false, "WP1(이륙)",   WAYPOINT},
      {{{150.0f,  60.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP2",         WAYPOINT},
      {{{350.0f, 200.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP3",         WAYPOINT},
      {{{350.0f,-200.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP4",         WAYPOINT},
      {{{180.0f, -40.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP5",         WAYPOINT},
      {{{ 60.0f, 120.0f,-25.0f}}, MULTIROTOR, 15.0f, true,  false, "REP(구조)",   REP_RESCUE},
      {{{180.0f, -40.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP5",         WAYPOINT},
      {{{350.0f,-200.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP4",         WAYPOINT},
      {{{350.0f, 200.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP3",         WAYPOINT},
      {{{150.0f,  60.0f,-25.0f}}, FIXED_WING,  0.0f, false, false, "WP2",         WAYPOINT},
      {{{  0.0f,   0.0f,-25.0f}}, MULTIROTOR,  3.0f, false, true,  "WP1(착륙)",   LANDING},
    };
  }

  uint64_t now_us() { return this->get_clock()->now().nanoseconds() / 1000; }

  float current_yaw() {
    const auto &q = curr_odom_.q;  // [w,x,y,z]
    return std::atan2(2.0f*(q[0]*q[3]+q[1]*q[2]), 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]));
  }
  float bearing_to(const std::array<float,3>& p) {
    return std::atan2(p[1]-curr_odom_.position[1], p[0]-curr_odom_.position[0]);
  }
  static float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f*static_cast<float>(M_PI);
    while (a < -M_PI) a += 2.0f*static_cast<float>(M_PI);
    return a;
  }

  // body FRD 벡터 -> NED (PX4 odom q = FRD->NED)
  Eigen::Vector3f body_frd_to_ned(const Eigen::Vector3f &v) const {
    Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]);
    q.normalize();
    return q * v;
  }

  // 현재 AGL: 비전(라이다) 우선, 없으면 odom 고도
  float current_agl() const {
    if (std::isfinite(vision_alt_) && vision_alt_ > 0.01f) return vision_alt_;
    return -curr_odom_.position[2];
  }

  void timer_callback() {
    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for odometry...");
      return;
    }
    if (flight_mode_ == LANDED) return;

    publish_offboard_control_mode();   // 단계에 따라 position/velocity

    // ===== 비전 착륙 시퀀스 =====
    if (landing_active_) {
      run_landing_sequence();
      return;
    }

    // ===== 일반 미션 =====
    if (item_idx_ >= mission_.size()) { flight_mode_ = LANDED; return; }
    publish_trajectory_setpoint();

    offboard_setpoint_counter_++;
    if (!offboard_enabled_ && offboard_setpoint_counter_ > 10) { set_offboard_mode(); offboard_enabled_ = true; }
    if (!armed_ && offboard_setpoint_counter_ > 12) { arm(); }
  }

  void publish_offboard_control_mode() {
    OffboardControlMode msg{};
    if (landing_active_ && land_stage_ == VISION_ALIGN) {
      msg.velocity = true;    // 비전 정밀착륙: x/y/z 속도 제어
    } else {
      msg.position = true;    // 미션 + 10m 하강: 위치 제어
    }
    msg.timestamp = now_us();
    offboard_control_mode_publisher_->publish(msg);
  }

  void publish_trajectory_setpoint() {
    if (item_idx_ >= mission_.size()) return;
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    TrajectorySetpoint msg{};
    msg.velocity = {NaN,NaN,NaN};
    msg.acceleration = {NaN,NaN,NaN};
    msg.jerk = {NaN,NaN,NaN};
    msg.yawspeed = NaN;

    const auto &tgt = mission_[item_idx_].pos;
    msg.position = {tgt[0], tgt[1], tgt[2]};
    msg.yaw = (phase_ == ALIGNING) ? target_yaw_ : NaN;
    msg.timestamp = now_us();
    trajectory_setpoint_publisher_->publish(msg);

    float dN = tgt[0]-curr_odom_.position[0];
    float dE = tgt[1]-curr_odom_.position[1];
    float dD = tgt[2]-curr_odom_.position[2];
    update_mission(std::sqrt(dN*dN+dE*dE+dD*dD), std::sqrt(dN*dN+dE*dE), std::fabs(dD));
  }

  // ===================== 비전 착륙 시퀀스 =====================
  void run_landing_sequence() {
    const float NaN = std::numeric_limits<float>::quiet_NaN();
    TrajectorySetpoint sp{};
    sp.acceleration = {NaN,NaN,NaN};
    sp.jerk = {NaN,NaN,NaN};
    sp.yaw = NaN;        // 헤딩 유지 (body 프레임 안정)
    sp.yawspeed = NaN;

    const float agl = current_agl();

    if (land_stage_ == DESCEND_10M) {
      // 홈 상공 10m로 하강 (위치 제어, x/y는 홈에 유지)
      sp.velocity = {NaN,NaN,NaN};
      sp.position = {home_n_, home_e_, -DESCEND_ALT};
      sp.timestamp = now_us();
      trajectory_setpoint_publisher_->publish(sp);

      if (std::fabs(agl - DESCEND_ALT) < DESCEND_TOL) {
        land_stage_ = VISION_ALIGN;
        RCLCPP_INFO(this->get_logger(),
          "[LAND] reached %.0fm (agl=%.2f). start vision landing.", DESCEND_ALT, agl);
      }
      return;
    }

    // ---- VISION_ALIGN: 비전 정밀착륙 (속도 제어) ----
    const bool valid_xy = std::isfinite(desired_x_) && std::isfinite(desired_y_);
    const bool aligned  = valid_xy &&
                          std::fabs(desired_x_) < LAND_TOL &&
                          std::fabs(desired_y_) < LAND_TOL;

    if (!valid_xy) { vis_lost_++; vis_hold_ = 0; }
    else           { vis_lost_ = 0; vis_hold_ = aligned ? vis_hold_ + 1 : 0; }

    const float ex  = valid_xy ? desired_x_ : 0.0f;   // right
    const float ey  = valid_xy ? desired_y_ : 0.0f;   // forward
    const float err = std::sqrt(ex*ex + ey*ey);

    // 수평 속도 (정렬 전엔 호버하며 마커로 접근)
    float v_forward = 0.0f, v_right = 0.0f;
    if (valid_xy && err >= LAND_DEADBAND) {
      float vmag = LAND_MAX_XY * std::tanh(LAND_GAIN * err);
      vmag = std::min(std::max(vmag, LAND_MIN_XY), LAND_MAX_XY);
      v_right   = vmag * (ex / err);
      v_forward = vmag * (ey / err);
    }
    Eigen::Vector3f v_ned = body_frd_to_ned(Eigen::Vector3f(v_forward, v_right, 0.0f));

    // 하강 속도: 정렬 충족(연속 ALIGN_NEED) 시에만, 고도별 차등
    float vz = 0.0f;
    if (valid_xy && vis_hold_ >= ALIGN_NEED) {
      vz = (agl > 3.0f) ? DESC_HIGH : (agl > 1.0f ? DESC_MID : DESC_LOW);
    }

    sp.position = {NaN, NaN, NaN};
    sp.velocity = {v_ned[0], v_ned[1], vz};   // NED, vz+ = 하강
    sp.timestamp = now_us();
    trajectory_setpoint_publisher_->publish(sp);

    // commit 래치: 한 번이라도 (저고도 + 정렬)이면 확정 -> 이후 타깃 놓쳐도 착륙
    if (valid_xy && vis_hold_ >= ALIGN_NEED && agl < LAND_COMMIT_ALT) {
      land_committed_ = true;
    }

    if (!nav_land_sent_ &&
        (land_committed_ ||
         (valid_xy && vis_hold_ >= ALIGN_NEED && agl < LAND_LOW_ALT))) {
      land_error_ = err;
      nav_land_sent_ = true;
      flight_mode_ = LANDED;
      RCLCPP_INFO(this->get_logger(),
        "[LAND] vision aligned, agl=%.2f, err=%.2f. NAV_LAND.", agl, err);
      land();
      print_score_summary();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "[VLAND] valid=%d hold=%d/%d dx=%.2f dy=%.2f err=%.2f agl=%.2f vz=%.2f",
      valid_xy, vis_hold_, ALIGN_NEED, desired_x_, desired_y_, err, agl, vz);
  }

  // ===================== 미션 상태기계 =====================
  void update_diagnostics() {
    float g = std::sqrt(curr_lpos_.ax*curr_lpos_.ax +
                        curr_lpos_.ay*curr_lpos_.ay +
                        curr_lpos_.az*curr_lpos_.az) / 9.81f;
    if (trans_monitoring_) {
      float alt = -curr_odom_.position[2];
      trans_max_sink_ = std::max(trans_max_sink_, trans_start_alt_ - alt);
      trans_peak_g_   = std::max(trans_peak_g_, g);
      if (--trans_monitor_left_ <= 0) {
        trans_monitoring_ = false;
        trans_records_.push_back({trans_label_, trans_max_sink_, trans_peak_g_});
        RCLCPP_INFO(this->get_logger(), "[TRANS] %s: 침하 %.2fm, 피크 %.2fG",
                    trans_label_.c_str(), trans_max_sink_, trans_peak_g_);
      }
    }
  }

  void update_mission(float dist3d, float dist_h, float dV) {
    update_diagnostics();
    if (!armed_) return;

    if (item_idx_ < item_scores_.size()) {
      item_scores_[item_idx_].min_h = std::min(item_scores_[item_idx_].min_h, dist_h);
      item_scores_[item_idx_].min_v = std::min(item_scores_[item_idx_].min_v, dV);
    }

    MissionItem &it = mission_[item_idx_];

    switch (phase_) {
      case FLYING:
        if (it.fly_mode == MULTIROTOR) {
          if (dist3d < MC_ACCEPT_RADIUS) { hold_counter_ = 0; phase_ = HOLDING;
            RCLCPP_INFO(this->get_logger(), "[MC] reached %s", it.name.c_str()); }
        } else {
          fw_min_dist_ = std::min(fw_min_dist_, dist_h);
          if (fw_min_dist_ < FW_GATE && dist_h > fw_min_dist_ + FW_RECEDE) {
            RCLCPP_INFO(this->get_logger(), "[FW] passed %s (closest %.1fm)", it.name.c_str(), fw_min_dist_);
            advance_item();
          }
        }
        break;

      case HOLDING: {
        if (it.land_here && dist_h > LAND_CENTER_TOL) { hold_counter_ = 0; return; }

        int need = static_cast<int>(it.hold_sec * 10.0f);
        if (++hold_counter_ < need) return;

        if (it.land_here) {
          // === NAV_LAND 대신 비전 착륙 시퀀스 시작 ===
          home_n_ = it.pos[0];
          home_e_ = it.pos[1];
          landing_active_ = true;
          land_stage_ = DESCEND_10M;
          RCLCPP_INFO(this->get_logger(),
            "[LAND] mission done. descend to %.0fm then vision landing.", DESCEND_ALT);
          return;
        }
        if (it.align_to_next_fw && item_idx_+1 < mission_.size()) {
          target_yaw_ = bearing_to(mission_[item_idx_+1].pos);
          yaw_hold_counter_ = 0;
          phase_ = ALIGNING;
          RCLCPP_INFO(this->get_logger(), "[ALIGN] %s -> %.1f deg",
                      it.name.c_str(), target_yaw_*180.0f/static_cast<float>(M_PI));
        } else {
          advance_item();
        }
        break;
      }

      case ALIGNING: {
        float err = wrap_pi(current_yaw() - target_yaw_);
        if (std::fabs(err) < YAW_ALIGN_TOL) {
          if (++yaw_hold_counter_ >= YAW_HOLD_THRESHOLD) {
            RCLCPP_INFO(this->get_logger(), "[ALIGN] aligned (%.1f deg) -> FW",
                        err*180.0f/static_cast<float>(M_PI));
            transition(FIXED_WING);
            advance_item();
          }
        } else { yaw_hold_counter_ = 0; }
        break;
      }
    }
  }

  void advance_item() {
    item_idx_++;
    fw_min_dist_ = std::numeric_limits<float>::max();
    hold_counter_ = 0;
    phase_ = FLYING;

    if (item_idx_ >= mission_.size()) {
      RCLCPP_INFO(this->get_logger(), "Mission complete.");
      flight_mode_ = LANDED;
      print_score_summary();
      return;
    }
    if (mission_[item_idx_].fly_mode == MULTIROTOR && flight_mode_ == FIXED_WING) {
      transition(MULTIROTOR);
    }
  }

  static void wp_tier(float h, float v, const char*& label, int& pts) {
    if (h < 2.0f && v < 2.0f)        { label = "상"; pts = 20; }
    else if (h < 4.0f && v < 4.0f)   { label = "중"; pts = 12; }
    else if (h < 6.0f && v < 10.0f)  { label = "하"; pts = 6;  }
    else                             { label = "미통과"; pts = 0; }
  }

  void print_score_summary() {
    RCLCPP_INFO(this->get_logger(), "================ 채점 추정 요약 ================");
    int total = 0;
    for (size_t i = 0; i < mission_.size(); ++i) {
      auto &m = mission_[i]; auto &s = item_scores_[i];
      if (m.kind == WAYPOINT) {
        const char* t; int p; wp_tier(s.min_h, s.min_v, t, p); total += p;
        RCLCPP_INFO(this->get_logger(), "  [%s] H=%.2fm V=%.2fm -> %s (%d점)",
                    m.name.c_str(), s.min_h, s.min_v, t, p);
      } else if (m.kind == REP_RESCUE) {
        RCLCPP_INFO(this->get_logger(), "  [%s] 도달오차 H=%.2fm V=%.2fm (구조점수 별도)",
                    m.name.c_str(), s.min_h, s.min_v);
      } else {
        float d = land_error_;
        int p = static_cast<int>(std::max(0.0f, 100.0f - 20.0f*d)); total += p;
        RCLCPP_INFO(this->get_logger(), "  [%s] 비전착륙 잔여오차 %.2fm -> %d점 (100-20d)",
                    m.name.c_str(), d, p);
      }
    }
    for (auto &tr : trans_records_) {
      const char* st; int sp;
      if (tr.sink < 2.0f) { st="상"; sp=25; } else if (tr.sink < 5.0f) { st="중"; sp=13; } else { st="하"; sp=7; }
      const char* gt; int gp;
      if (tr.peak_g <= 0.3f) { gt="상"; gp=25; } else if (tr.peak_g <= 0.6f) { gt="중"; gp=13; }
      else if (tr.peak_g <= 0.9f) { gt="하"; gp=7; } else { gt="초과"; gp=0; }
      total += sp + gp;
      RCLCPP_INFO(this->get_logger(), "  [천이 %s] 침하 %.2fm(%s %d) / 피크 %.2fG(%s %d)",
                  tr.label.c_str(), tr.sink, st, sp, tr.peak_g, gt, gp);
    }
    RCLCPP_INFO(this->get_logger(), "  추정 합계(경로점+천이+착륙) %d점", total);
    RCLCPP_INFO(this->get_logger(), "================================================");
  }

  void set_offboard_mode() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f); }
  void arm() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
               armed_ = true; RCLCPP_INFO(this->get_logger(), "Arm command sent"); }
  void land() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
                RCLCPP_INFO(this->get_logger(), "NAV_LAND command sent"); }

  void transition(FlightMode target_mode) {
    if (target_mode == flight_mode_) return;
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
                            static_cast<float>(target_mode), 0.0f);
    RCLCPP_INFO(this->get_logger(), "[TRANSITION] -> %s",
                target_mode == FIXED_WING ? "Fixed Wing" : "Multirotor");
    flight_mode_ = target_mode;

    trans_monitoring_   = true;
    trans_monitor_left_ = TRANS_MONITOR_CYCLES;
    trans_start_alt_    = -curr_odom_.position[2];
    trans_max_sink_     = 0.0f;
    trans_peak_g_       = 0.0f;
    trans_label_        = (target_mode == FIXED_WING) ? "MC->FW" : "FW->MC";
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f) {
    VehicleCommand msg{};
    msg.timestamp = now_us();
    msg.param1 = param1; msg.param2 = param2; msg.command = command;
    msg.target_system = 1; msg.target_component = 1;
    msg.source_system = 1; msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_publisher_->publish(msg);
  }
};

int main(int argc, char *argv[]) {
  std::cout << "Starting VTOL mission + vision landing node..." << std::endl;
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightTest>());
  rclcpp::shutdown();
  return 0;
}
