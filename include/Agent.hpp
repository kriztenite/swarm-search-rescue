#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "GridMap.hpp"

struct Message {
  int sender_id{0};
  int origin_id{0};
  int message_id{0};
  int hop_count{0};
  int max_hops{5};
  double survivor_x{0.0};
  double survivor_y{0.0};
  bool survivor_found{false};
  bool visited{false};
  double timestamp{0.0};
};

struct SurvivorRecord {
  double x{0.0};
  double y{0.0};
  bool visited{false};
  double timestamp{0.0};
};

struct RobotConfig {
  double max_speed{130.0};
  double max_angular_vel{2.5};
  double max_linear_accel{1.5};
  double max_angular_accel{3.14};
  double target_reached_threshold{15.0};
  double lookahead_distance{40.0};
  double kp_angular{2.5};
  double survivor_ttl{120.0};  // Time-to-live in seconds
  double duplicate_cluster_radius{25.0};
};

struct Pose {
  double x{0.0};
  double y{0.0};
  double theta{0.0};  // Heading angle in radiance
};

struct Velocity {
  double linear{0.0};   // v(m/s)
  double angular{0.0};  // omega (rad/s)
};

class RobotAgent {
 public:
  RobotAgent(int id, double x, double y,
             const RobotConfig& config = RobotConfig{});

  void step(double dt, double current_time, const GridMap& map);
  void set_target_velocity(double v, double omega);

  void set_navigation_target(double tx, double ty);
  bool has_target() const { return has_target_; }

  // Communication inbox methods
  void receive_message(const Message& msg);
  const std::vector<Message>& get_inbox() const { return inbox_; }
  const std::vector<Message>& get_outbox() const { return outbox_; }
  void clear_inboxes();

  // Getters
  const Pose& get_pose() const { return pose_; }
  int get_id() const { return id_; }
  double get_battery() const { return battery_level_; }
  const std::vector<SurvivorRecord>& get_known_survivors() const {
    return known_survivors_;
  }

  // Add a discovered survivor to local memory
  void add_survivor_memory(double x, double y, bool visited,
                           double current_time);
  void mark_survivor_visited(double x, double y);
  void prune_expired_memories(double current_time);

 private:
  int id_;
  Pose pose_;
  Velocity current_vel_;
  Velocity target_vel_;

  RobotConfig config_;
  double battery_level_{100.0};

  // Navigate target state
  bool has_target_{false};
  double target_x_{0.0};
  double target_y_{0.0};

  // Local agent memory and communication state
  std::vector<Message> inbox_;
  std::vector<Message> outbox_;
  std::vector<int> seen_message_ids_;
  std::vector<SurvivorRecord> known_survivors_;
};
