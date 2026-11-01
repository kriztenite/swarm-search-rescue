#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "GridMap.hpp"
#include "RobotConfig.hpp"

enum class RobotBehavior { SEARCHING, RETURNING_TO_BASE, RECHARGING };

struct Message {
  uint64_t sender_id{0};
  uint64_t origin_id{0};
  uint64_t message_id{0};  // use uint64_t to avoid collisions
  uint8_t hop_count{0};
  uint8_t max_hops{5};

  double survivor_x{0.0};
  double survivor_y{0.0};
  bool survivor_found{false};
  bool visited{false};
  double timestamp{0.0};

  // Prefer trivially copyable structs(no virtual functions, POD)
} __attribute__((packed));

static_assert(std::is_trivially_copy_assignable_v<Message>,
              "Message must be POD");

struct SurvivorRecord {
  double x{0.0};
  double y{0.0};
  bool visited{false};
  double timestamp{0.0};
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
  explicit RobotAgent(uint64_t id, double x, double y,
                      const RobotConfig& config = RobotConfig{});
  // RobotAgent(const RobotAgent&) = delete;  // prevent accidental copies
  // RobotAgent& operator=(const RobotAgent) = delete;

  // step simulation
  void step(double dt, double current_time, const GridMap& map);

  // Direct velocity control(overrides navigation)
  void set_target_velocity(double v, double omega);

  // Navigate targets
  void set_navigation_target(double tx, double ty);
  [[nodiscard]] bool has_target() const { return has_target_; }

  // Communication
  void receive_message(const Message& msg);
  [[nodiscard]] const std::vector<Message>& get_inbox() const { return inbox_; }
  [[nodiscard]] const std::vector<Message>& get_outbox() const {
    return outbox_;
  }
  void clear_inboxes();

  // Getters
  [[nodiscard]] const Pose& get_pose() const { return pose_; }
  [[nodiscard]] int get_id() const { return id_; }
  [[nodiscard]] double get_battery() const { return battery_level_; }
  [[nodiscard]] RobotBehavior get_behavior() const { return behavior_; }
  [[nodiscard]] const std::vector<SurvivorRecord>& get_known_survivors() const {
    return known_survivors_;
  }

  // Manage memory
  void add_survivor_memory(double x, double y, bool visited,
                           double current_time);
  void mark_survivor_visited(double x, double y);
  void prune_expired_memories(double current_time);

 private:
  // Identify Agent & state
  uint64_t id_;
  Pose pose_;
  Velocity current_vel_;
  Velocity target_vel_;
  RobotConfig config_;

  // Battery & behavior
  double battery_level_{100.0};
  RobotBehavior behavior_{RobotBehavior::SEARCHING};
  double last_message_time_{0.0};  // for message throttling

  // Navigation
  bool has_target_{false};
  double target_x_{0.0};
  double target_y_{0.0};

  // Communication
  std::vector<Message> inbox_;
  std::vector<Message> outbox_;
  std::vector<int> seen_message_ids_;

  // memory(pruned automatically)
  std::vector<SurvivorRecord> known_survivors_;

  // Helper methods
  [[nodiscard]] uint64_t generate_message_id(double current_time);
};
