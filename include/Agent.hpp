#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "GridMap.hpp"

struct Message {
  int sender_id;
  double survivor_x;
  double survivor_y;
  bool survivor_found;
  bool visited{false};
};

struct SurvivorRecord {
  double x;
  double y;
  bool visited{false};
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
  RobotAgent(int id, double x, double y, double max_speed = 2.0);

  void step(double dt, const GridMap& map);
  void set_target_velocity(double v, double omega);

  // Set a physical world coordinate target for navigation
  void set_navigation_target(double tx, double ty);
  bool has_target() const { return has_target_; }

  // Communication inbox methods
  void receive_message(const Message& msg);
  const std::vector<Message>& get_inbox() const { return inbox_; }
  void clear_inbox();

  // Getters
  Pose get_pose() const { return pose_; }
  int get_id() const { return id_; }
  double get_battery() const { return battery_level_; }
  const std::vector<SurvivorRecord>& get_known_survivors() const {
    return known_survivors_;
  }

  // Add a discovered survivor to local memory
  void add_survivor_memory(double x, double y, bool visited = false);
  void mark_survivor_visited(double x, double y);

 private:
  int id_;
  Pose pose_;
  Velocity current_vel_;
  Velocity target_vel_;

  double max_speed_;
  double battery_level_{100.0};  // Percentage
  // obstacle avoidance
  double lookahead_distance_{40.0};
  double diverg_angle_{0.785};  // 45 degrees
  double lookahead_gain_{0.7};
  double steer_direction_gain_{1.5};
  double vel_reduction_gain_{0.7};
  double vel_reduction_deadend{0.3};

  double kp_angular_{2.5};
  double target_reached_threshold_{15.0};
  // Kinematic constraints
  double max_linear_accel_{1.5};    // m/s^2
  double max_angular_accel_{3.14};  // rad/s^2

  // Navigate target state
  bool has_target_{false};
  double target_x_{0.0};
  double target_y_{0.0};

  // Local agent memory and communication state
  std::vector<Message> inbox_;
  std::vector<SurvivorRecord> known_survivors_;

  // Helper functions
  // void updateNavigation();  // Navigation & steering
  // void avoidObstacles(const GridMap& map);
  // void updateVelocities(double dt);                // Velocity smoothing
  // void updatePose(double dt, const GridMap& map);  // Physics & collision
  // void updateBattery(double dt);                   // Battery Management
};
