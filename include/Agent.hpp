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

  // Communication inbox methods
  void receive_message(const Message& msg);
  const std::vector<Message>& get_inbox() const { return inbox_; }
  void clear_inbox();

  // Getters
  Pose get_pose() const { return pose_; }
  int get_id() const { return id_; }
  double get_battery() const { return battery_level_; }
  const std::vector<std::pair<double, double>>& get_known_survivors() const {
    return known_survivors_;
  }

  // Add a discovered survivor to local memory
  void add_survivor_memory(double x, double y);

 private:
  int id_;
  Pose pose_;
  Velocity current_vel_;
  Velocity target_vel_;

  double max_speed_;
  double battery_level_{100.0};  // Percentage

  // Kinematic constraints
  double max_linear_accel_{1.5};    // m/s^2
  double max_angular_accel_{3.14};  // rad/s^2

  // Navigate target state
  bool has_target_{false};
  double target_x_{0.0};
  double target_y_{0.0};

  // Local agent memory and communication state
  std::vector<Message> inbox_;
  std::vector<std::pair<double, double>> known_survivors_;

  // Helper functions
  void updateProportionalSteering();
  void updateVelocities(double dt);
  void updatePose(double dt, const GridMap& map);
  void updateBattery(double dt);
};
