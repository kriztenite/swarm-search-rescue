#pragma once

#include <cmath>
#include <vector>

struct Pose {
  double x{0.0};
  double y{0.0};
  double theta{0.0}; // Heading angle in radiance
};

struct Velocity {
  double linear{0.0};  // v(m/s)
  double angular{0.0}; // omega (rad/s)
};

class RobotAgent {
public:
  RobotAgent(int id, double x, double y, double max_speed = 2.0);

  // Update state based on kinematic unicycle model
  void step(double dt);

  // Set control targets
  void set_target_velocity(double v, double omega);

  // Getters
  Pose get_pose() const {return pose_;}
  int get_id() const { return id_;}
  double get_battery() const {return battery_level_;}

private:
  int id_;
  Pose pose_;
  Velocity current_vel_;
  Velocity target_vel_;

  double max_speed_;
  double battery_level_{100.0}; // Percentage

  // Kinematic constraints
  double max_linear_accel_{1.5};  // m/s^2
  double max_angular_accel_{3.14}; // rad/s^2
};
