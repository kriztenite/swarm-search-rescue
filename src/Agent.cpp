#include "Agent.hpp"

#include <algorithm>
#include <cmath>

RobotAgent::RobotAgent(int id, double x, double y, double max_speed)
    : id_(id), max_speed_(max_speed) {
  pose_.x = x;
  pose_.y = y;
  pose_.theta = 0.0;
}

void RobotAgent::set_target_velocity(double v, double omega) {
  target_vel_.linear = std::clamp(v, -max_speed_, max_speed_);
  target_vel_.angular = omega;
}

void RobotAgent::receive_message(const Message& msg) {
  // Avoid storing duplicate messages
  inbox_.push_back(msg);
}

void RobotAgent::clear_inbox() {
  inbox_.clear();
}

void RobotAgent::add_survivor_memory(double x, double y) {
  // Avoid duplicates
  auto it = std::find_if(known_survivors_.begin(), known_survivors_.end(),
                         [x, y](const auto& pos) {
                           return std::abs(pos.first - x) < 5.0 &&
                                  std::abs(pos.second - y) < 5.0;
                         });
  if (it == known_survivors_.end())
    known_survivors_.push_back({x, y});
}

void RobotAgent::step(double dt, const GridMap& map) {
  // Apply acceleration limits to linear velocity
  double dv = target_vel_.linear - current_vel_.linear;
  double max_dv = max_linear_accel_ * dt;
  current_vel_.linear += std::clamp(dv, -max_dv, max_dv);

  // Appyly acceleration limits to angular velocity
  double domega = target_vel_.angular - current_vel_.angular;
  double max_domega = max_angular_accel_ * dt;
  current_vel_.angular += std::clamp(domega, -max_domega, max_domega);

  // Predict the next position using unicycle kinematics
  auto next_x = pose_.x + current_vel_.linear * std::cos(pose_.theta) * dt;
  auto next_y = pose_.y + current_vel_.linear * std::sin(pose_.theta) * dt;

  // Check if the predicted position collides with an obstacle on the GridMap
  if (!map.is_collision(next_x, next_y)) {
    // Safe to move
    pose_.x = next_x;
    pose_.y = next_y;
  } else {
    // Collision detected: stop linear velocity and reverse target slightly
    current_vel_.linear = 0.0;
    target_vel_.linear = -target_vel_.linear * 0.5;
  }

  pose_.theta += current_vel_.angular * dt;
  // Normalize theta to [-pi, pi]
  pose_.theta = std::atan2(std::sin(pose_.theta), std::cos(pose_.theta));

  // Deplete battery proportionally to movement
  double speed_factor =
      std::abs(current_vel_.linear) + std::abs(current_vel_.angular) * 0.2;
  battery_level_ =
      std::max(0.0, battery_level_ - (0.05 + 0.1 * speed_factor) * dt);
}
