#include "Agent.hpp"

#include <algorithm>

RobotAgent::RobotAgent(int id, double x, double y, double max_speed)
    : id_(id), max_speed_(max_speed) {
  pose_.x = x;
  pose_.y = y;
  pose_.theta = 0.0;
}

void RobotAgent::set_target_velocity(double v, double omega) {
  has_target_ = false;  // Overide to disable active navigation
  target_vel_.linear = std::clamp(v, -max_speed_, max_speed_);
  target_vel_.angular = omega;
}

void RobotAgent::set_navigation_target(double tx, double ty) {
  target_x_ = tx;
  target_y_ = ty;
  has_target_ = true;
}

void RobotAgent::receive_message(const Message& msg) {
  // Avoid storing duplicate messages
  auto exists = std::any_of(known_survivors_.begin(), known_survivors_.end(),
                            [&msg](const SurvivorRecord& rec) {
                              return std::hypot(rec.x - msg.survivor_x,
                                                rec.y - msg.survivor_y) < 25.0;
                            });
  if (!exists) {
    inbox_.push_back(msg);
    add_survivor_memory(msg.survivor_x, msg.survivor_y);
  }
}

void RobotAgent::clear_inbox() {
  inbox_.clear();
}

void RobotAgent::add_survivor_memory(double x, double y, bool visited) {
  // Avoid duplicates
  auto it = std::find_if(known_survivors_.begin(), known_survivors_.end(),
                         [x, y](const SurvivorRecord& pos) {
                           return std::hypot(pos.x - x, pos.y - y);
                         });
  if (it == known_survivors_.end()) {
    known_survivors_.push_back({x, y, visited});
  } else {
    if (visited)
      it->visited = true;
  }
}

void RobotAgent::mark_survivor_visited(double x, double y) {
  for (auto& rec : known_survivors_) {
    if (std::hypot(rec.x - x, rec.y - y) < 25.0) {
      rec.visited = true;
    }
  }
}

void RobotAgent::step(double dt, const GridMap& map) {
  bool obstacle_detected = false;
  double avoidance_omega = 0.0;
  double avoidance_linear = max_speed_;

  // 1. Proactive Obstacle Avoidance (Raycasting lookahead)
  double lookahead_x = pose_.x + lookahead_distance_ * std::cos(pose_.theta);
  double lookahead_y = pose_.y + lookahead_distance_ * std::sin(pose_.theta);

  if (map.is_collision(lookahead_x, lookahead_y)) {
    obstacle_detected = true;

    double left_angle = pose_.theta + 0.785;   // +45 degrees
    double right_angle = pose_.theta - 0.785;  // -45 degrees

    double left_x =
        pose_.x + (lookahead_distance_ * 0.7) * std::cos(left_angle);
    double left_y =
        pose_.y + (lookahead_distance_ * 0.7) * std::sin(left_angle);

    double right_x =
        pose_.x + (lookahead_distance_ * 0.7) * std::cos(right_angle);
    double right_y =
        pose_.y + (lookahead_distance_ * 0.7) * std::sin(right_angle);

    bool left_clear = !map.is_collision(left_x, left_y);
    bool right_clear = !map.is_collision(right_x, right_y);

    // Assign ABSOLUTE turning velocities (no cumulative += bug)
    if (left_clear && !right_clear) {
      avoidance_omega = 2.0;  // Turn left safely
      avoidance_linear = max_speed_ * 0.5;
    } else if (right_clear && !left_clear) {
      avoidance_omega = -2.0;  // Turn right safely
      avoidance_linear = max_speed_ * 0.5;
    } else {
      avoidance_omega = 2.5;  // Spin out of corner
      avoidance_linear = 0.1;
    }
  }

  // 2. Navigation Control (Only active if NOT dodging an obstacle)
  if (obstacle_detected) {
    target_vel_.linear = avoidance_linear;
    target_vel_.angular = avoidance_omega;
  } else if (has_target_) {
    double dx = target_x_ - pose_.x;
    double dy = target_y_ - pose_.y;
    double distance = std::hypot(dx, dy);

    if (distance < target_reached_threshold_) {
      has_target_ = false;
      target_vel_.linear = 0.0;
      target_vel_.angular = 0.0;
    } else {
      double target_theta = std::atan2(dy, dx);
      double angle_error = std::atan2(std::sin(target_theta - pose_.theta),
                                      std::cos(target_theta - pose_.theta));

      target_vel_.angular = kp_angular_ * angle_error;
      target_vel_.linear = max_speed_ * std::max(0.2, std::cos(angle_error));
    }
  }

  // 3. Apply acceleration limits
  double dv = target_vel_.linear - current_vel_.linear;
  current_vel_.linear +=
      std::clamp(dv, -max_linear_accel_ * dt, max_linear_accel_ * dt);

  double domega = target_vel_.angular - current_vel_.angular;
  current_vel_.angular +=
      std::clamp(domega, -max_angular_accel_ * dt, max_angular_accel_ * dt);

  // 4. Update Position
  double next_x = pose_.x + current_vel_.linear * std::cos(pose_.theta) * dt;
  double next_y = pose_.y + current_vel_.linear * std::sin(pose_.theta) * dt;

  if (!map.is_collision(next_x, next_y)) {
    pose_.x = next_x;
    pose_.y = next_y;
  } else {
    current_vel_.linear = 0.0;
  }

  pose_.theta += current_vel_.angular * dt;
  pose_.theta = std::atan2(std::sin(pose_.theta), std::cos(pose_.theta));

  // 5. Battery drain
  double speed_factor =
      std::abs(current_vel_.linear) + std::abs(current_vel_.angular) * 0.2;
  battery_level_ =
      std::max(0.0, battery_level_ - (0.05 + 0.1 * speed_factor) * dt);
}
