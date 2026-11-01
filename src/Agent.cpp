#include "Agent.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

RobotAgent::RobotAgent(uint64_t id, double x, double y,
                       const RobotConfig& config)
    : id_(id), config_(config) {
  pose_.x = x;
  pose_.y = y;
  pose_.theta = 0.0;

  // Reserve space to avoid reallocations during runtime
  known_survivors_.reserve(16);
  inbox_.reserve(32);
  outbox_.reserve(32);
  seen_message_ids_.reserve(64);
}

void RobotAgent::set_target_velocity(double v, double omega) {
  if (std::isnan(v) || std::isnan(omega))
    return;

  has_target_ = false;  // Overide to disable active navigation
  target_vel_.linear = std::clamp(v, -config_.max_speed, config_.max_speed);
  target_vel_.angular =
      std::clamp(omega, -config_.max_angular_vel, config_.max_angular_vel);
}

void RobotAgent::set_navigation_target(double tx, double ty) {
  if (std::isnan(tx) || std::isnan(ty))
    return;
  target_x_ = tx;
  target_y_ = ty;
  has_target_ = true;
}

void RobotAgent::receive_message(const Message& msg) {
  // Check if we've seen this message ID to prevent broadcast storms
  auto it = std::find(seen_message_ids_.begin(), seen_message_ids_.end(),
                      msg.message_id);
  if (it == seen_message_ids_.end() && msg.hop_count < msg.max_hops) {
    // circular buffer logic
    if (seen_message_ids_.size() >= config_.max_seen_message_ids) {
      seen_message_ids_.erase(seen_message_ids_.begin());  // Remove oldest
    }
    seen_message_ids_.push_back(msg.message_id);
    inbox_.push_back(msg);

    // Prepare for multi-hop relaying
    Message relay_msg = msg;
    relay_msg.sender_id = id_;
    relay_msg.hop_count++;
    outbox_.push_back(relay_msg);
  }
}

void RobotAgent::clear_inboxes() {
  inbox_.clear();
  // Keep outbox for mesh propagation(cleared next step externally)
}

void RobotAgent::add_survivor_memory(double x, double y, bool visited,
                                     double current_time) {
  // Throttle message generation to reduce bandwidth
  if (current_time - last_message_time_ < config_.message_generation_interval) {
    return;
  }
  last_message_time_ = current_time;

  // Check for duplicate using cluster radius
  auto it = std::find_if(known_survivors_.begin(), known_survivors_.end(),
                         [this, x, y](const SurvivorRecord& rec) {
                           return std::hypot(rec.x - x, rec.y - y) <
                                  config_.duplicate_cluster_radius;
                         });

  if (it == known_survivors_.end()) {
    known_survivors_.push_back({x, y, visited, current_time});

    // Generate a unique message ID for mesh broadcasting
    uint64_t msg_id = generate_message_id(current_time);
    Message new_msg{id_, id_, msg_id, 0, 5, x, y, true, visited, current_time};
    outbox_.push_back(new_msg);
  } else if (visited && !it->visited) {
    it->visited = true;
    // Broadcast updated visited status
    std::uint64_t msg_id = id_ * 2000 + static_cast<int>(current_time * 10);
    Message update_msg = {id_, id_, msg_id, 0,    5,
                          x,   y,   true,   true, current_time};
    outbox_.push_back(update_msg);
  }
}
// Generate unique id for robot using atomic counter
uint64_t RobotAgent::generate_message_id(double current_time) {
  static uint64_t message_counter{0};
  return (static_cast<uint64_t>(id_) << 48 |      // Top 16 bits: robot ID
          (++message_counter) & 0xFFFFFFFFFFFF);  // Lower 48 bits: sequence
}

void RobotAgent::mark_survivor_visited(double x, double y) {
  for (auto& rec : known_survivors_) {
    if (std::hypot(rec.x - x, rec.y - y) < config_.duplicate_cluster_radius) {
      rec.visited = true;
    }
  }
}

void RobotAgent::prune_expired_memories(double current_time) {
  // Remove unvisited memories that exceed TTL
  known_survivors_.erase(
      std::remove_if(known_survivors_.begin(), known_survivors_.end(),
                     [this, current_time](const SurvivorRecord& rec) {
                       return !rec.visited && (current_time - rec.timestamp >
                                               config_.survivor_ttl);
                     }),
      known_survivors_.end());
}

bool RobotAgent::handle_charging_state(double dt) {
  if (behavior_ == RobotBehavior::RECHARGING) {
    battery_level_ += config_.recharge_rate * dt;
    target_vel_ = {0.0, 0.0};
    current_vel_ = {0.0, 0.0};

    if (battery_level_ >= 100.0) {
      battery_level_ = 100.0;
      behavior_ = RobotBehavior::SEARCHING;
      has_target_ = false;
    }
    return true;  // Handled, skip regular movement
  }

  // Trigger return-to-base if battery is low
  if (battery_level_ < config_.low_battery_threshold &&
      behavior_ == RobotBehavior::SEARCHING) {
    behavior_ = RobotBehavior::RETURNING_TO_BASE;
  }

  // Check if reached base station while returning
  if (behavior_ == RobotBehavior::RETURNING_TO_BASE) {
    set_navigation_target(config_.base_x, config_.base_y);
    double dist_to_base = std::hypot(config_.base_x, config_.base_y);

    if (dist_to_base < std::max(30.0, config_.target_reached_threshold)) {
      behavior_ = RobotBehavior::RECHARGING;
      target_vel_ = {0.0, 0.0};
      current_vel_ = {0.0, 0.0};
      return true;
    }
  }

  // stop robot if battery is exhausted
  if (battery_level_ <= 0.0) {
    target_vel_ = {0.0, 0.0};
    current_vel_ = {0.0, 0.0};
    return true;
  }
  return false;  // Continue normal simulation
}

void RobotAgent::compute_navigation_velocity() {
  if (!has_target_)
    return;

  double dx = target_x_ - pose_.x;
  double dy = target_y_ - pose_.y;
  double distance = std::hypot(dx, dy);

  if (distance < config_.target_reached_threshold) {
    has_target_ = false;
    target_vel_.linear = 0.0;
    target_vel_.angular = 0.0;

  } else {
    double target_theta = std::atan2(dy, dx);
    double angle_error = std::atan2(std::sin(target_theta - pose_.theta),
                                    std::cos(target_theta - pose_.theta));

    double desired_omega = config_.kp_angular * angle_error;
    double desired_linear =
        config_.max_speed * std::max(0.2, std::cos(angle_error));

    if (distance < 80.0) {
      double braking_factor = std::clamp(distance / 80.0, 0.15, 1.0);
      desired_linear *= braking_factor;
    }

    target_vel_.linear = desired_linear;
    target_vel_.angular = std::clamp(desired_omega, -config_.max_angular_vel,
                                     config_.max_angular_vel);
  }
}

void RobotAgent::apply_obstacle_avoidance(const GridMap& map) {
  double lookahead_x =
      pose_.x + config_.lookahead_distance * std::cos(pose_.theta);
  double lookahead_y =
      pose_.y + config_.lookahead_distance * std::sin(pose_.theta);

  if (map.is_collision(lookahead_x, lookahead_y)) {
    double left_angle = pose_.theta + 0.785;   // +45 degrees
    double right_angle = pose_.theta - 0.785;  // -45 degrees

    bool left_clear = !map.is_collision(
        pose_.x + (config_.lookahead_distance * 0.7) * std::cos(left_angle),
        pose_.y + (config_.lookahead_distance * 0.7) * std::sin(left_angle));

    double right_clear = !map.is_collision(
        pose_.x + (config_.lookahead_distance * 0.7) * std::cos(right_angle),
        pose_.y + (config_.lookahead_distance * 0.7) * std::sin(right_angle));

    if (left_clear && !right_clear) {
      target_vel_.linear =
          std::max(15.0, config_.max_speed * 0.4);  // Turn left safely
      target_vel_.angular = 0.9;                    // slow down slightly
    } else if (right_clear && !left_clear) {
      target_vel_.linear =
          std::max(15.0, config_.max_speed * 0.4);  // Turn right safely
      target_vel_.angular = -0.9;
    } else if (!left_clear && !right_clear) {
      target_vel_.linear = -config_.max_speed * 0.5;
      target_vel_.angular = 0.0;  // Don't spin
    } else {
      // both clear
      target_vel_.angular = config_.max_speed * 0.35;
      target_vel_.angular = 0.8;
    }
  }
}

void RobotAgent::apply_kinematics(double dt, const GridMap& map) {
  double dv = target_vel_.linear - current_vel_.linear;
  current_vel_.linear += std::clamp(dv, -config_.max_linear_accel * dt,
                                    config_.max_linear_accel * dt);

  double domega = target_vel_.angular - current_vel_.angular;
  current_vel_.angular += std::clamp(domega, -config_.max_angular_accel * dt,
                                     config_.max_angular_accel * dt);

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

  // Normalize theat to [-π, π]
  while (pose_.theta > M_PI)
    pose_.theta -= 2 * M_PI;
  while (pose_.theta < -M_PI)
    pose_.theta += 2 * M_PI;
}

void RobotAgent::update_battery_drain(double dt) {
  double speed_factor =
      std::abs(current_vel_.linear) + std::abs(current_vel_.angular) * 0.2;
  if (speed_factor > 0.0) {
    battery_level_ -= (0.1 * speed_factor) * dt;
  }
  battery_level_ = std::max(0.0, battery_level_);
}

void RobotAgent::step(double dt, double current_time, const GridMap& map) {
  // Prune stale unvisited memories periodically
  prune_expired_memories(current_time);

  // 0. Base Station Charging & Battery State Machine
  if (handle_charging_state(dt)) {
    return;
  }

  // 1. Navigation & Proportional Steering Contorl
  compute_navigation_velocity();
  // 2. Proactive Obstacle Avoidance(Raycasting lookahead)
  apply_obstacle_avoidance(map);
  // 3. Apply acceleration limits
  apply_kinematics(dt, map);

  // 5. Battery drain
  update_battery_drain(dt);
}
