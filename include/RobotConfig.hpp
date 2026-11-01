#pragma once
#include <cmath>

struct RobotConfig {
  // Movement constraints
  double max_speed{130.0};
  double max_angular_vel{2.5};
  double max_linear_accel{1.5};
  double max_angular_accel{3.14};

  // Navigation thresholds
  double target_reached_threshold{15.0};
  double lookahead_distance{20.0};
  double kp_angular{2.5};

  // Survivor tracking
  double survivor_ttl{120.0};  // Time-to-live in seconds
  double duplicate_cluster_radius{25.0};

  // Base station
  double base_x{100.0};
  double base_y{100.0};

  // Battery management
  double low_battery_threshold{25.0};  //%
  double recharge_rate{35.0};          //% per second
  double battery_drain_idle{0.05};

  // Message & Communication
  int max_seen_message_ids{1000};           // circular buffer size
  double message_generation_interval{0.5};  // Min seconds between broadcasts
};
