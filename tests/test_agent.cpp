#include "Agent.hpp"
#include "GridMap.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_initial_state() {
  RobotAgent agent(1, 100.0, 200.0, 50.0);
  Pose pose = agent.get_pose();

  assert(agent.get_id() == 1);
  assert(pose.x == 100.0);
  assert(pose.y == 200.0);
  assert(pose.theta == 0.0);
  assert(agent.get_battery() == 100.0);
  std::cout << "[PASSED] test_initial_state\n";
}

void test_linear_motion_with_map() {
  // Create a 10x10 grid with 10.0 resolution (total world size: 100x100)
  GridMap map(10.0, 10.0, 10.0);

  // Place an obstacle at grid cell (5, 0) -> world x: [50, 60], y: [0, 10]
  map.set_cell(5, 0, CellState::OBSTACLE);

  // Agent starts at x=30.0, y=5.0 moving right
  RobotAgent agent(1, 30.0, 50, 100.0);
  agent.set_target_velocity(10.0, 0.0);

  // Step simulation forward
  agent.step(1.0, map);

  Pose pose = agent.get_pose();
  assert(pose.x > 30.0); // should move forward safely
  std::cout << "[PASSED] test_linear_motion_with_map\n";
}

void test_obstacle_collision_stop() {
  // 10x10 grid, 10.0 resolution
  GridMap map(10, 10, 10.0);

  // Put an obstacle right in front of the robot at x: [40, 50)
  map.set_cell(4, 0, CellState::OBSTACLE);

  // Agent starts at x=35.0, y=5.0 moving right towards the obstacle
  RobotAgent agent(1, 35.0, 5.0, 100.0);
  agent.set_target_velocity(20.0, 0.0);

  // Step simulation enough to hit the obstacle zone
  agent.step(1.0, map);

  Pose pose = agent.get_pose();
  // It should have hit the collision check and stopped before or at the
  // obstacle boundary (40.0)
  assert(pose.x <= 40.0);
  std::cout << "[PASSED] test_obstacle_collision_stop\n";
}

void test_survivor_collection() {
  GridMap map(10, 10, 10.0);

  // Place a survivor at grid cell (2, 2) -> world coordinates (25.0, 25.0)
  map.set_cell(2, 2, CellState::SURVIVOR);

  // Check survivor at exact world coordinate inside cell (2,2)
  bool found_first = map.check_survivor(25.0, 25.0);
  assert(found_first == true);

  // Checking the same spot again should return false because it was cleared
  bool found_second = map.check_survivor(25.0, 25.0);
  assert(found_second == false);

  std::cout << "[PASSED] test_survivor_collection\n";
}

void test_battery_depletion() {
  GridMap map(10.0, 10.0, 10.0);

  RobotAgent agent(1, 0.0, 0.0, 100.0);
  agent.set_target_velocity(20.0, 1.0);

  double initial_battery = agent.get_battery();

  // Run for several steps
  for (int i = 0; i < 10; ++i) {
    agent.step(0.1, map);
  }

  assert(agent.get_battery() < initial_battery);
  std::cout << "[PASSED] test_battery_depletion\n";
}

int main() {
  std::cout << "--- Running SwarmSim Comprehensive Test Suite ---\n";
  test_initial_state();
  test_linear_motion_with_map();
  test_obstacle_collision_stop();
  test_survivor_collection();
  test_battery_depletion();
  std::cout << "All unit tests passed successfully!\n";
  return 0;
}
