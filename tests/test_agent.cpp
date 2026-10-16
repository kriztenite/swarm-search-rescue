#include <iostream>
#include <cassert>
#include <cmath>
#include "Agent.hpp"

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

void test_linear_motion() {
    RobotAgent agent(1, 0.0, 0.0, 100.0);
    
    // Set target velocity forward
    agent.set_target_velocity(10.0, 0.0);

    // Step simulation forward by 1.0 second
    agent.step(1.0);

    Pose pose = agent.get_pose();
    // Due to acceleration limits, it ramps up smoothly so x should be > 0
    assert(pose.x > 0.0);
    assert(pose.y == 0.0);
    std::cout << "[PASSED] test_linear_motion\n";
}

void test_battery_depletion() {
    RobotAgent agent(1, 0.0, 0.0, 100.0);
    agent.set_target_velocity(20.0, 1.0);

    double initial_battery = agent.get_battery();
    
    // Run for several steps
    for (int i = 0; i < 10; ++i) {
        agent.step(0.1);
    }

    assert(agent.get_battery() < initial_battery);
    std::cout << "[PASSED] test_battery_depletion\n";
}

int main() {
    std::cout << "--- Running RobotAgent Standard C++ Test Suite ---\n";
    test_initial_state();
    test_linear_motion();
    test_battery_depletion();
    std::cout << "All unit tests passed successfully!\n";
    return 0;
}
