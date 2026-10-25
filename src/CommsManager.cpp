#include "CommsManager.hpp"

#include <cmath>

CommsManager::CommsManager(double comms_range) : comms_range_(comms_range) {}

void CommsManager::communicate(std::vector<RobotAgent>& agents) {
  for (auto& agent : agents) {
    agent.clear_inboxes();
  }

  // Multi-hop mesh exchange between neighbors within comms range
  for (size_t i{0}; i < agents.size(); i++) {
    for (size_t j{i + 1}; j < agents.size(); ++j) {
      Pose pose_i = agents[i].get_pose();
      Pose pose_j = agents[j].get_pose();

      // Calculate euclidean distance between robot
      double distance = std::hypot(pose_j.x - pose_i.x, pose_j.y - pose_i.y);

      if (distance <= comms_range_) {
        // Relay robots i's outbox messages to robot
        for (const auto& msg : agents[i].get_outbox()) {
          agents[j].receive_message(msg);
        }
      }

      // Relay robot j's outbox messages to robot i
      for (const auto& msg : agents[j].get_outbox()) {
        agents[i].receive_message(msg);
      }
    }
  }
}
