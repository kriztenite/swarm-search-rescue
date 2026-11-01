#include "CommsManager.hpp"

#include <cmath>

CommsManager::CommsManager(double comms_range) : comms_range_(comms_range) {}

void CommsManager::communicate(std::vector<RobotAgent>& agents) {
  // clear inboxes
  for (auto& agent : agents) {
    agent.clear_inboxes();
  }

  const double comms_range_sq = comms_range_ * comms_range_;

  // Multi-hop mesh exchange between neighbors within comms range
  for (size_t i{0}; i < agents.size(); i++) {
    const Pose& pose_i = agents[i].get_pose();
    const auto& outbox_i = agents[i].get_outbox();

    if (outbox_i.empty())
      continue;

    for (size_t j{0}; j < agents.size(); j++) {
      const Pose& pose_j = agents[j].get_pose();

      // Calculate euclidean distance between robot
      auto dx = pose_j.x - pose_i.x;
      if (std::abs(dx) > comms_range_)
        continue;
      auto dy = pose_j.y - pose_i.y;
      if (std::abs(dy) > comms_range_)
        continue;
      double distance = (dx * dx) + (dy * dy);

      if (distance <= comms_range_) {
        // Relay robots i's outbox messages to robot
        for (const auto& msg : outbox_i) {
          agents[j].receive_message(msg);
        }

        // Relay robot j's outbox messages to robot i
        for (const auto& msg : agents[j].get_outbox()) {
          agents[i].receive_message(msg);
        }
      }
    }
  }
}
