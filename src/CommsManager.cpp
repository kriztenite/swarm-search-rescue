#include "CommsManager.hpp"

#include <cmath>

CommsManager::CommsManager(double comms_range) : comms_range_(comms_range) {}

void CommsManager::communicate(std::vector<RobotAgent>& agents) {
  // Check every pair of robots for line-of-sight / radio proximity
  for (size_t i{0}; i < agents.size(); i++) {
    for (size_t j{i + 1}; j < agents.size(); ++j) {
      Pose pose_i = agents[i].get_pose();
      Pose pose_j = agents[j].get_pose();

      // Calculate Euclidean distance between robot i and robot j
      auto dx = pose_i.x - pose_i.x;
      auto dy = pose_j.y - pose_j.x;
      auto distance = std::sqrt(dx * dx + dy * dy);

      // If within communicate range, they can exchange information
      // Share robot i's memories with robot j
      for (const auto& surv : agents[i].get_known_survivors()) {
        Message msg{agents[i].get_id(), surv.first, surv.second, true};
        agents[j].receive_message(msg);
      }

      // Share robot j's memories with robot i
      for (const auto& surv : agents[j].get_known_survivors()) {
        Message msg{agents[j].get_id(), surv.first, surv.second, true};
        agents[i].receive_message(msg);
      }
    }
  }
}
