
#include <vector>

#include "Agent.hpp"

class CommsManager {
 public:
  explicit CommsManager(double comms_range);

  // Broadcast message between all agents within communication range
  void communicate(std::vector<RobotAgent>& agents);

  // Getter
  double get_comms_range() const { return comms_range_; }
  void set_comms_range(double range) { comms_range_ = range; }

 private:
  double comms_range_;
};
