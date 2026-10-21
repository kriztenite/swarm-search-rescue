#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "Agent.hpp"
#include "GridMap.hpp"

class MetricsLogger {
 public:
  explicit MetricsLogger(const std::string& filename);
  ~MetricsLogger();

  void logs_step(double time, const std::vector<RobotAgent>& agents,
                 const GridMap& map, int total_survivors);

 private:
  std::ofstream csv_file_;
};
