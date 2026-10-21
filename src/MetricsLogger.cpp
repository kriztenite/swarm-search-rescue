#include "MetricsLogger.hpp"

#include <iostream>

MetricsLogger::MetricsLogger(const std::string& filename) {
  csv_file_.open(filename);
  if (csv_file_.is_open())
    // Write CSV Header
    csv_file_ << "Time, RescuedSurvivors, AvgBattery, ExploredPercentage\n";
}

MetricsLogger::~MetricsLogger() {
  if (csv_file_.is_open()) {
    csv_file_.close();
  }
}

void MetricsLogger::logs_step(double time,
                              const std::vector<RobotAgent>& agents,
                              const GridMap& map, int total_survivors) {
  if (!csv_file_.is_open())
    return;

  double total_battery = 0.0;
  for (const auto& agent : agents)
    total_battery += agent.get_battery();

  double avg_battery = agents.empty() ? 0.0 : total_battery / agents.size();

  // Calculate exploration percentage
  int total_cells = map.get_width() * map.get_height();
  int revealed_cells = 0;
  int remaining_survivors = total_survivors;  // Simplified tracking for demo

  for (int y = 0; y < map.get_height(); ++y) {
    for (int x = 0; x < map.get_width(); ++x) {
      if (map.get_cell(x, y) != CellState::UNKNOWN)
        revealed_cells++;
    }
  }

  double explored_pct =
      (static_cast<double>(revealed_cells) / total_cells) * 100.0;

  // Write row
  csv_file_ << time << "," << (total_survivors - remaining_survivors) << ","
            << avg_battery << explored_pct << "\n";
}
