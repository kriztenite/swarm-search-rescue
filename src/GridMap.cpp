#include "GridMap.hpp"

#include <algorithm>
#include <cmath>

GridMap::GridMap(int width, int height, double resolution)
    : width_(width), height_(height), resolution_(resolution) {
  // Initialize entire map as UNKNOWN
  grid_.resize(height_, std::vector<CellState>(width_, CellState::UNKNOWN));
}

// int GridMap::index(int x, int y) const {
//   return y * width_ + x;
// }

void GridMap::set_cell(int x, int y, CellState state) {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    grid_[y][x] = state;
  }
}

CellState GridMap::get_cell(int x, int y) const {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    return grid_[y][x];
  }
  // Out of bounds is treated as an obstacle/wall boundary
  return CellState::OBSTACLE;
}

bool GridMap::is_collision(double world_x, double world_y) const {
  // Convert continuous world cordinalte to discrete grid indices
  int gx = static_cast<int>(world_x / resolution_);
  int gy = static_cast<int>(world_y / resolution_);

  CellState state = get_cell(gx, gy);
  return (state == CellState::OBSTACLE);  // out of bounds is a collision
}

bool GridMap::check_survivor(double world_x, double world_y) {
  int gx = static_cast<int>(world_x / resolution_);
  int gy = static_cast<int>(world_y / resolution_);

  if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
    if (get_cell(gx, gy) == CellState::SURVIVOR) {
      // Survivor found! clear the cell to Free space so it's only found once
      set_cell(gx, gy, CellState::FREE);
      return true;
    }
  }
  return false;
}
void GridMap::reveal_radius(double world_x, double world_y,
                            double sensor_range) {
  int center_gx = static_cast<int>(world_x / resolution_);
  int center_gy = static_cast<int>(world_y / resolution_);
  int radius_cells = static_cast<int>(sensor_range / resolution_);

  for (int gy = center_gy - radius_cells; gy <= center_gy + radius_cells;
       ++gy) {
    for (int gx = center_gx - radius_cells; gx <= center_gx + radius_cells;
         ++gx) {
      if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
        // Check if within circular sensor radius
        double wx = (gx + 0.5) * resolution_;
        double wy = (gy + 0.5) * resolution_;
        if (std::hypot(wx - world_x, wy - world_y)) {
          // If the cell was UNKNOWN and not a wall/survivor by default
          // reveal as FREE
          if (grid_[gy][gx] == CellState::UNKNOWN) {
            grid_[gy][gx] = CellState::FREE;
          }
        }
      }
    }
  }
}
std::vector<std::pair<int, int>> GridMap::get_frontiers() const {
  std::vector<std::pair<int, int>> frontiers;

  // 4-connected neighbors offset
  const int dx[] = {0, 0, 1, -1};
  const int dy[] = {1, -1, 0, 0};

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      // A frontier cell must be FREE
      if (grid_[y][x] == CellState::FREE) {
        bool is_frontier = false;
        // Check if any neighbor is UNKNOWN
        for (int i = 0; i < 4; ++i) {
          int nx = x + dx[i];
          int ny = y + dy[i];
          if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
            if (grid_[ny][nx] == CellState::UNKNOWN) {
              is_frontier = true;
              break;
            }
          }
        }
        if (is_frontier) {
          frontiers.push_back({x, y});
        }
      }
    }
  }
  return frontiers;
}
