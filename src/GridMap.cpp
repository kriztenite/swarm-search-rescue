#include "GridMap.hpp"
#include <algorithm>
#include <cmath>

GridMap::GridMap(int width, int height, double resolution)
    : width_(width), height_(height), resolution_(resolution) {
  // Initialize entire map as Free space
  grid_.resize(width_ * height_, CellState::FREE);
}

int GridMap::index(int x, int y) const { return y * width_ + x; }

void GridMap::set_cell(int x, int y, CellState state) {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    grid_[index(x, y)] = state;
  }
}

CellState GridMap::get_cell(int x, int y) const {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    return grid_[index(x, y)];
  }
  // Out of bounds is treated as an obstacle/wall boundary
  return CellState::OBSTACLE;
}

bool GridMap::is_collision(double world_x, double world_y) const {
  // Convert continuous world cordinalte to discrete grid indices
  int gx = static_cast<int>(world_x / resolution_);
  int gy = static_cast<int>(world_y / resolution_);

  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
    return true; // out of bounds is a collision
  }

  return grid_[index(gx, gy)] == CellState::OBSTACLE;
}

bool GridMap::check_survivor(double world_x, double world_y) {
  int gx = static_cast<int>(world_x / resolution_);
  int gy = static_cast<int>(world_y / resolution_);

  if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
    if (grid_[index(gx, gy)] == CellState::SURVIVOR) {
      // Survivor found! clear the cell to Free space so it's only found once
      grid_[index(gx, gy)] = CellState::FREE;
      return true;
    }
  }
  return false;
}
