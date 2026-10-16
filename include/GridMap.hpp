#pragma once

#include <vector>

enum class CellState {
  FREE = 0,
  OBSTACLE = 1,
  SURVIVOR = 2,
};

class GridMap {
public:
  GridMap(int width, int height, double resolution);

  // Set a cell's state
  void set_cell(int x, int y, CellState state);

  // Get a cell's state safely (returns OBSTACLE if out of bounds)
  CellState get_cell(int x, int y) const;

  // check if a continous(x,y) world cordinate collides with an obstacle
  bool is_collision(double world_x, double world_y) const;

  // Check if a surviror is found at cordinates and clear it if collected
  bool check_survivor(double world_x, double world_y);

  // Getters
  int get_width() const { return width_; }
  int get_height() const { return height_; }
  double get_resolution() const { return resolution_; }

private:
  int width_;         // Number of column in grid
  int height_;        // Number of row in grid
  double resolution_; // Size of each cell in world units (e.g., meters)

  // 2D grid stored as a flat vector for performance
  std::vector<CellState> grid_;

  // Helper to convert 2D grid index
  int index(int x, int y) const;
};
