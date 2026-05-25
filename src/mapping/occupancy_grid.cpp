#include "mapping/occupancy_grid.hpp"

#include <stdexcept>

namespace toy_rover::mapping
{

  OccupancyGrid::OccupancyGrid(int width, int height, double resolution_m)
      : width_(width),
        height_(height),
        resolution_m_(resolution_m),
        cells_(static_cast<std::size_t>(width * height), Cell::Unknown)
  {
    if (width_ <= 0 || height_ <= 0)
    {
      throw std::invalid_argument("grid dimensions must be positive");
    }
    if (resolution_m_ <= 0.0)
    {
      throw std::invalid_argument("grid resolution must be positive");
    }
  }

  int OccupancyGrid::width() const noexcept { return width_; }

  int OccupancyGrid::height() const noexcept { return height_; }

  double OccupancyGrid::resolution_m() const noexcept { return resolution_m_; }

  bool OccupancyGrid::in_bounds(GridIndex index) const noexcept
  {
    return index.x >= 0 && index.y >= 0 && index.x < width_ && index.y < height_;
  }

  Cell OccupancyGrid::at(GridIndex index) const
  {
    if (!in_bounds(index))
    {
      throw std::out_of_range("grid index out of bounds");
    }
    return cells_[linear_index(index)];
  }

  void OccupancyGrid::set(GridIndex index, Cell value)
  {
    if (!in_bounds(index))
    {
      throw std::out_of_range("grid index out of bounds");
    }
    cells_[linear_index(index)] = value;
  }

  std::size_t OccupancyGrid::linear_index(GridIndex index) const
  {
    return static_cast<std::size_t>(index.y * width_ + index.x);
  }

  const std::vector<Cell> &OccupancyGrid::cells() const noexcept
  {
    return cells_;
  }

} // namespace toy_rover::mapping
