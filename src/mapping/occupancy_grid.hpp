#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace toy_rover::mapping
{

  enum class Cell : std::int8_t
  {
    Unknown = -1,
    Free = 0,
    Occupied = 100,
  };

  struct GridIndex
  {
    int x{0};
    int y{0};

    friend bool operator==(const GridIndex &lhs, const GridIndex &rhs)
    {
      return lhs.x == rhs.x && lhs.y == rhs.y;
    }
  };

  class OccupancyGrid
  {
  public:
    OccupancyGrid(int width, int height, double resolution_m);

    [[nodiscard]] int width() const noexcept;
    [[nodiscard]] int height() const noexcept;
    [[nodiscard]] double resolution_m() const noexcept;
    [[nodiscard]] bool in_bounds(GridIndex index) const noexcept;

    [[nodiscard]] Cell at(GridIndex index) const;
    void set(GridIndex index, Cell value);

    [[nodiscard]] std::size_t linear_index(GridIndex index) const;
    [[nodiscard]] const std::vector<Cell> &cells() const noexcept;

  private:
    int width_;
    int height_;
    double resolution_m_;
    std::vector<Cell> cells_;
  };

} // namespace toy_rover::mapping
