#include <array>
#include <cstdint>

namespace ifm3d::ods_occupancy_grid
{
  constexpr uint64_t TIMESTAMP_NS = 1581093211432340979;
  constexpr uint32_t WIDTH = 200;
  constexpr uint32_t HEIGHT = 200;
  constexpr std::array<float, 6> TRANSFORM_CELL_CENTER_TO_USER =
    {0.05000000075, 0, -4.974999905, 0, 0.05000000075, -4.974999905};
}
