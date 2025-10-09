#include <array>
#include <cstdint>

namespace ifm3d::ods_extrinsic_calibration_correction
{
  constexpr uint32_t VERSION = 1;
  constexpr float COMPLETION_RATE = 0.0;
  constexpr std::array<float, 3> ROT_DELTA_VALUE = {0.0, 0.0, 0.0};
  constexpr std::array<uint8_t, 3> ROT_DELTA_VALID = {0, 0, 0};
  constexpr std::array<float, 3> ROT_HEAD_TO_USER = {0.0, 0.0, 0.0};
}
