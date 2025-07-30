#include <array>
#include <cstdint>

namespace ifm3d
{
  namespace ods_extrinsic_calibration_correction
  {
    constexpr uint32_t version = 1;
    constexpr float completion_rate = 0.0;
    constexpr std::array<float, 3> rot_delta_value = {0.0, 0.0, 0.0};
    constexpr std::array<uint8_t, 3> rot_delta_valid = {0, 0, 0};
    constexpr std::array<float, 3> rot_head_to_user = {0.0, 0.0, 0.0};
  }
}