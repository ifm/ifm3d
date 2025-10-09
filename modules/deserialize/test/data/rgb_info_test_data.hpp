#include <array>
#include <cstdint>

namespace ifm3d::rgb_info
{
  constexpr uint32_t VERSION = 1;
  constexpr uint32_t FRAME_COUNTER = 8174615;
  constexpr uint64_t TIMESTAMP_NS = 1581500914182583358;
  constexpr float EXPOSURE_TIME = 0.009999998845;

  constexpr std::array<float, 6> EXTRINCSIC_OPTIC_TO_USER = {0.01400010101,
                                                             0,
                                                             0.01738012396,
                                                             -0.0205945354,
                                                             -0.002193912864,
                                                             0.009186834097};
  constexpr uint32_t INTRINSIC_CALIB_MODEL_ID = 0;
  inline constexpr std::array<float, 32> INTRINSIC_CALIB_MODEL_PARAM = {
    577.0866699,
    577.0866699,
    618.0302124,
    390.7052002,
    0,
    0.7169150114,
    -0.7727349997,
    0,
    0,
    0.5817289948,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0};
  constexpr uint32_t INVERSE_INTRINSIC_CALIB_MODEL_ID = 0;
  constexpr std::array<float, 32> INVERSE_INTRINSIC_CALIB_MODEL_PARAM = {
    577.0866699,
    577.0866699,
    618.0302124,
    390.7052002,
    0,
    -0.1384820044,
    0.00955399964,
    0,
    0,
    -0.0002139999997,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
  };
}
