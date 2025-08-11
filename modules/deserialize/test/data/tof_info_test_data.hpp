#include <array>
#include <cstdint>
#include <string>

namespace ifm3d::tof_info
{
  constexpr uint32_t VERSION = 4;
  constexpr float DISTANCE_RESOLUTION = 0.00015259;
  constexpr float AMPLITUDE_RESOLUTION = 2.32989e-08;
  constexpr std::array<float, 3> AMP_NORMALIZATION_FACTORS = {
    0.0500309,
    0.0025,
    0.000200025,
  };
  constexpr std::array<float, 6> EXTRINCSIC_OPTIC_TO_USER =
    {0, 0, 0.00841006, -0.0204563, 0.0181109, -0.00622652};
  constexpr uint32_t INTRINSIC_CALIB_MODEL_ID = {0};
  constexpr std::array<float, 32> INTRINSIC_CALIB_MODEL_PARAM = {
    128.5714264,
    128.5714264,
    112.7824249,
    83.43323517,
    0,
    0.5029289722,
    -0.4033640027,
    0,
    0,
    0.6367549896,
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
  constexpr uint32_t INVERSE_INTRINSIC_CALIB_MODEL_ID = {1};
  constexpr std::array<float, 32> INVERSE_INTRINSIC_CALIB_MODEL_PARAM = {
    128.5714264,
    128.5714264,
    112.7824249,
    83.43323517,
    0,
    -0.2719089985,
    0.05658800155,
    0,
    0,
    -0.004511999898,
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
  constexpr std::array<uint64_t, 3> EXPOSURE_TIMESTAMPS_NS = {
    1581587830148502150,
    1581587830144571155,
    1581587830131172393,
  };
  constexpr std::array<float, 3> EXPOSURE_TIMES_S = {
    0.0000199877,
    0.0004,
    0.00499938,
  };

  constexpr float ILLU_TEMPERATURE = 58.336933135986328;

  inline const std::string MODE = "standard_range4m";
  inline const std::string IMAGER = "IRS2381C";
  constexpr std::uint32_t MEASUREMENT_BLOCK_INDEX = 0;
  constexpr float MEASUREMENT_RANGE_MIN = 0;
  constexpr float MEASUREMENT_RANGE_MAX = 4.143615246;
}
