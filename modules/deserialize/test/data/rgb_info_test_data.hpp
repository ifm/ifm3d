#include <array>

namespace ifm3d
{
  namespace rgb_info
  {
    constexpr uint32_t version = 1;
    constexpr uint32_t frame_counter = 8174615;
    constexpr uint64_t timestamp_ns = 1581500914182583358;
    constexpr float exposure_time = 0.009999998845;

    constexpr std::array<float, 6> extrincsic_optic_to_user = {0.01400010101,
                                                               0,
                                                               0.01738012396,
                                                               -0.0205945354,
                                                               -0.002193912864,
                                                               0.009186834097};
    constexpr uint32_t intrinsic_calib_model_id = 0;
    std::array<float, 32> intrinsic_calib_model_param = {577.0866699,
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
    constexpr uint32_t inverse_intrinsic_calib_model_id = 0;
    std::array<float, 32> inverse_intrinsic_calib_model_param = {
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
}