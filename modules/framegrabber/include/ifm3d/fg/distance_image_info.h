#ifndef IFM3D_DISTANCE_IMAGE_INFO_H
#define IFM3D_DISTANCE_IMAGE_INFO_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <memory>

namespace ifm3d
{
    constexpr auto CHUNK_SIZE_INFO_OFFSET = 4;
    constexpr auto HEADER_SIZE_INFO_OFFSET = 8;
    constexpr auto HEADER_VERSION_INFO_OFFSET = 12;

    struct IntrinsicCalibration
    {
      uint32_t model_iD;
      float model_parameters[32];
    };
/*
    std::vector<std::uint8_t> convertAmplitude(
        const uint16_t u16Amplitude[], 
        float amplitudeResolution,
        std::uint32_t width, std::uint32_t height);

    std::vector<std::uint8_t> calculateXYZDFromDistanceImageInfo(
        const float extrinsic_optic_to_user[],
        const IntrinsicCalibration &intrinsicCalibration,
        const std::vector<std::uint16_t> &distanceBuffer,
        float distResolution,
        std::uint32_t width, std::uint32_t height);*/

    class DistanceImageInfo
    {
      const float dist_resolution;
      const float ampl_resolution;
      const std::vector<float> amp_norm_factors;
      const std::vector<float> extrinsic_optic_to_user;
      const IntrinsicCalibration intrinsic_calibration;
      const IntrinsicCalibration inverse_intrinsic_calibration;
      const std::uint32_t width, height;
      const std::vector<std::uint16_t> u16_distance_buffer;
      const std::vector<std::uint16_t> u16_amplitude_buffer;

      public:
        DistanceImageInfo(
          const float dist_res,
          const float ampl_res,
          const std::vector<float> &amp_norm_fctrs,
          const std::vector<float> &extr_opt_to_usr,
          const IntrinsicCalibration &intr_calib,
          const IntrinsicCalibration &inv_intr_calib,
          const std::vector<std::uint16_t> &distance_buffer,
          const std::vector<std::uint16_t> &amplitude_buffer,
          const std::uint32_t width, const std::uint32_t height);
        ~DistanceImageInfo() = default;
        std::vector<std::uint8_t> getXYZDVector();
        std::vector<std::uint8_t> getAmplitudeVector();
    };
    std::unique_ptr<DistanceImageInfo> CreateDistanceImageInfo(const std::vector<std::uint8_t> &data_buffer,
                                                               const std::size_t didx, const std::size_t aidx,
                                                               const std::uint32_t width, const std::uint32_t height);
} // end: namespace ifm3d

#endif // IFM3D_DISTANCE_IMAGE_INFO_H
