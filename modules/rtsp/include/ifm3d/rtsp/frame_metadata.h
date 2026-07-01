/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_FRAME_METADATA_H
#define IFM3D_RTSP_FRAME_METADATA_H

#include <array>
#include <cstdint>

#include <ifm3d/rtsp/module_rtsp.h>

namespace ifm3d
{
  /** @ingroup RTSP
   *
   * @brief Metadata transported in an H.264 SEI "unregistered user data"
   * message (ITU-T H.264 §D.1, payloadType 5).
   *
   * O3C devices embed a `RGB_INFO` structure (ticket O3C-6884) in every coded
   * video frame. When the `RtspClient` decodes a frame it parses this SEI
   * payload and attaches it to the delivered `ifm3d::Buffer` as JSON metadata
   * (see the `frame_metadata` keys below), so applications can correlate the
   * decoded image with the device's frame counter, timestamp and calibration.
   *
   * Wire layout (all little-endian, IEEE-754 floats), 308 bytes total:
   *
   *     Offset  Field                    Type             Size
   *     0x0000  version                  uint32           4
   *     0x0004  frame_counter            uint32           4
   *     0x0008  timestamp_ns             uint64           8
   *     0x0010  exposure_time            float32          4
   *     0x0014  extrinsic_optic_to_user  float32[6]       24
   *     0x002C  intrinsic_calibration    uint32 + f32[32] 132
   *     0x00B0  inverse_intrinsic_calib  uint32 + f32[32] 132
   */
  struct IFM3D_EXPORT RgbInfo
  {
    /** Number of float parameters in an intrinsic calibration model. */
    static constexpr std::size_t INTRINSIC_PARAM_COUNT = 32;

    /**
     * Serialized size of the RGB_INFO field block, in bytes (the SEI
     * user-data payload excluding the leading 16-byte UUID).
     */
    static constexpr std::size_t WIRE_SIZE = 308;

    /**
     * 16-byte RFC-4122 UUID identifying the RGB_INFO SEI payload
     * (UUIDv5 of name "RGB_INFO"): e6ccb4b0-0071-580e-828d-928c60078143.
     */
    static constexpr std::array<std::uint8_t, 16> UUID = {0xe6,
                                                          0xcc,
                                                          0xb4,
                                                          0xb0,
                                                          0x00,
                                                          0x71,
                                                          0x58,
                                                          0x0e,
                                                          0x82,
                                                          0x8d,
                                                          0x92,
                                                          0x8c,
                                                          0x60,
                                                          0x07,
                                                          0x81,
                                                          0x43};

    /** Intrinsic / inverse-intrinsic calibration model. */
    struct Intrinsic
    {
      std::uint32_t model_id = 0;
      std::array<float, INTRINSIC_PARAM_COUNT> model_parameters{};
    };

    std::uint32_t version = 0;
    std::uint32_t frame_counter = 0;
    std::uint64_t timestamp_ns = 0;
    float exposure_time = 0.0F;

    /** transX, transY, transZ [m], rotX, rotY, rotZ [rad]. */
    std::array<float, 6> extrinsic_optic_to_user{};

    Intrinsic intrinsic;
    Intrinsic inverse_intrinsic;
  };

  /** @ingroup RTSP
   *
   * JSON key names under which RGB_INFO fields are stored in the
   * `ifm3d::Buffer` metadata of decoded RTSP frames.
   */
  namespace frame_metadata
  {
    constexpr const char* VERSION = "version";
    constexpr const char* FRAME_COUNTER = "frame_counter";
    constexpr const char* TIMESTAMP_NS = "timestamp_ns";
    constexpr const char* EXPOSURE_TIME = "exposure_time";
    constexpr const char* EXTRINSIC_OPTIC_TO_USER = "extrinsic_optic_to_user";
    constexpr const char* INTRINSIC_MODEL_ID = "intrinsic_model_id";
    constexpr const char* INTRINSIC_PARAMETERS = "intrinsic_parameters";
    constexpr const char* INVERSE_INTRINSIC_MODEL_ID =
      "inverse_intrinsic_model_id";
    constexpr const char* INVERSE_INTRINSIC_PARAMETERS =
      "inverse_intrinsic_parameters";
  } // namespace frame_metadata

} // namespace ifm3d

#endif // IFM3D_RTSP_FRAME_METADATA_H
