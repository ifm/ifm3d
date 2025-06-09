/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_FG_Buffer_ID_H
#define IFM3D_FG_Buffer_ID_H

#include <cstdint>
#include <ifm3d/device/device.h>

namespace ifm3d
{
  /** @ingroup FrameGrabber
   *
   * buffer_ids available for use with the default Organizer.
   */
  // NOLINTNEXTLINE(readability-identifier-naming,readability-enum-initial-value)
  enum class buffer_id : uint64_t
  {
    // clang-format off
    
    /** @hideinitializer 
     * @brief \c
    */
    RADIAL_DISTANCE_IMAGE = static_cast<uint64_t>(ifm3d::ImageChunk::RADIAL_DISTANCE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    NORM_AMPLITUDE_IMAGE = static_cast<uint64_t>(ifm3d::ImageChunk::NORM_AMPLITUDE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    AMPLITUDE_IMAGE = static_cast<uint64_t>(ifm3d::ImageChunk::AMPLITUDE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    GRAYSCALE_IMAGE = static_cast<uint64_t>(ifm3d::ImageChunk::GRAYSCALE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    RADIAL_DISTANCE_NOISE = static_cast<uint64_t>(ifm3d::ImageChunk::RADIAL_DISTANCE_NOISE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    REFLECTIVITY = static_cast<uint64_t>(ifm3d::ImageChunk::REFLECTIVITY), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_X_COMPONENT = static_cast<uint64_t>(ifm3d::ImageChunk::CARTESIAN_X_COMPONENT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_Y_COMPONENT = static_cast<uint64_t>(ifm3d::ImageChunk::CARTESIAN_Y_COMPONENT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_Z_COMPONENT = static_cast<uint64_t>(ifm3d::ImageChunk::CARTESIAN_Z_COMPONENT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_ALL = static_cast<uint64_t>(ifm3d::ImageChunk::CARTESIAN_ALL), 
    
    /** @hideinitializer 
     * @brief \c
    */
    UNIT_VECTOR_ALL = static_cast<uint64_t>(ifm3d::ImageChunk::UNIT_VECTOR_ALL), 
    
    /** @hideinitializer  
     * @brief \c
    */
    MONOCHROM_2D_12BIT = static_cast<uint64_t>(ifm3d::ImageChunk::MONOCHROM_2D_12BIT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    MONOCHROM_2D = static_cast<uint64_t>(ifm3d::ImageChunk::MONOCHROM_2D), 
    
    /** @hideinitializer 
     * @brief \c
    */
    JPEG_IMAGE = static_cast<uint64_t>(ifm3d::ImageChunk::JPEG_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CONFIDENCE_IMAGE = static_cast<uint64_t>(ifm3d::ImageChunk::CONFIDENCE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    DIAGNOSTIC = static_cast<uint64_t>(ifm3d::ImageChunk::DIAGNOSTIC), 
    
    /** @hideinitializer 
     * @brief \c
    */
    JSON_DIAGNOSTIC = static_cast<uint64_t>(ifm3d::ImageChunk::JSON_DIAGNOSTIC), 
    
    /** @hideinitializer 
     * @brief \c
    */
    EXTRINSIC_CALIB = static_cast<uint64_t>(ifm3d::ImageChunk::EXTRINSIC_CALIB), 
    
    /** @hideinitializer 
     * @brief \c
    */
    INTRINSIC_CALIB = static_cast<uint64_t>(ifm3d::ImageChunk::INTRINSIC_CALIB), 
    
    /** @hideinitializer 
     * @brief \c
    */
    INVERSE_INTRINSIC_CALIBRATION = static_cast<uint64_t>(ifm3d::ImageChunk::INVERSE_INTRINSIC_CALIBRATION), 
    

    O3R_DISTANCE_IMAGE_INFO [[deprecated]] = static_cast<uint64_t>(ifm3d::ImageChunk::TOF_INFO), 
    O3R_RGB_IMAGE_INFO [[deprecated]] = static_cast<uint64_t>(ifm3d::ImageChunk::RGB_INFO), 
    
    /** @hideinitializer 
     * @brief ifm3d::TOFInfoV3 / ifm3d::TOFInfoV4
    */
    TOF_INFO = static_cast<uint64_t>(ifm3d::ImageChunk::TOF_INFO), 
    
    /** @hideinitializer 
     * @brief ifm3d::RGBInfoV1
    */
    RGB_INFO = static_cast<uint64_t>(ifm3d::ImageChunk::RGB_INFO), 
    
    /** @hideinitializer 
     * @brief \c
    */
    JSON_MODEL = static_cast<uint64_t>(ifm3d::ImageChunk::JSON_MODEL), 
    
    /** @hideinitializer 
     * @brief \c
    */
    ALGO_DEBUG = static_cast<uint64_t>(ifm3d::ImageChunk::ALGO_DEBUG), 
    
    /** @hideinitializer
     * @brief ifm3d::ODSOccupancyGridV1
    */
    O3R_ODS_OCCUPANCY_GRID = static_cast<uint64_t>(ifm3d::ImageChunk::O3R_ODS_OCCUPANCY_GRID), 
    
    /** @hideinitializer
     * @brief ifm3d::ODSInfoV1
    */
    O3R_ODS_INFO = static_cast<uint64_t>(ifm3d::ImageChunk::O3R_ODS_INFO), 
    
    /** @hideinitializer
     * @brief ifm3d::O3R_RESULT_JSON
    */
    O3R_RESULT_JSON = static_cast<uint64_t>(ifm3d::ImageChunk::O3R_RESULT_JSON),

    /** @hideinitializer 
     * @brief ifm3d::O3R_RESULT_ARRAY2D
    */
    O3R_RESULT_ARRAY2D = static_cast<uint64_t>(ifm3d::ImageChunk::O3R_RESULT_ARRAY2D),

    /** @hideinitializer 
     * @brief ifm3d::O3R_RESULT_IMU
    */
    O3R_RESULT_IMU = static_cast<uint64_t>(ifm3d::ImageChunk::O3R_RESULT_IMU),

    /** @hideinitializer
     * @brief ifm3d::ODSPolarOccupancyGridV1
    */
    O3R_ODS_POLAR_OCC_GRID = static_cast<uint64_t>(ifm3d::ImageChunk::O3R_ODS_POLAR_OCC_GRID),

    /** @hideinitializer
     * @brief ifm3d::ODSExtrinsicCalibrationCorrectionV1
    */
    O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION = static_cast<uint64_t>(ifm3d::ImageChunk::O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION),

    /** @hideinitializer 
     * @brief The point cloud encoded as a 3 channel XYZ image
    */
    XYZ = (std::numeric_limits<std::uint32_t>::max)(),
    
    /** @hideinitializer 
     * @brief \c
     */
    EXPOSURE_TIME,
    
    /** @hideinitializer 
     * @brief \c
    */
    ILLUMINATION_TEMP,

    O3R_ODS_FLAGS,
    O3R_MCC_LIVE_IMAGE,
    O3R_MCC_MOTION_IMAGE,
    O3R_MCC_STATIC_IMAGE,
    O3R_ODS_RENDERED_ZONES,

    // clang-format on
  };

  // --- NEW STRUCT AND MAPPING ---
  // Structure to hold information about how a logical buffer_id is transported
  struct TransportInfo
  {
    ifm3d::buffer_id transport_id;    // The generic buffer_id it's wrapped in
                                      // (e.g., O3R_RESULT_ARRAY2D)
    std::string metadata_type_string; // The string value in the metadata that
                                      // identifies it
  };

  // Static mapping: Logical buffer_id -> TransportInfo
  // This map defines which specific data types are transported within generic
  // buffers and how to identify them via their metadata 'type' field.
  const std::unordered_map<buffer_id, TransportInfo>
    LOGICAL_TO_TRANSPORT_MAPPING = {
      {ifm3d::buffer_id::O3R_ODS_RENDERED_ZONES,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "ods_rendered_zones"}},
      {ifm3d::buffer_id::O3R_ODS_FLAGS,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "ods_flags"}},
      {ifm3d::buffer_id::O3R_MCC_STATIC_IMAGE,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "mcc_static_image"}},
      {ifm3d::buffer_id::O3R_MCC_LIVE_IMAGE,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "mcc_live_image"}},
      {ifm3d::buffer_id::O3R_MCC_MOTION_IMAGE,
       {ifm3d::buffer_id::O3R_RESULT_ARRAY2D, "mcc_motion_image"}}};
}
#endif // IFM3D_FG_Buffer_ID_H