// -*- c++ -*-
/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 * Copyright (C) 2018 ifm syntron gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IFM3D_SIMPLE_IMAGE_IMAGE_BUFFER_H__
#define __IFM3D_SIMPLE_IMAGE_IMAGE_BUFFER_H__

#include <cstdint>
#include <memory>
#include <vector>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{
/**
 * The ImageBuffer class is a composite data structure used to hold
 * time-synchronized images from the sensor. It organizes a single byte
 * buffer read from the sensor into its component parts.
 *
 * This class is not thread safe
 */
class SimpleImageBuffer : public ifm3d::ByteBuffer
{
public:

  struct Img
  {
    std::vector<std::uint8_t> data;
    int width;
    int height;
    pixel_format format;
  };

  struct Point
  {
    float x;
    float y;
    float z;
  };

  struct PointCloud
  {
    std::vector<Point> points;
    int width;
    int height;
  };

  using Ptr = std::shared_ptr<SimpleImageBuffer>;

  /**
   * Allocates space for the individual component images.
   */
  SimpleImageBuffer();

  /**
   * RAII decallocations
   */
  virtual ~SimpleImageBuffer();

  // disable move semantics
  SimpleImageBuffer(SimpleImageBuffer&&) = delete;
  SimpleImageBuffer& operator=(SimpleImageBuffer&&) = delete;

  // copy semantics
  SimpleImageBuffer(const SimpleImageBuffer& src_buff);
  SimpleImageBuffer& operator=(const SimpleImageBuffer& src_buff);

  /**
   * Accessor for the wrapped radial distance image
   */
  Img DistanceImage();

  /**
   * Accessor for the wrapped unit vectors
   */
  Img UnitVectors();

  /**
   * Accessor the the wrapped ambient light image
   */
  Img GrayImage();

  /**
   * Accessor for the normalized amplitude image
   */
  Img AmplitudeImage();

  /**
   * Accessor for the raw amplitude image
   */
  Img RawAmplitudeImage();

  /**
   * Accessor for the confidence image
   */
  Img ConfidenceImage();

  /**
   * Accessor for the image encoding of the point cloud
   *
   * 3-channel image of spatial planes X, Y, Z
   */
  Img XYZImage();

  /**
   * Returns the point cloud
   */
  PointCloud Cloud();

  /**
   * Returns a 6-element vector containing the extrinsic
   * calibration of the camera. NOTE: This is the extrinsics WRT to the ifm
   * optical frame.
   *
   * The elements are: tx, ty, tz, rot_x, rot_y, rot_z
   *
   * Translation units are mm, rotations are degrees
   *
   * Users of this library are highly DISCOURAGED from using the extrinsic
   * calibration data stored on the camera itself.
   */
  std::vector<float> Extrinsics();

  /**
   * Returns a 3-element vector containing the exposure times (usec) for the
   * current frame. Unused exposure times are reported as 0.
   *
   * If all elements are reported as 0 either the exposure times are not
   * configured to be returned back in the data stream from the camera or an
   * error in parsing them has occured.
   */
  std::vector<std::uint32_t> ExposureTimes();

  /**
   * Synchronizes the parsed out image data with the internally wrapped byte
   * buffer.
   */
  virtual void Organize();

private:
  class Impl;
  std::unique_ptr<Impl> pImpl;

}; // end: class SimpleImageBuffer
} // end: namespace ifm3d

#endif // __IFM3D_SIMPLE_IMAGE_IMAGE_BUFFER_H__
