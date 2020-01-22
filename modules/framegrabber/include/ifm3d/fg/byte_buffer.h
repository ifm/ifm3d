// -*- c++ -*-
/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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

#ifndef __IFM3D_FG_BYTE_BUFFER_H__
#define __IFM3D_FG_BYTE_BUFFER_H__

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <ifm3d/fg/frame_grabber_export.h>

namespace ifm3d
{
  using TimePointT =
    std::chrono::time_point<std::chrono::system_clock,
                            std::chrono::nanoseconds>;

  extern IFM3D_FRAME_GRABBER_EXPORT const std::size_t IMG_TICKET_SZ; // bytes
  extern IFM3D_FRAME_GRABBER_EXPORT const std::size_t IMG_BUFF_START;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::size_t NUM_EXTRINSIC_PARAM;
  extern IFM3D_FRAME_GRABBER_EXPORT const std::size_t NUM_INTRINSIC_PARAM;

  enum class pixel_format : std::uint32_t
  {
    FORMAT_8U = 0,
    FORMAT_8S = 1,
    FORMAT_16U = 2,
    FORMAT_16S = 3,
    FORMAT_32U = 4,
    FORMAT_32S = 5,
    FORMAT_32F = 6,
    FORMAT_64U = 7,
    FORMAT_64F = 8,
    FORMAT_16U2 = 9,
    FORMAT_32F3 = 10
  };

  enum class image_chunk : std::uint32_t
  {
    RADIAL_DISTANCE = 100,
    AMPLITUDE = 101, // normalized amplitude
    RAW_AMPLITUDE = 103,
    GRAY = 104, // ambient light
    CARTESIAN_X = 200,
    CARTESIAN_Y = 201,
    CARTESIAN_Z = 202,
    CARTESIAN_ALL = 203,
    UNIT_VECTOR_ALL = 223,
    CONFIDENCE = 300,
    DIAGNOSTIC_DATA = 302,
    EXTRINSIC_CALIBRATION = 400,
    INTRINSIC_CALIBRATION = 401,
    INVERSE_INTRINSIC_CALIBRATION = 402,
    JSON_MODEL = 500,
  };
  // intrinsic param key which cahn be used for indexing the vector of the
  // intrinsic parameter
  enum class intrinsic_param : std::uint32_t
  {
    F_X = 0,    //Focal length of the camera in the sensor's x axis direction.
    F_Y = 1,    //Focal length of the camera in the sensor's y axis direction.
    M_X = 2,    //Main point in the sensor's x direction
    M_Y = 3,    //Main point in the sensor's x direction
    ALPHA = 4,  //Skew parameter
    K1 = 5,   //First radial distortion coefficient
    K2 = 6,   //Second radial distortion coefficient
    K5 = 7,   //Third radial distortion coefficient
    K3 = 8,   //First tangential distortion coefficient
    K4 = 9,   //Second tangential distortion coefficient
    TRANS_X = 10, //Translation along x-direction in meters.
    TRANS_Y = 11, //Translation along y-direction in meters.
    TRANS_Z = 12, //Translation along Z-direction in meters.
    ROT_X = 13, //Rotation along x-axis in radians. Positive values indicate clockwise rotation.
    ROT_Y = 14, //Rotation along y-axis in radians. Positive values indicate clockwise rotation.
    ROT_Z = 15  //Rotation along z-axis in radians. Positive values indicate clockwise rotation.
  };

  enum class extrinsic_param : std::uint32_t
  {
    TRANS_X = 0,  //Translation along x-direction in meters.
    TRANS_Y = 1,  //Translation along y-direction in meters.
    TRANS_Z = 2,  //Translation along Z-direction in meters.
    ROT_X = 3,  //Rotation along x-axis in radians. Positive values indicate clockwise rotation.
    ROT_Y = 4,  //Rotation along y-axis in radians. Positive values indicate clockwise rotation.
    ROT_Z = 5   //Rotation along z-axis in radians. Positive values indicate clockwise rotation.
  };

  /**
   * Validates the passed in "ticket" from the sensor. This is a low-level
   * PCIC protocol detail.
   *
   * @param[in] buff The raw ticket bytes from the sensor
   *
   * @return true if the ticket buffer is valid
   */
  bool verify_ticket_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Verifies that the passed in image buffer is valid.
   *
   * @param[in] buff The raw bytes from the sensor including all
   *                 framing and chunk headers
   *
   * @return true if the image buffer is valid
   */
  bool verify_image_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Extracts the image buffer size from an image ticket buffer received from
   * the sensor.
   *
   * NOTE: The size of the passed in buffer is not checked. It is assumed
   * that you have already called `verify_ticket_buffer` on the passed
   * in buff.
   *
   * @param[in] buff A verified image ticket buffer
   *
   * @return The expected size of the image buffer
   */
  std::size_t get_image_buffer_size(const std::vector<std::uint8_t>& buff);

  /**
   * Finds the index into the image buffer of where the chunk of `chunk_type'
   * begins.
   *
   * @param[in] buff The image buffer to search
   * @param[in] chunk_type The type of chunk to look for
   * @param[in] start_idx The first index into the byte buffer
   *                      to look for the chunk of interest
   *
   * @return The index into the buffer of where the chunk begins or
   *         std::numeric_limits<std::size_t>::max() if the chunk was not
   *         found.
   */
  std::size_t get_chunk_index(const std::vector<std::uint8_t>& buff,
                              ifm3d::image_chunk chunk_type,
                              std::size_t start_idx = ifm3d::IMG_BUFF_START);

  /**
   * Create a value of type T from sizeof(T) bytes of the passed in byte
   * buffer. Given that the ifm sensors transmit data in little endian
   * format, this function will swap bytes if necessary for the host
   * representation of T.
   *
   * @param[in] buff A pointer to a buffer in memory intended to be interpreted
   * as data of type T and assuming the buffer is little endian.
   *
   * @return An interpretation of `buff` as type T with bytes swapped as
   * appropriate for the host's byte ordering semantics.
   */
  template<typename T>
  T mkval(const unsigned char *buff)
  {
    union
    {
      T v;
      unsigned char bytes[sizeof(T)];
    } value;

#if !defined(_WIN32) && __BYTE_ORDER == __BIG_ENDIAN
    std::reverse_copy(buff, buff + sizeof(T), value.bytes);
#else
    std::copy(buff, buff + sizeof(T), value.bytes);
#endif

    return value.v;
  }

  /**
   * The ByteBuffer class is used to hold a validated byte buffer from the
   * sensor that represents a single time-synchronized set of images based on
   * the current schema mask set on the active framegrabber.
   *
   * The ByteBuffer imposes no specific image or point cloud data
   * structure. This class is intended to be subclassed where more user-friendly
   * data structures can be used to gain access to the bytes in a semantically
   * meaningful manner.
   *
   * There are two primary interfaces (documented below) that image container
   * developers should implement. They are:
   *
   * ``ImCreate``
   * ``CloudCreate``
   *
   * These functions will be called as customization hooks at the time of
   * parsing the raw image bytes from the sensor. It is the contract of this
   * interface that all calls to ``ImCreate`` will be made before the single
   * call to ``CloudCreate``. The reason for this part of the contract is to
   * give all image container implementers the ability to set the point cloud
   * intensity data from one of the image containers. That is, at the
   * time of calling ``CloudCreate`` all image data (e.g., amplitude or gray)
   * data are avaialble for coloring the point cloud intensity pixels. In
   * addition, it is guaranteed that the first call to ``ImCreate`` will be the
   * confidence image. So, the confidence bits for a given pixel can be
   * consulting while constructing the payload images.
   *
   * We note that the ploymorphic behaviors implemented by this class are
   * static (resolved at compile-time via CRTP) rather than dynamic at runtime
   * (via a vtable). This is why there are no interface functions declared
   * `virtual`.
   *
   * NOTE: The ByteBuffer is NOT thread safe!
   */
  template <typename Derived>
  class ByteBuffer
  {
  public:
    using Ptr = std::shared_ptr<ByteBuffer<Derived> >;

    /**
     * Default initializes instance vars
     */
    ByteBuffer();

    /**
     * RAII dealloc
     */
    ~ByteBuffer();

    // move semantics
    ByteBuffer(ByteBuffer&&);
    ByteBuffer& operator=(ByteBuffer&&);

    // copy ctor/assignment operator
    ByteBuffer(const ByteBuffer& src_buff);
    ByteBuffer& operator=(const ByteBuffer& src_buff);

    /**
     * Returns a copy of the underlying byte buffer read from the camera
     */
    std::vector<std::uint8_t> Bytes();

    /**
     * Returns the state of the `dirty' flag
     */
    bool Dirty() const noexcept;

    /**
     * Sets the data from the passed in `buff' to the internally wrapped byte
     * buffer. This function assumes the passed in `buff' is valid.
     *
     * By default, this function will take in `buff` and `swap` contents with
     * its internal buffer so that the operation is O(1) and requires no data
     * copies. If you want copy behavior, specify the `copy` flag and
     * complexity will be linear in the size of the byte buffer which is driven
     * by the schema mask currently applied to the running framegrabber.
     *
     * @param[in] buff Raw data bytes to copy/swap to internal buffers
     * @param[in] copy If true, the data are copied from `buff` to the
     *                 internally wrapped buffer and `buff` will remain
     *                 unchanged.
     */
    void SetBytes(std::vector<std::uint8_t>& buff, bool copy = false);

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
     *  Returns a 16-element vector containing the intrinsic
     * calibration of the camera.
     *
     * The elements are:
     *
     * Name  Data type   Unit      Description
     * fx    32 bit float  px        Focal length of the camera in the sensor's x axis direction.
     * fy    32 bit float  px        Focal length of the camera in the sensor's y axis direction.
     * mx    32 bit float  px        Main point in the sensor's x direction
     * my    32 bit float  px        Main point in the sensor's y direction
     * alpha 32 bit float  dimensionless Skew parameter
     * k1    32 bit float  dimensionless First radial distortion coefficient
     * k2    32 bit float  dimensionless Second radial distortion coefficient
     * k5    32 bit float  dimensionless Third radial distortion coefficient
     * k3    32 bit float  dimensionless First tangential distortion coefficient
     * k4    32 bit float  dimensionless Second tangential distortion coefficient
     * transX  32 bit float  mm        Translation along x-direction in meters.
     * transY  32 bit float  mm        Translation along y-direction in meters.
     * transZ  32 bit float  mm        Translation along z-direction in meters.
     * rotX  32 bit float  degree        Rotation along x-axis in radians. Positive values indicate clockwise rotation.
     * rotY  32 bit float  degree        Rotation along y-axis in radians. Positive values indicate clockwise rotation.
     * rotZ  32 bit float  degree        Rotation along z-axis in radians. Positive values indicate clockwise rotation.
     *
     */
    std::vector<float> Intrinsics();

    /**
     *  Returns a 16-element vector containing the inverse intrinsic
     *  calibration of the camera. See Intrinsics() for further information
     */
    std::vector<float> InverseIntrinsics();

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
     * Returns the time stamp of the image data.
     *
     * NOTE: To get the timestamp of the confidence data, you
     * need to make sure your current pcic schema mask have enabled confidence
     * data.
     */
    ifm3d::TimePointT TimeStamp();

    /**
     * Returns the temperature of the illumination unit.
     *
     * NOTE: To get the temperature of the illumination unit to the frame, you
     * need to make sure your current pcic schema asks for it.
     */
    float IlluTemp();

    /**
     * Returns the JSON model of the output of the active application
     *
     * NOTE: To get the JSON data for the application running on the device,
     * you need to make sure your current pcic schema asks for it by including
     * ifm3d::JSON_MODEL in the schema. This will return a blank JSON string
     * ("{}") for Camera devices like the O3D303, versus ifm Smart Sensors like
     * the O3D302.
     */
    std::string JSONModel();

    /**
     * This is the interface hook that synchronizes the internally wrapped byte
     * buffer with the semantically meaningful image/cloud data
     * structures. Within the overall `ifm3d` framework, this function is
     * called by the `FrameGrabber` when a complete "frame packet" has been
     * recieved. This then parses the bytes and, in-line, will statically
     * dispatch to the underly dervied class to populate their image/cloud data
     * structures.
     *
     * Additionally, this function will populate the extrinsics, exposure
     * times, timestamp, and illumination temperature as appropriate and
     * subject to the current pcic schema.
     */
    void Organize();

  protected:
    /**
     * This function is part of the ByteBuffer interface, intended to be
     * overloaded by image container implementers. It is a callback hook that
     * is called once for each 2D image type specified in the current pcic
     * schema for each frame recieved by the framegrabber. All 2D image
     * callbacks are  guaranteed to be called prior to the ``CloudCreate``
     * callback -- this is to allow for "coloring" the intensity channel of the
     * point cloud with data from one of the 2D images (already parsed)
     *
     * For a given frame, the first ``ImCreate`` callback will be the
     * confidence image.
     *
     * @param[in] im The 2D image type currently being processed
     * @param[in] fmt The pixel format of the image (see `ifm3d::pixel_format`)
     * @param[in] idx The index into the byte buffer, `bytes`, as to where the
     *                pixel data begin
     * @param[in] width The image width (pixels)
     * @param[in] height The image height (pixels)
     * @param[in] nchan The number of channels in the image
     * @param[in] npts The total number of image points (width * height)
     * @param[in] bytes A const reference to the byte buffer to process
     */
    template <typename T>
    void ImCreate(ifm3d::image_chunk im,
                  std::uint32_t fmt,
                  std::size_t idx,
                  std::uint32_t width,
                  std::uint32_t height,
                  int nchan,
                  std::uint32_t npts,
                  const std::vector<std::uint8_t>& bytes)
    {
      static_cast<Derived *>(this)
        ->template ImCreate<T>(im, fmt, idx, width, height, nchan, npts, bytes);
    }

    /**
     * This function is part of the ByteBuffer interface, intended to be
     * overloaded by image container implementers. It is a callback hook that
     * is called once for each frame recieved by the framegrabber if the
     * cartesian data are specified in the current pcic schema. All 2D image
     * callbacks (i.e., ``ImCreate``) are  guaranteed to be called prior to
     * this function -- this is to allow for "coloring" the intensity channel
     * of the point cloud with data from one of the 2D images (already
     * parsed). It is also implied that for a given frame, before this function
     * is called, the image container implementer will have had the opportunity
     * to construct the confidence image associated with this frame.
     *
     * @param[in] fmt The pixel format of the image (see `ifm3d::pixel_format`)
     * @param[in] xidx The index into `bytes` where the x-coords start
     * @param[in] yidx The index into `bytes` where the y-coords start
     * @param[in] zidx The index into `bytes` where the z-coords start
     * @param[in] width Number of columns in the point cloud
     * @param[in] height Number of rows in the point cloud
     * @param[in] npts Total points in the point cloud (width * height)
     * @paramin] bytes A const reference to the byte buffer to process
     *
     */
    template <typename T>
    void CloudCreate(std::uint32_t fmt,
                     std::size_t xidx,
                     std::size_t yidx,
                     std::size_t zidx,
                     std::uint32_t width,
                     std::uint32_t height,
                     std::uint32_t npts,
                     const std::vector<std::uint8_t>& bytes)
    {
      static_cast<Derived *>(this)
        ->template CloudCreate<T>(fmt, xidx, yidx, zidx, width, height, npts, bytes);
    }

    /**
     * Mutates the dirty flag
     */
    void _SetDirty(bool flg) noexcept;

    /**
     * Flag used to indicate if the wrapped byte buffer needs to be
     * `Organized'. I.e., in a subclass, this would indicate if your parsed out
     * image data structures need to be synchronized to the underlying byte
     * buffer or not.
     */
    bool dirty_;

    /**
     * Raw bytes read off the wire from the camera.
     */
    std::vector<std::uint8_t> bytes_;

    /**
     * Extrinsic calibration WRT camera optical frame:
     * tx, ty, tz, rotx, roty, rotz. Translation units are mm, rotational units
     * are degrees.
     */
    std::vector<float> extrinsics_;

    /**
     * Intrinsic calibration WRT camera lense
     */
    std::vector<float> intrinsics_;

    /**
     * Inverse intrinsic calibration WRT camera lense:
     */
    std::vector<float> inverseIntrinsics_;

    /**
     * Exposure time(s) (up to 3), registered to the current frame.
     */
    std::vector<std::uint32_t> exposure_times_;

    /**
     * Camera timestamp of the current frame
     */
    ifm3d::TimePointT time_stamp_;

    /**
     * Temperature of the illumination unit synchronized in time with the
     * current frame data.
     */
    float illu_temp_;

    /**
     * JSON string of the active application output
     */
    std::string json_model_;

  private:
    /**
     * flag for checking if intrinsic values are already available
     */
    bool intrinsic_available;
    bool inverse_intrinsic_available;
  }; // end: class ByteBuffer

} // end: namespace ifm3d

#include <ifm3d/fg/detail/byte_buffer.hpp>

#endif // __IFM3D_FG_BYTE_BUFFER_H__
