/* SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 ifm electronic gmbh
 *
 * THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
 */

#ifndef O3R_UNCOMPRESS_DI_H
#define O3R_UNCOMPRESS_DI_H

#include <cstdint>

namespace ifm3d
{

  /**
   @brief c code example for converting the distance matrix to cartesian
   coordinates.

   @param dist resulting distance matrix, must have width*height elements,
   allocated externally
   @param x resulting cartesian x matrix, must have width*height elements,
   allocated externally
   @param y resulting cartesian y matrix, must have width*height elements,
   allocated externally
   @param z resulting cartesian z matrix, must have width*height elements,
   allocated externally
   @param u16_dist encoded distance matrix as returned from O3R
   @param dist_resolution the resolution of the distance matrix as returned
   from O3R
   @param intr_model_id the intrinsic model_id as returned from O3R
   @param intr_model_parameters the intrinsic model parameters as returned from
   O3R
   @param extr_trans_x the extrinsic translation, X [m]
   @param extr_trans_y the extrinsic translation, Y [m]
   @param extr_trans_z the extrinsic translation, Z [m]
   @param extr_rot_x the extrinsic rotation around X axis [rad]
   @param extr_rot_y the extrinsic rotation around Y axis [rad]
   @param extr_rot_z the extrinsic rotation around Z axis [rad]
   @param width the width of the images
   @param height the height of the images
   */
  int32_t xyzd_from_distance(float dist[],
                             float x[],
                             float y[],
                             float z[],
                             const uint16_t u16_dist[],
                             float dist_resolution,
                             uint32_t intr_model_id,
                             const float intr_model_parameters[],
                             float extr_trans_x,
                             float extr_trans_y,
                             float extr_trans_z,
                             float extr_rot_x,
                             float extr_rot_y,
                             float extr_rot_z,
                             uint16_t width,
                             uint16_t height);

  /**
   @brief c code example for converting the distance_noise matrix to [m].

   @param distance_noise resulting distance noise matrix, must have
   width*height elements, allocated externally
   @param u16_distance_noise encoded distance noise matrix as returned from O3R
   @param dist_resolution the resolution of the distance matrix as returned
   from O3R
   @param width the width of the images
   @param height the height of the images
   */
  int32_t convert_distance_noise(float distance_noise[],
                                 const uint16_t u16_distance_noise[],
                                 float dist_resolution,
                                 uint16_t width,
                                 uint16_t height);

  /**
   @brief c code example for converting the encoded amplitude matrix.

   @param amplitude resulting amplitude matrix, must have width*height
   elements, allocated externally
   @param u16_amplitude encoded amplitude as returned from O3R
   @param amplitude_resolution the resolution of the distance matrix as
   returned from O3R
   @param width the width of the images
   @param height the height of the images
   */
  int32_t convert_amplitude(float amplitude[],
                            const uint16_t u16_amplitude[],
                            float amplitude_resolution,
                            uint16_t width,
                            uint16_t height);

  /** @brief c code example for eval_intrinsic function

      @param vx the x component of the unit vectors, array must have
     width*height elements, allocated from outside
      @param vy the y component of the unit vectors, array must have
     width*height elements, allocated from outside
      @param vz the z component of the unit vectors, array must have
     width*height elements, allocated from outside
      @param model_id the model_id as returned from O3R
      @param model_parameters the model parameters as returned from O3R
      @param width the width of the images as returned from O3R
      @param height the height of the images as returned from O3R
      @return zero on success, non-zero on error
  */
  int32_t eval_intrinsic(float vx[],
                         float vy[],
                         float vz[],
                         uint32_t model_id,
                         const float model_parameters[],
                         uint16_t width,
                         uint16_t height);

  /**
   @brief c code example for converting the euler angles to a rotation matrix.

   @param r resulting 3x3 rotation matrix, allocated externally
   @param rot_x rotation around X axis [rad]
   @param rot_y rotation around Y axis [rad]
   @param rot_z rotation around Z axis [rad]
   */
  int32_t rot_mat_from_angles(float r[][3],
                              float rot_x,
                              float rot_y,
                              float rot_z);

}

#endif
