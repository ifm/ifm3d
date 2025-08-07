/* SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 ifm electronic gmbh
 *
 * THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
 */

#include <cmath>

#include <cstdint>

// NOLINTBEGIN

#ifndef M_PI
#  define M_PI (3.141592653589793f)
#endif

/** @brief c code example for eval_intrinsic function

    @param vx the x component of the unit vectors, array must have width*height
   elements, allocated from outside
    @param vy the y component of the unit vectors, array must have width*height
   elements, allocated from outside
    @param vz the z component of the unit vectors, array must have width*height
   elements, allocated from outside
    @param model_id the model_id as returned from O3R
    @param model_parameters the model parameters as returned from O3R
    @param width the width of the images as returned from O3R
    @param height the height of the images as returned from O3R
    @return zero on success, non-zero on error
*/
namespace ifm3d
{

  int32_t
  eval_intrinsic(float vx[],
                 float vy[],
                 float vz[],
                 uint32_t model_id,
                 const float model_parameters[],
                 uint16_t width,
                 uint16_t height)
  {
    switch (model_id)
      {
        case 0: {
          float const fx = model_parameters[0];
          float const fy = model_parameters[1];
          float const mx = model_parameters[2];
          float const my = model_parameters[3];
          float const alpha = model_parameters[4];
          float const k1 = model_parameters[5];
          float const k2 = model_parameters[6];
          float const k3 = model_parameters[7];
          float const k4 = model_parameters[8];
          float const k5 = model_parameters[9];
          uint16_t ix = 0;
          uint16_t iy = 0;
          uint32_t idx = 0;
          for (iy = 0; iy < height; iy++)
            {
              for (ix = 0; ix < width; ix++)
                {
                  float r2 = NAN;
                  float fradial = NAN;
                  float h = NAN;
                  float tx = NAN;
                  float ty = NAN;
                  float dx = NAN;
                  float dy = NAN;
                  float fnorm = NAN;
                  float cx = ((float)ix + 0.5F - mx) / fx;
                  float const cy = ((float)iy + 0.5F - my) / fy;
                  cx -= alpha * cy;
                  r2 = cx * cx + cy * cy;
                  fradial = 1.F + r2 * (k1 + r2 * (k2 + r2 * k5));
                  h = 2.F * cx * cy;
                  tx = k3 * h + k4 * (r2 + 2 * cx * cx);
                  ty = k3 * (r2 + 2 * cy * cy) + k4 * h;
                  dx = fradial * cx + tx;
                  dy = fradial * cy + ty;
                  fnorm = 1.F / sqrtf((dx * dx) + (dy * dy) + 1.F);
                  vx[idx] = fnorm * dx;
                  vy[idx] = fnorm * dy;
                  vz[idx] = fnorm;
                  ++idx;
                }
            }
        }
        return 0;
        case 2: {
          float const fx = model_parameters[0];
          float const fy = model_parameters[1];
          float const mx = model_parameters[2];
          float const my = model_parameters[3];
          float const alpha = model_parameters[4];
          float const k1 = model_parameters[5];
          float const k2 = model_parameters[6];
          float const k3 = model_parameters[7];
          float const k4 = model_parameters[8];
          float const theta_max = model_parameters[9];
          uint16_t ix = 0;
          uint16_t iy = 0;
          uint32_t idx = 0;
          for (iy = 0; iy < height; iy++)
            {
              for (ix = 0; ix < width; ix++)
                {
                  float theta = NAN;
                  float theta_s = NAN;
                  float sin_theta = NAN;
                  float phi_s = NAN;
                  float p_radial = NAN;
                  float cx = ((float)ix + 0.5F - mx) / fx;
                  float const cy = ((float)iy + 0.5F - my) / fy;
                  cx -= alpha * cy;
                  theta_s = sqrtf((cx * cx) + (cy * cy));
                  phi_s = (theta_s < theta_max) ? theta_s : theta_max;
                  phi_s = phi_s * phi_s;
                  p_radial =
                    1.F +
                    phi_s * (k1 + phi_s * (k2 + phi_s * (k3 + phi_s * k4)));
                  theta = theta_s * p_radial;
                  theta =
                    (theta < 0.0F) ? 0.0F : ((theta > M_PI) ? M_PI : theta);
                  sin_theta = sinf(theta);
                  vx[idx] =
                    (theta_s > 0.0F) ? ((cx / theta_s) * sin_theta) : 0.0F;
                  vy[idx] =
                    (theta_s > 0.0F) ? ((cy / theta_s) * sin_theta) : 0.0F;
                  vz[idx] = cosf(theta);
                  ++idx;
                }
            }
        }
        return 0;
      default:
        return 1;
      }
  }

  /**
   @brief c code example for converting the euler angles to a rotation matrix.

   @param r resulting 3x3 rotation matrix, allocated externally
   @param rot_x rotation around X axis [rad]
   @param rot_y rotation around Y axis [rad]
   @param rot_z rotation around Z axis [rad]
   */
  int32_t
  rot_mat_from_angles(float r[][3], float rot_x, float rot_y, float rot_z)
  {
    float const cx = cosf(rot_x);
    float const sx = sinf(rot_x);
    float const cy = cosf(rot_y);
    float const sy = sinf(rot_y);
    float const cz = cosf(rot_z);
    float const sz = sinf(rot_z);
    r[0][0] = cy * cz;
    r[0][1] = -cy * sz;
    r[0][2] = sy;
    r[1][0] = cx * sz + cz * sx * sy;
    r[1][1] = cx * cz - sx * sy * sz;
    r[1][2] = -cy * sx;
    r[2][0] = sx * sz - cx * cz * sy;
    r[2][1] = cz * sx + cx * sy * sz;
    r[2][2] = cx * cy;

    return 0;
  }

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
  int32_t
  xyzd_from_distance(float dist[],
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
                     uint16_t height)
  {
    uint32_t idx = 0;
    float r[3][3];

    /* Note: if necessary, the rotated unit vectors might be cached if
             intr_model_id, intr_model_parameters, extrRot[XYZ], width and
       height do not change. This is not performed here for simplicity. */

    if (0 != eval_intrinsic(x,
                            y,
                            z,
                            intr_model_id,
                            intr_model_parameters,
                            width,
                            height))
      {
        return 1;
      }

    if (0 != rot_mat_from_angles(r, extr_rot_x, extr_rot_y, extr_rot_z))
      {
        return 1;
      }

    for (idx = 0; idx < (uint32_t)width * (uint32_t)height; idx++)
      {
        float vx = NAN;
        float vy = NAN;
        float vz = NAN;
        float d = NAN;
        d = (float)u16_dist[idx] * dist_resolution;
        vx = x[idx];
        vy = y[idx];
        vz = z[idx];
        x[idx] = (u16_dist[idx] == 0) ?
                   0.0F :
                   ((d * (r[0][0] * vx + r[0][1] * vy + r[0][2] * vz)) +
                    extr_trans_x);
        y[idx] = (u16_dist[idx] == 0) ?
                   0.0F :
                   ((d * (r[1][0] * vx + r[1][1] * vy + r[1][2] * vz)) +
                    extr_trans_y);
        z[idx] = (u16_dist[idx] == 0) ?
                   0.0F :
                   ((d * (r[2][0] * vx + r[2][1] * vy + r[2][2] * vz)) +
                    extr_trans_z);
        dist[idx] = d;
      }
    return 0;
  }

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
  int32_t
  convert_distance_noise(float distance_noise[],
                         const uint16_t u16_distance_noise[],
                         float dist_resolution,
                         uint16_t width,
                         uint16_t height)
  {
    uint32_t idx = 0;
    for (idx = 0; idx < (uint32_t)width * (uint32_t)height; idx++)
      {
        distance_noise[idx] = (float)u16_distance_noise[idx] * dist_resolution;
      }
    return 0;
  }

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
  int32_t
  convert_amplitude(float amplitude[],
                    const uint16_t u16_amplitude[],
                    float amplitude_resolution,
                    uint16_t width,
                    uint16_t height)
  {
    uint32_t idx = 0;
    for (idx = 0; idx < (uint32_t)width * (uint32_t)height; idx++)
      {
        float const a = (float)u16_amplitude[idx] - 1.F;
        amplitude[idx] = (a >= 0.0F) ? (a * a * amplitude_resolution) : -1.F;
      }
    return 0;
  }
}

// NOLINTEND