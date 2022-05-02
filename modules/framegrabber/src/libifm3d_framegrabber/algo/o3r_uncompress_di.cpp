/* SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 ifm electronic gmbh
 *
 * THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
 */

#include <math.h>
#include <memory.h>

#include <cstddef>
#include <cstdint>

/** @brief c code example for evalIntrinsic function

    @param vx the x component of the unit vectors, array must have width*height
   elements, allocated from outside
    @param vy the y component of the unit vectors, array must have width*height
   elements, allocated from outside
    @param vz the z component of the unit vectors, array must have width*height
   elements, allocated from outside
    @param modelID the modelID as returned from O3R
    @param modelParameters the model parameters as returned from O3R
    @param width the width of the images as returned from O3R
    @param height the height of the images as returned from O3R
    @return zero on success, non-zero on error
*/
namespace ifm3d
{

  int32_t
  evalIntrinsic(float vx[],
                float vy[],
                float vz[],
                uint32_t modelID,
                const float modelParameters[],
                uint16_t width,
                uint16_t height)
  {
    switch (modelID)
      {
        case 0: {
          float fx = modelParameters[0];
          float fy = modelParameters[1];
          float mx = modelParameters[2];
          float my = modelParameters[3];
          float alpha = modelParameters[4];
          float k1 = modelParameters[5];
          float k2 = modelParameters[6];
          float k3 = modelParameters[7];
          float k4 = modelParameters[8];
          float k5 = modelParameters[9];
          uint16_t ix, iy;
          uint32_t idx = 0;
          for (iy = 0; iy < height; iy++)
            {
              for (ix = 0; ix < width; ix++)
                {
                  float r2, fradial, h, tx, ty, dx, dy, fnorm;
                  float cx = ((float)ix + 0.5f - mx) / fx;
                  float cy = ((float)iy + 0.5f - my) / fy;
                  cx -= alpha * cy;
                  r2 = cx * cx + cy * cy;
                  fradial = 1.f + r2 * (k1 + r2 * (k2 + r2 * k5));
                  h = 2.f * cx * cy;
                  tx = k3 * h + k4 * (r2 + 2 * cx * cx);
                  ty = k3 * (r2 + 2 * cy * cy) + k4 * h;
                  dx = fradial * cx + tx;
                  dy = fradial * cy + ty;
                  fnorm = 1.f / sqrtf(dx * dx + dy * dy + 1.f);
                  vx[idx] = fnorm * dx;
                  vy[idx] = fnorm * dy;
                  vz[idx] = fnorm;
                  ++idx;
                }
            }
        }
        return 0;
        case 2: {
          float fx = modelParameters[0];
          float fy = modelParameters[1];
          float mx = modelParameters[2];
          float my = modelParameters[3];
          float alpha = modelParameters[4];
          float k1 = modelParameters[5];
          float k2 = modelParameters[6];
          float k3 = modelParameters[7];
          float k4 = modelParameters[8];
          float theta_max = modelParameters[9];
          uint16_t ix, iy;
          uint32_t idx = 0;
          for (iy = 0; iy < height; iy++)
            {
              for (ix = 0; ix < width; ix++)
                {
                  float theta, theta_s, sin_theta, phi_s, p_radial;
                  float cx = ((float)ix + 0.5f - mx) / fx;
                  float cy = ((float)iy + 0.5f - my) / fy;
                  cx -= alpha * cy;
                  theta_s = sqrtf(cx * cx + cy * cy);
                  phi_s = (theta_s < theta_max) ? theta_s : theta_max;
                  phi_s = phi_s * phi_s;
                  p_radial =
                    1.f +
                    phi_s * (k1 + phi_s * (k2 + phi_s * (k3 + phi_s * k4)));
                  theta = theta_s * p_radial;
                  theta =
                    (theta < 0.0f) ? 0.0f : ((theta > M_PI) ? M_PI : theta);
                  sin_theta = sinf(theta);
                  vx[idx] =
                    (theta_s > 0.0f) ? ((cx / theta_s) * sin_theta) : 0.0f;
                  vy[idx] =
                    (theta_s > 0.0f) ? ((cy / theta_s) * sin_theta) : 0.0f;
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

   @param R resulting 3x3 rotation matrix, allocated externally
   @param rotX rotation around X axis [rad]
   @param rotY rotation around Y axis [rad]
   @param rotZ rotation around Z axis [rad]
   */
  int32_t
  rotMatFromAngles(float R[][3], float rotX, float rotY, float rotZ)
  {
    float cx = cosf(rotX);
    float sx = sinf(rotX);
    float cy = cosf(rotY);
    float sy = sinf(rotY);
    float cz = cosf(rotZ);
    float sz = sinf(rotZ);
    R[0][0] = cy * cz;
    R[0][1] = -cy * sz;
    R[0][2] = sy;
    R[1][0] = cx * sz + cz * sx * sy;
    R[1][1] = cx * cz - sx * sy * sz;
    R[1][2] = -cy * sx;
    R[2][0] = sx * sz - cx * cz * sy;
    R[2][1] = cz * sx + cx * sy * sz;
    R[2][2] = cx * cy;

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
   @param u16Dist encoded distance matrix as returned from O3R
   @param distResolution the resolution of the distance matrix as returned from
   O3R
   @param intrModelID the intrinsic modelID as returned from O3R
   @param intrModelParameters the intrinsic model parameters as returned from
   O3R
   @param extrTransX the extrinsic translation, X [m]
   @param extrTransY the extrinsic translation, Y [m]
   @param extrTransZ the extrinsic translation, Z [m]
   @param extrRotX the extrinsic rotation around X axis [rad]
   @param extrRotY the extrinsic rotation around Y axis [rad]
   @param extrRotZ the extrinsic rotation around Z axis [rad]
   @param width the width of the images
   @param height the height of the images
   */
  int32_t
  xyzdFromDistance(float dist[],
                   float x[],
                   float y[],
                   float z[],
                   const uint16_t u16Dist[],
                   float distResolution,
                   uint32_t intrModelID,
                   const float intrModelParameters[],
                   float extrTransX,
                   float extrTransY,
                   float extrTransZ,
                   float extrRotX,
                   float extrRotY,
                   float extrRotZ,
                   uint16_t width,
                   uint16_t height)
  {
    uint32_t idx;
    float R[3][3];

    /* Note: if necessary, the rotated unit vectors might be cached if
             intrModelID, intrModelParameters, extrRot[XYZ], width and height
       do not change. This is not performed here for simplicity. */

    if (0 != evalIntrinsic(x,
                           y,
                           z,
                           intrModelID,
                           intrModelParameters,
                           width,
                           height))
      {
        return 1;
      }

    if (0 != rotMatFromAngles(R, extrRotX, extrRotY, extrRotZ))
      {
        return 1;
      }

    for (idx = 0; idx < (uint32_t)width * (uint32_t)height; idx++)
      {
        float vx, vy, vz, d;
        d = (float)u16Dist[idx] * distResolution;
        vx = x[idx];
        vy = y[idx];
        vz = z[idx];
        x[idx] =
          (u16Dist[idx] == 0) ?
            0.0f :
            (d * (R[0][0] * vx + R[0][1] * vy + R[0][2] * vz) + extrTransX);
        y[idx] =
          (u16Dist[idx] == 0) ?
            0.0f :
            (d * (R[1][0] * vx + R[1][1] * vy + R[1][2] * vz) + extrTransY);
        z[idx] =
          (u16Dist[idx] == 0) ?
            0.0f :
            (d * (R[2][0] * vx + R[2][1] * vy + R[2][2] * vz) + extrTransZ);
        dist[idx] = d;
      }
    return 0;
  }

  /**
   @brief c code example for converting the distanceNoise matrix to [m].

   @param distanceNoise resulting distance noise matrix, must have width*height
   elements, allocated externally
   @param u16DistanceNoise encoded distance noise matrix as returned from O3R
   @param distResolution the resolution of the distance matrix as returned from
   O3R
   @param width the width of the images
   @param height the height of the images
   */
  int32_t
  convertDistanceNoise(float distanceNoise[],
                       const uint16_t u16DistanceNoise[],
                       float distResolution,
                       uint16_t width,
                       uint16_t height)
  {
    uint32_t idx;
    for (idx = 0; idx < (uint32_t)width * (uint32_t)height; idx++)
      {
        distanceNoise[idx] = (float)u16DistanceNoise[idx] * distResolution;
      }
    return 0;
  }

  /**
   @brief c code example for converting the encoded amplitude matrix.

   @param amplitude resulting amplitude matrix, must have width*height
   elements, allocated externally
   @param u16Amplitude encoded amplitude as returned from O3R
   @param amplitudeResolution the resolution of the distance matrix as returned
   from O3R
   @param width the width of the images
   @param height the height of the images
   */
  int32_t
  convertAmplitude(float amplitude[],
                   const uint16_t u16Amplitude[],
                   float amplitudeResolution,
                   uint16_t width,
                   uint16_t height)
  {
    uint32_t idx;
    for (idx = 0; idx < (uint32_t)width * (uint32_t)height; idx++)
      {
        float a = (float)u16Amplitude[idx] - 1.f;
        amplitude[idx] = (a >= 0.0f) ? (a * a * amplitudeResolution) : -1.f;
      }
    return 0;
  }

}