/* SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 ifm electronic gmbh
 *
 * THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
 */

#ifndef O3R_UNCOMPRESS_DI_H
#define O3R_UNCOMPRESS_DI_H

namespace ifm3d
{
// extern "C" {
#include <cstddef>
#include <stdint.h>
#include <cstdint>

/**
 @brief c code example for converting the distance matrix to cartesian coordinates.

 @param dist resulting distance matrix, must have width*height elements, allocated externally
 @param x resulting cartesian x matrix, must have width*height elements, allocated externally
 @param y resulting cartesian y matrix, must have width*height elements, allocated externally
 @param z resulting cartesian z matrix, must have width*height elements, allocated externally
 @param u16Dist encoded distance matrix as returned from O3R
 @param distResolution the resolution of the distance matrix as returned from O3R
 @param intrModelID the intrinsic modelID as returned from O3R
 @param intrModelParameters the intrinsic model parameters as returned from O3R
 @param extrTransX the extrinsic translation, X [m]
 @param extrTransY the extrinsic translation, Y [m]
 @param extrTransZ the extrinsic translation, Z [m]
 @param extrRotX the extrinsic rotation around X axis [rad]
 @param extrRotY the extrinsic rotation around Y axis [rad]
 @param extrRotZ the extrinsic rotation around Z axis [rad]
 @param width the width of the images
 @param height the height of the images
 */
int32_t xyzdFromDistance(float dist[], float x[], float y[], float z[], 
                         const uint16_t u16Dist[], float distResolution,
                         uint32_t intrModelID, const float intrModelParameters[],
                         float extrTransX, float extrTransY, float extrTransZ,
                         float extrRotX, float extrRotY, float extrRotZ,
                         uint16_t width, uint16_t height);
                         
/**
 @brief c code example for converting the distanceNoise matrix to [m].

 @param distanceNoise resulting distance noise matrix, must have width*height elements, allocated externally
 @param u16DistanceNoise encoded distance noise matrix as returned from O3R
 @param distResolution the resolution of the distance matrix as returned from O3R
 @param width the width of the images
 @param height the height of the images
 */
int32_t convertDistanceNoise(float distanceNoise[], const uint16_t u16DistanceNoise[], float distResolution,
                             uint16_t width, uint16_t height);            

/**
 @brief c code example for converting the encoded amplitude matrix.

 @param amplitude resulting amplitude matrix, must have width*height elements, allocated externally
 @param u16Amplitude encoded amplitude as returned from O3R
 @param amplitudeResolution the resolution of the distance matrix as returned from O3R
 @param width the width of the images
 @param height the height of the images
 */
int32_t convertAmplitude(float amplitude[], const uint16_t u16Amplitude[], float amplitudeResolution, 
                         uint16_t width, uint16_t height);

/** @brief c code example for evalIntrinsic function 
    
    @param vx the x component of the unit vectors, array must have width*height elements, allocated from outside
    @param vy the y component of the unit vectors, array must have width*height elements, allocated from outside
    @param vz the z component of the unit vectors, array must have width*height elements, allocated from outside
    @param modelID the modelID as returned from O3R
    @param modelParameters the model parameters as returned from O3R
    @param width the width of the images as returned from O3R
    @param height the height of the images as returned from O3R
    @return zero on success, non-zero on error
*/
int32_t evalIntrinsic(float vx[], float vy[], float vz[], 
                      uint32_t modelID, const float modelParameters[], uint16_t width, uint16_t height);

/**
 @brief c code example for converting the euler angles to a rotation matrix.

 @param R resulting 3x3 rotation matrix, allocated externally
 @param rotX rotation around X axis [rad]
 @param rotY rotation around Y axis [rad]
 @param rotZ rotation around Z axis [rad]
 */
int32_t rotMatFromAngles(float R[][3], float rotX, float rotY, float rotZ);

}

#endif
