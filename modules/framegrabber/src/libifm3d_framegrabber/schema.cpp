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

#include <ifm3d/fg/schema.h>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>
#include <ifm3d/camera/util.h>

const std::uint16_t ifm3d::IMG_RDIS     = (1<<0); // 2**0
const std::uint16_t ifm3d::IMG_AMP      = (1<<1); // 2**1
const std::uint16_t ifm3d::IMG_RAMP     = (1<<2); // 2**2
const std::uint16_t ifm3d::IMG_CART     = (1<<3); // 2**3
const std::uint16_t ifm3d::IMG_UVEC     = (1<<4); // 2**4
const std::uint16_t ifm3d::EXP_TIME     = (1<<5); // 2**5
const std::uint16_t ifm3d::IMG_GRAY     = (1<<6); // 2**6
const std::uint16_t ifm3d::ILLU_TEMP    = (1<<7); // 2**7
const std::uint16_t ifm3d::INTR_CAL     = (1<<8); // 2**8
const std::uint16_t ifm3d::INV_INTR_CAL = (1<<9); // 2**9
const std::uint16_t ifm3d::JSON_MODEL   = (1<<10); // 2**10

auto __ifm3d_schema_mask__ = []()->std::uint16_t
  {
    try
      {
        return std::getenv("IFM3D_MASK") == nullptr ?
            ifm3d::IMG_AMP|ifm3d::IMG_CART :
            std::stoul(std::string(std::getenv("IFM3D_MASK"))) & 0xFFFF;
      }
    catch (const std::exception& /*ex*/)
      {
        return ifm3d::IMG_AMP|ifm3d::IMG_CART;
      }
  };

const std::uint16_t ifm3d::DEFAULT_SCHEMA_MASK = __ifm3d_schema_mask__();

std::string
ifm3d::make_o3x_json_from_mask(std::uint16_t mask)
{
  std::string schema =
  R"(
      {
         "Apps":
         [
           {
             "Index":"1")";

  if((mask & ifm3d::IMG_RDIS) == ifm3d::IMG_RDIS)
    {
      schema +=
      R"(,
             "OutputDistanceImage":"true")";
    }
  else
    {
      schema +=
      R"(,
             "OutputDistanceImage":"false")";
    }

  if((mask & ifm3d::IMG_AMP) == ifm3d::IMG_AMP)
    {
      schema +=
      R"(,
             "OutputAmplitudeImage":"true")";
    }
  else
    {
      schema +=
      R"(,
             "OutputAmplitudeImage":"false")";
    }

  if((mask & ifm3d::IMG_GRAY) == ifm3d::IMG_GRAY)
    {
      schema +=
      R"(,
             "OutputGrayscaleImage":"true")";
    }
  else
    {
      schema +=
      R"(,
             "OutputGrayscaleImage":"false")";
    }

  if((mask & ifm3d::IMG_CART) == ifm3d::IMG_CART)
    {
      schema +=
      R"(,
             "OutputXYZImage":"true")";
    }
  else
    {
      schema +=
      R"(,
             "OutputXYZImage":"false")";
    }

// Note: this is not yet supported by o3x
//  if((mask & ifm3d::ILLU_TEMP) == ifm3d::ILLU_TEMP)
//    {
//      schema +=
//      R"(,
//             "OutputIlluminatorTemperature":"true")";
//    }
//  else
//    {
//      schema +=
//      R"(,
//             "OutputIlluminatorTemperature":"false")";
//    }

  // other invariants
  schema +=
  R"(,
             "OutputConfidenceImage":"true"
            }
         ]
      }
   )";

  return schema;
}

std::string
ifm3d::make_schema(std::uint16_t mask)
{
  std::string schema =
  R"(
      {
        "layouter": "flexible",
        "format"  : {"dataencoding":"ascii"},
        "elements":
         [
           {"type":"string", "value":"star", "id":"start_string"})";

  if((mask & ifm3d::IMG_RDIS) == ifm3d::IMG_RDIS)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"distance_image"})";
    }

  if((mask & ifm3d::IMG_AMP) == ifm3d::IMG_AMP)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"normalized_amplitude_image"})";
    }

  if((mask & ifm3d::IMG_RAMP) == ifm3d::IMG_RAMP)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"amplitude_image"})";
    }

  if((mask & ifm3d::IMG_GRAY) == ifm3d::IMG_GRAY)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"grayscale_image"})";
    }

  if((mask & ifm3d::IMG_CART) == ifm3d::IMG_CART)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"x_image"},
           {"type":"blob", "id":"y_image"},
           {"type":"blob", "id":"z_image"})";
    }

  if((mask & ifm3d::IMG_UVEC) == ifm3d::IMG_UVEC)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"all_unit_vector_matrices"})";
    }

  // intrinsic calibration
  if ((mask & ifm3d::INTR_CAL) == ifm3d::INTR_CAL)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"intrinsic_calibration"})";
    }

  // intrinsic calibration
  if ((mask & ifm3d::INV_INTR_CAL) == ifm3d::INV_INTR_CAL)
    {
      schema +=
      R"(,
           {"type":"blob", "id":"inverse_intrinsic_calibration"})";
    }

  if ((mask & ifm3d::JSON_MODEL) == ifm3d::JSON_MODEL)
    {
      schema +=
        R"(,
             {"type":"blob", "id":"json_model"})";
    }

  // confidence_image and extrinsics are invariant
  schema +=
    R"(,
           {"type":"blob", "id":"confidence_image"},
           {"type":"blob", "id":"extrinsic_calibration"})";

  if((mask & ifm3d::EXP_TIME) == ifm3d::EXP_TIME)
    {
      schema +=
      R"(,
           {"type":"string", "id":"exposure_times", "value":"extime"},
           {
            "type":"uint32", "id":"exposure_time_1",
            "format":{"dataencoding":"binary", "order":"little"}
           },
           {
             "type":"uint32", "id":"exposure_time_2",
             "format":{"dataencoding":"binary", "order":"little"}
           },
           {
             "type":"uint32", "id":"exposure_time_3",
             "format":{"dataencoding":"binary", "order":"little"}
           })";
    }

  if((mask & ifm3d::ILLU_TEMP) == ifm3d::ILLU_TEMP)
    {
      schema +=
      R"(,
           {"type":"string", "id":"temp_illu", "value":"temp_illu"},
           {
            "type":"float32", "id":"temp_illu",
            "format":{"dataencoding":"binary", "order":"little"}
           })";
    }

  // other invariants
  schema +=
  R"(,
           {"type":"string", "value":"stop", "id":"end_string"}
         ]
      }
   )";

  return schema;
}

std::uint16_t
ifm3d::schema_mask_from_string(const std::string& in)
{
  std::uint16_t mask = 0;
  std::vector<std::string> mask_parts = ifm3d::split(in, '|');
  for (auto part : mask_parts)
    {
      ifm3d::trim(part);
      if (part == "IMG_RDIS")
        {
          mask |= ifm3d::IMG_RDIS;
        }
      else if (part == "IMG_AMP")
        {
          mask |= ifm3d::IMG_AMP;
        }
      else if (part == "IMG_RAMP")
        {
          mask |= ifm3d::IMG_RAMP;
        }
      else if (part == "IMG_GRAY")
        {
          mask |= ifm3d::IMG_GRAY;
        }
      else if (part == "IMG_CART")
        {
          mask |= ifm3d::IMG_CART;
        }
      else if (part == "IMG_UVEC")
        {
          mask |= ifm3d::IMG_UVEC;
        }
      else if (part == "EXP_TIME")
        {
          mask |= ifm3d::EXP_TIME;
        }
      else if (part == "ILLU_TEMP")
        {
          mask |= ifm3d::ILLU_TEMP;
        }
      else if (part == "INTR_CAL")
        {
          mask |= ifm3d::INTR_CAL;
        }
      else if (part == "INV_INTR_CAL")
        {
          mask |= ifm3d::INV_INTR_CAL;
        }
      else if (part == "JSON_MODEL")
      {
        mask |= ifm3d::JSON_MODEL;
      }
    }
  return mask;
}
