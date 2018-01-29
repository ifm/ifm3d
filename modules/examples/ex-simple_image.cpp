/*
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

//
// ex-simple_image.cpp
//
// Capture a frame from the camera, and process the image data. For
// exemplary purposes, we print out information about the img such as
// the dimension and data type of the pixels
//

#include <iostream>
#include <memory>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/simple_image.h>
#include <boost/type_index.hpp>

/**
* Read sizeof(T) bytes from the data buffer
* and interpret the result as a value of type T
*/
template <typename T>
T get_val(const std::uint8_t* data)
{
  union
  {
    T v;
    unsigned char bytes[sizeof(T)];
  } value;

  std::copy(data, data + sizeof(T), value.bytes);
  return value.v;
}

/**
* Converts the raw image data to a vector<T>
*/
template <typename T>
std::vector<T> read_as(const ifm3d::SimpleImageBuffer::Img& img)
{
  int num_channels = img.data.size() / (img.width * img.height * sizeof(T));
  std::vector<T> result(img.width * img.height * num_channels);

  for (std::size_t i = 0; i < result.size(); i++)
    {
      result[i] = get_val<T>(img.data.data() + (i * sizeof(T)));
    }

  return result;
}

/**
* Process the image data
* For this example, we just print out some information...
*/
template <typename T>
void process_data(std::string image_name, const ifm3d::SimpleImageBuffer::Img& img, const std::vector<T>& data)
{
  std::cout << "Processing image: " << image_name
            << " (width: " << img.width
            << ", height: " << img.height
            << ", num channels: " << data.size() / (img.width * img.height)
            << ", data_type: " << boost::typeindex::type_id<T>().pretty_name()
            << ")" << std::endl;
}

int main(int argc, const char **argv)
{
  auto cam = ifm3d::Camera::MakeShared("192.168.0.48");
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam,(ifm3d::IMG_AMP|ifm3d::IMG_RDIS|ifm3d::IMG_CART));
  auto buffer = std::make_shared<ifm3d::SimpleImageBuffer>();
  if (! fg->WaitForFrame(buffer.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }


  for (auto it : std::map<std::string, ifm3d::SimpleImageBuffer::Img>
  {
      {"Distance", buffer->DistanceImage()},
      {"Amplitude", buffer->AmplitudeImage()},
      {"Confidence", buffer->ConfidenceImage()},
      {"XYZImage", buffer->XYZImage()},
    })
  {
	const std::string& name = it.first;
	const ifm3d::SimpleImageBuffer::Img& img = it.second;
    switch (img.format)
      {
      case ifm3d::pixel_format::FORMAT_8U:
        process_data(name, img, read_as<std::uint8_t>(img));
        break;
      case ifm3d::pixel_format::FORMAT_8S:
        process_data(name, img, read_as<std::int8_t>(img));
        break;
      case ifm3d::pixel_format::FORMAT_16U:
        process_data(name, img, read_as<std::uint16_t>(img));
        break;
      case ifm3d::pixel_format::FORMAT_16S:
        process_data(name, img, read_as<std::int16_t>(img));
        break;
      case ifm3d::pixel_format::FORMAT_32U:
        process_data(name, img, read_as<std::uint32_t>(img));
        break;
      case ifm3d::pixel_format::FORMAT_32S:
        process_data(name, img, read_as<std::int32_t>(img));
        break;
      case ifm3d::pixel_format::FORMAT_64U:
        process_data(name, img, read_as<std::uint64_t>(img));
        break;
      case ifm3d::pixel_format::FORMAT_32F:
        process_data(name, img, read_as<float>(img));
        break;
      case ifm3d::pixel_format::FORMAT_64F:
        process_data(name, img, read_as<double>(img));
        break;
      }
  }

  return 0;
}
