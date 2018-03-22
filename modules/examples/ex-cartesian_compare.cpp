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
// ex-cartesian_compare.cpp
//
// Computes the cartesian data from unit vectors, extrinsics, and
// radial depth image and compares this computation to the on-board camera's
// computation of the cartesian data.
//

#include <iostream>
#include <limits>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "ifm3d/camera.h"
#include "ifm3d/fg.h"
#include "ifm3d/image.h"

int main(int argc, const char** argv)
{
  // Get access to the camera and an image buffer to hold and organize the data
  // from the camera
  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();
  ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();

  // Initialize the framegrabber, with a schema that only streams in the unit
  // vectors and get the unit vectors. We only need to get them once.
  ifm3d::FrameGrabber::Ptr fg =
      std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_UVEC);
  if (!fg->WaitForFrame(img.get(), 10000))
    {
      std::cerr << "Failed to get unit vectors." << std::endl;
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

  // rotated unit vectors
  cv::Mat ex, ey, ez;
  std::vector<cv::Mat> uvec_channels(3);
  cv::split(img->UnitVectors(), uvec_channels);
  ex = uvec_channels[0];
  ey = uvec_channels[1];
  ez = uvec_channels[2];

  // Reinitialize the framegrabber with a schema that streams the radial
  // distance image (and for comparision only, the cartesian data). NOTE: The
  // extrinsics will be available as part of this schema as they are an
  // invariant.
  fg.reset(new ifm3d::FrameGrabber(cam, ifm3d::IMG_RDIS | ifm3d::IMG_CART));

  // Typically, the rest of this would be in a loop, but we are only going to
  // compute the cartesian data once.
  if (!fg->WaitForFrame(img.get(), 10000))
    {
      std::cerr << "Failed to get image data" << std::endl;
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

  // hold a reference to the radial distance image and convert to float
  cv::Mat rdis = img->DistanceImage();
  cv::Mat rdis_f;
  rdis.convertTo(rdis_f, CV_32FC1);

  // get a copy of the extrinsics
  std::vector<float> extrinsics = img->Extrinsics();

  // NOTE: The unit vectors are already rotated, so, we only need to extract
  // out the translation vector. The units are in mm.
  float tx = extrinsics[0];
  float ty = extrinsics[1];
  float tz = extrinsics[2];

  // Compute the cartesian data
  cv::Mat x_f = ex.mul(rdis_f) + tx;
  cv::Mat y_f = ey.mul(rdis_f) + ty;
  cv::Mat z_f = ez.mul(rdis_f) + tz;

  // explicitly set to zero any bad pixels
  cv::Mat mask;
  cv::Mat mask_ = rdis != 0;
  mask_ /= 255;
  mask_.convertTo(mask, CV_32FC1);
  x_f = x_f.mul(mask);
  y_f = y_f.mul(mask);
  z_f = z_f.mul(mask);

  // Transform from the IFM/O3D camera frame to the libifm3d/ROS camera frame
  cv::Mat x = z_f;
  cv::Mat y = -x_f;
  cv::Mat z = -y_f;

  // convert to meters
  if (cam->IsO3D())
    {
      x /= 1000.;
      y /= 1000.;
      z /= 1000.;
    }


  // construct and write out the computed cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      computed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  computed_cloud->width = x.cols;
  computed_cloud->height = x.rows;
  computed_cloud->points.resize(x.cols * x.rows);
  std::size_t idx = 0;
  for (std::size_t i = 0; i < x.rows; ++i)
    {
      for (std::size_t j = 0; j < x.cols; ++j)
        {
          computed_cloud->points[idx].x = x.at<float>(i, j);
          computed_cloud->points[idx].y = y.at<float>(i, j);
          computed_cloud->points[idx].z = z.at<float>(i, j);
          idx += 1;
        }
    }
  pcl::io::savePCDFileASCII("computed_cloud.pcd", *computed_cloud);

  // write out the on-board computed cloud ... note, we need to manually strip
  // out the intensity channel so we can clearly discern the reference points
  // from the computed points
  pcl::PointCloud<ifm3d::PointT>::Ptr cloud = img->Cloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      ref_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ref_cloud->width = cloud->width;
  ref_cloud->height = cloud->height;
  ref_cloud->points.resize(cloud->points.size());
  for (std::size_t i = 0; i < cloud->points.size(); ++i)
    {
      ref_cloud->points[i].x = cloud->points[i].x;
      ref_cloud->points[i].y = cloud->points[i].y;
      ref_cloud->points[i].z = cloud->points[i].z;
    }
  pcl::io::savePCDFileASCII("reference_cloud.pcd", *ref_cloud);

  return 0;
}
