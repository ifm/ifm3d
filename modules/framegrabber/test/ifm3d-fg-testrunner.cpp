#include <ifm3d/fg.h>
#include <gtest/gtest.h>

int
main(int argc, char** argv)
{

  std::map<std::string, std::string> device_to_filter(
    {{"O3D",
      "FrameGrabberTest.*:-FrameGrabberTest.BlankSchema2D:FrameGrabberTest."
      "schema_o3r_rgb_image_info:FrameGrabberTest.schema_o3r_dist_image_info:"
      "FrameGrabberTest.only_algo_debug:"
      "FrameGrabberTest.algo_with_other_data:FrameGrabberTest."
      "DistanceNoiseImage_type:"
      "FrameGrabberTest.confidence_image_2D:FrameGrabberTest.onError:"
      "FrameGrabberTest.digonistic_data_grabber:FrameGrabberTest.portinfo"},
     {"O3X",
      "FrameGrabberTest.*:-FrameGrabberTest.BlankSchema2D:"
      "FrameGrabberTest.schema_o3r_rgb_image_info:FrameGrabberTest.schema_"
      "o3r_dist_image_info:FrameGrabberTest.only_algo_debug:FrameGrabberTest."
      "algo_with_other_data:"
      "FrameGrabberTest.SWTriggerMultipleClients:FrameGrabberTest."
      "confidence_image_2D::FrameGrabberTest.JSON_model:FrameGrabberTest."
      "onError:FrameGrabberTest.digonistic_data_grabber:FrameGrabberTest."
      "portinfo"},
     {"O3R",
      "FrameGrabberTest.*:-FrameGrabberTest.FactoryDefaults:FrameGrabberTest."
      "SoftwareTrigger:FrameGrabberTest."
      "SWTriggerMultipleClients:FrameGrabberTest.JSON_model"},
     {"device_independent", "DistanceImageInfo.*:Buffer.*:Schema.*"}});

  std::map<ifm3d::Device::device_family, std::string> device_family_to_device(
    {{ifm3d::Device::device_family::O3D, "O3D"},
     {ifm3d::Device::device_family::O3X, "O3X"},
     {ifm3d::Device::device_family::O3R, "O3R"}});

  std::string gtest_filter = device_to_filter["device_independent"];

  auto get_connected_device = [&]() -> std::string {
    try
      {
        auto cam = ifm3d::Device::MakeShared();
        return device_family_to_device[cam->WhoAmI()];
      }
    catch (...)
      {
        return "";
      }
  };

  gtest_filter += ":" + device_to_filter[get_connected_device()];

  ::testing::GTEST_FLAG(filter) = gtest_filter.c_str();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
