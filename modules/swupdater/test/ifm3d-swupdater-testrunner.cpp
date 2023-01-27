#include <ifm3d/swupdater.h>
#include <gtest/gtest.h>

int
main(int argc, char** argv)
{
  std::map<std::string, std::string> device_to_filter(
    {{"O3D", "SWUpdater.*"},
     {"O3X", "SWUpdater.*"},
     {"O3R",
      "SWUpdater.*:-SWUpdater.FactoryDefaults:-SWUpdater.FlashEmptyFile"},
     {"device_independent", ""}});

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
