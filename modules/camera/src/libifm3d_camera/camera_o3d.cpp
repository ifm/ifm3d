#include <ifm3d/camera/camera_o3d.h>

//================================================
// O3DCamera class - the public interface
//================================================

ifm3d::O3DCamera::O3DCamera(const std::string& ip,
                            const std::uint16_t xmlrpc_port,
                            const std::string& password)
  : ifm3d::Camera::Camera(ip, xmlrpc_port, password)
{}

ifm3d::O3DCamera::~O3DCamera() = default;

std::unordered_map<std::string, std::string>
ifm3d::O3DCamera::TimeInfo()
{
  if (this->CheckMinimumFirmwareVersion(ifm3d::O3D_TIME_SUPPORT_MAJOR,
                                        ifm3d::O3D_TIME_SUPPORT_MINOR,
                                        ifm3d::O3D_TIME_SUPPORT_PATCH))
    {
      return ifm3d::Camera::TimeInfo();
    }
  return json::parse("{}");
}

bool
ifm3d::O3DCamera::IsO3X()
{
  return false;
}

bool
ifm3d::O3DCamera::IsO3R()
{
  return false;
}

bool
ifm3d::O3DCamera::IsO3D()
{
  return true;
}