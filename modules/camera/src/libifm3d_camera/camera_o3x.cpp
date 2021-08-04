#include <ifm3d/camera/camera_o3x.h>

//================================================
// O3XCamera class - the public interface
//================================================

ifm3d::O3XCamera::O3XCamera(const std::string& ip,
                            const std::uint16_t xmlrpc_port,
                            const std::string& password)
  : ifm3d::Camera::Camera(ip, xmlrpc_port, password)
{}

ifm3d::O3XCamera::~O3XCamera() = default;

bool
ifm3d::O3XCamera::IsO3X()
{
  return true;
}

bool
ifm3d::O3XCamera::IsO3D()
{
  return false;
}

bool
ifm3d::O3XCamera::IsO3R()
{
  return false;
}