#include <ifm3d/camera/camera_o3r.h>
#include <camera_o3r_impl.hpp>

//================================================
// O3RCamera class - the public interface
//================================================

ifm3d::O3RCamera::O3RCamera(const std::string& ip,
                            const std::uint16_t xmlrpc_port)
  : ifm3d::CameraBase::CameraBase(ip, xmlrpc_port),
    pImpl(new ifm3d::O3RCamera::Impl(XWrapper()))
{}

ifm3d::O3RCamera::~O3RCamera() = default;

bool
ifm3d::O3RCamera::IsO3X()
{
  return false;
}

bool
ifm3d::O3RCamera::IsO3D()
{
  return false;
}

bool
ifm3d::O3RCamera::IsO3R()
{
  return true;
}

void
ifm3d::O3RCamera::FromJSON(const json& j)
{
  this->pImpl->SetTemporaryConfiguration(j.dump());
  this->pImpl->SaveInitConfiguration();
}

json
ifm3d::O3RCamera::ToJSON()
{
  return json::parse(this->pImpl->GetTemporaryConfiguration());
}