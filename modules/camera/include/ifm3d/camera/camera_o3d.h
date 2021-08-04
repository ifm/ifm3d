#ifndef IFM3D_CAMERA_CAMERA_O3D_H
#define IFM3D_CAMERA_CAMERA_O3D_H

#include <ifm3d/camera/camera.h>

namespace ifm3d
{
  /**
   * Camera specialization for O3D
   */
  class O3DCamera : public Camera
  {
  public:
    using Ptr = std::shared_ptr<O3DCamera>;
    O3DCamera(const std::string& ip = ifm3d::DEFAULT_IP,
              const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
              const std::string& password = ifm3d::DEFAULT_PASSWORD);

    virtual ~O3DCamera();
    O3DCamera(O3DCamera&&) = delete;
    O3DCamera& operator=(O3DCamera&&) = delete;
    O3DCamera(O3DCamera&) = delete;
    O3DCamera& operator=(O3DCamera&) = delete;

    std::unordered_map<std::string, std::string> TimeInfo() override;
    bool IsO3X() override;
    bool IsO3D() override;
    bool IsO3R() override;
  }; // end: class O3DCamera
}
#endif // IFM3D_CAMERA_CAMERA_O3D_H