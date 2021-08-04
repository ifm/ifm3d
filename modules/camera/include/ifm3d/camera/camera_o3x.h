#ifndef IFM3D_CAMERA_CAMERA_O3X_H
#define IFM3D_CAMERA_CAMERA_O3X_H

#include <ifm3d/camera/camera.h>

namespace ifm3d
{
  /**
   * Camera specialization for O3X
   */
  class O3XCamera : public Camera
  {
  public:
    using Ptr = std::shared_ptr<O3XCamera>;
    O3XCamera(const std::string& ip = ifm3d::DEFAULT_IP,
              const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
              const std::string& password = ifm3d::DEFAULT_PASSWORD);

    virtual ~O3XCamera();
    O3XCamera(O3XCamera&&) = delete;
    O3XCamera& operator=(O3XCamera&&) = delete;
    O3XCamera(O3XCamera&) = delete;
    O3XCamera& operator=(O3XCamera&) = delete;

    bool IsO3X() override;
    bool IsO3D() override;
    bool IsO3R() override;
  }; // end: class O3XCamera
}
#endif // IFM3D_CAMERA_CAMERA_O3X_H