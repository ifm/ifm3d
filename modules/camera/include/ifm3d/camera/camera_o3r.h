#ifndef IFM3D_CAMERA_CAMERA_O3R_H
#define IFM3D_CAMERA_CAMERA_O3R_H

#include <ifm3d/camera/camera_base.h>

namespace ifm3d
{
  /**
   * Camera specialization for O3R
   */
  class O3RCamera : public CameraBase
  {
  public:
    using Ptr = std::shared_ptr<O3RCamera>;
    O3RCamera(const std::string& ip = ifm3d::DEFAULT_IP,
              const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT);

    virtual ~O3RCamera();
    O3RCamera(O3RCamera&&) = delete;
    O3RCamera& operator=(O3RCamera&&) = delete;
    O3RCamera(O3RCamera&) = delete;
    O3RCamera& operator=(O3RCamera&) = delete;

    bool IsO3X() override;
    bool IsO3D() override;
    bool IsO3R() override;

    json ToJSON() override;
    void FromJSON(const json& j) override;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  }; // end: class O3RCamera
}
#endif // IFM3D_CAMERA_CAMERA_O3R_H