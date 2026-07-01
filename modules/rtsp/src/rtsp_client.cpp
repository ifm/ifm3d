/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/rtsp/rtsp_client.h>

#include <future>
#include <memory>
#include <string>
#include <utility>

#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>

#include "rtsp_client_impl.hpp"

namespace ifm3d
{
  namespace
  {
    RtspClient::Config
    apply_port_info(RtspClient::Config config, const ifm3d::PortInfo& port)
    {
      if (port.rtsp)
        {
          config.port = port.rtsp->control.tcp_port;
          if (!port.rtsp->media_endpoints.empty())
            {
              std::string path = port.rtsp->media_endpoints.front().path;
              if (!path.empty() && path.front() == '/')
                {
                  path.erase(path.begin());
                }
              config.stream_path = std::move(path);
            }
        }
      return config;
    }
  } // namespace

  RtspClient::RtspClient(ifm3d::Device::Ptr device)
    : RtspClient(std::move(device), Config{})
  {}

  RtspClient::RtspClient(ifm3d::Device::Ptr device, Config config)
    : _impl(std::make_unique<Impl>(std::move(device), std::move(config)))
  {}

  RtspClient::RtspClient(ifm3d::Device::Ptr device,
                         const ifm3d::PortInfo& port)
    : RtspClient(std::move(device), port, Config{})
  {}

  RtspClient::RtspClient(ifm3d::Device::Ptr device,
                         const ifm3d::PortInfo& port,
                         Config config)
    : RtspClient(std::move(device), apply_port_info(std::move(config), port))
  {}

  RtspClient::~RtspClient()
  {
    if (_impl)
      {
        _impl->Stop().wait();
      }
  }

  RtspClient::RtspClient(RtspClient&&) noexcept = default;
  RtspClient& RtspClient::operator=(RtspClient&&) noexcept = default;

  std::shared_future<void>
  RtspClient::Start()
  {
    return _impl->Start();
  }

  std::shared_future<void>
  RtspClient::Stop()
  {
    return _impl->Stop();
  }

  bool
  RtspClient::IsRunning() const
  {
    return _impl->IsRunning();
  }

  RtspClient::State
  RtspClient::GetState() const
  {
    return _impl->GetState();
  }

  void
  RtspClient::OnNewFrame(NewFrameCallback callback)
  {
    _impl->OnNewFrame(std::move(callback));
  }

  void
  RtspClient::OnNalUnit(NalUnitCallback callback)
  {
    _impl->OnNalUnit(std::move(callback));
  }

  void
  RtspClient::OnError(ErrorCallback callback)
  {
    _impl->OnError(std::move(callback));
  }

  void
  RtspClient::OnStateChange(StateChangeCallback callback)
  {
    _impl->OnStateChange(std::move(callback));
  }

} // namespace ifm3d
