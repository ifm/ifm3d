/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtsp_client_impl.hpp"

#include <array>
#include <asio/executor_work_guard.hpp>
#include <asio/post.hpp>
#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <ifm3d/common/err.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/rtsp/nal_unit.h>
#include <ifm3d/rtsp/rtsp_client.h>

#include "decoder_host.hpp"
#include "h264_depacketizer.hpp"
#include "rtsp_session.hpp"
#include "sdp_parser.hpp"
#include "sei_parser.hpp"

namespace ifm3d
{
  namespace
  {
    ifm3d::rtsp::RtspSession::TransportType
    to_session_transport(RtspClient::Transport transport)
    {
      switch (transport)
        {
        case RtspClient::Transport::UDP:
          return ifm3d::rtsp::RtspSession::TransportType::UDP;
        case RtspClient::Transport::INTERLEAVED:
        default:
          return ifm3d::rtsp::RtspSession::TransportType::INTERLEAVED;
        }
    }
  } // namespace

  RtspClient::Impl::Impl(ifm3d::Device::Ptr device, Config config)
    : _device(std::move(device)),
      _config(std::move(config))
  {
    _start_future = _start_promise.get_future().share();
    _stop_future = _stop_promise.get_future().share();
  }

  RtspClient::Impl::~Impl()
  {
    if (_running.load())
      {
        try
          {
            Stop().wait();
          }
        catch (const std::exception& ex)
          {
            LOG_DEBUG("RtspClient best-effort shutdown failed: {}", ex.what());
          }
        catch (...)
          {
            LOG_DEBUG("RtspClient best-effort shutdown failed: unknown error");
          }
      }
    if (_worker.joinable())
      {
        _worker.join();
      }
  }

  void
  RtspClient::Impl::resolve_endpoint(std::string& addr,
                                     std::uint16_t& port,
                                     std::string& path) const
  {
    if (_config.url && !_config.url->empty())
      {
        std::string url = *_config.url;
        const std::string scheme = "rtsp://";
        if (url.rfind(scheme, 0) == 0)
          {
            url = url.substr(scheme.size());
          }

        std::string authority = url;
        std::string url_path;
        const auto slash = url.find('/');
        if (slash != std::string::npos)
          {
            authority = url.substr(0, slash);
            url_path = url.substr(slash + 1);
          }

        std::string host = authority;
        std::uint16_t url_port = _config.port;
        const auto colon = authority.find(':');
        if (colon != std::string::npos)
          {
            host = authority.substr(0, colon);
            url_port = static_cast<std::uint16_t>(
              std::stoi(authority.substr(colon + 1)));
          }

        addr = host;
        port = url_port;
        path = url_path;
        return;
      }

    if (_device)
      {
        addr = _device->IP();
      }
    port = _config.port;
    path = _config.stream_path;
  }

  std::shared_future<void>
  RtspClient::Impl::Start()
  {
    if (_running.exchange(true))
      {
        return _start_future;
      }

    _start_promise = std::promise<void>();
    _start_future = _start_promise.get_future().share();
    _stop_promise = std::promise<void>();
    _stop_future = _stop_promise.get_future().share();
    _start_resolved = false;
    _pending_metadata.reset();

    std::string addr;
    std::uint16_t port = _config.port;
    std::string path = _config.stream_path;
    try
      {
        resolve_endpoint(addr, port, path);
      }
    catch (const std::exception& ex)
      {
        _running.store(false);
        handle_error(IFM3D_RTSP_ERROR,
                     std::string("invalid configuration: ") + ex.what());
        return _start_future;
      }

    if (addr.empty())
      {
        _running.store(false);
        handle_error(
          IFM3D_RTSP_ERROR,
          "no device or url provided to resolve the stream address");
        return _start_future;
      }

    {
      _decoder_host =
        std::make_unique<ifm3d::rtsp::DecoderHost>(_config.decoder,
                                                   _config.output_format);
      _decoder_host->on_frame = [this](ifm3d::Buffer buffer) {
        if (_on_new_frame)
          {
            _on_new_frame(std::move(buffer));
          }
      };
      _decoder_host->on_error = [this](int code, const std::string& msg) {
        handle_error(code, msg);
      };
      _decoder_host->Init();
    }

    _session = std::make_unique<ifm3d::rtsp::RtspSession>(_ctx);
    _session->on_error = [this](int code) {
      handle_error(code, ifm3d::Error(code).what());
    };
    _session->on_sdp = [this](const ifm3d::rtsp::SdpInfo& info) {
      handle_sdp(info);
    };
    _session->on_playing = [this]() {
      set_state(State::PLAYING);
      resolve_start();
    };

    set_state(State::CONNECTING);

    const auto transport = to_session_transport(_config.transport);
    _work_guard.emplace(asio::make_work_guard(_ctx));
    _ctx.restart();

    _worker = std::thread([this, addr, port, transport, path]() {
      try
        {
          _session->InitConnection(addr, port, transport, path);
          _ctx.run();
        }
      catch (const std::exception& ex)
        {
          LOG_ERROR("RtspClient worker terminated: {}", ex.what());
          handle_error(IFM3D_RTSP_ERROR, ex.what());
        }
    });

    return _start_future;
  }

  std::shared_future<void>
  RtspClient::Impl::Stop()
  {
    if (!_running.exchange(false))
      {
        if (_stop_future.valid() && _stop_future.wait_for(std::chrono::seconds(
                                      0)) == std::future_status::ready)
          {
            return _stop_future;
          }
        std::promise<void> resolved;
        resolved.set_value();
        _stop_future = resolved.get_future().share();
        return _stop_future;
      }

    asio::post(_ctx, [this]() {
      if (_session)
        {
          _session->Teardown();
        }
      _work_guard.reset();
      _ctx.stop();
    });

    if (_worker.joinable())
      {
        _worker.join();
      }

    set_state(State::STOPPED);

    try
      {
        _stop_promise.set_value();
      }
    catch (const std::future_error&)
      {
        // IGNORE: promise already satisfied
      }

    _session.reset();
    _depacketizer.reset();
    _decoder_host.reset();

    return _stop_future;
  }

  bool
  RtspClient::Impl::IsRunning() const
  {
    return _running.load();
  }

  RtspClient::State
  RtspClient::Impl::GetState() const
  {
    std::lock_guard<std::mutex> const lock(_state_mutex);
    return _state;
  }

  void
  RtspClient::Impl::OnNewFrame(NewFrameCallback callback)
  {
    _on_new_frame = std::move(callback);
  }

  void
  RtspClient::Impl::OnNalUnit(NalUnitCallback callback)
  {
    _on_nal_unit = std::move(callback);
  }

  void
  RtspClient::Impl::OnError(ErrorCallback callback)
  {
    _on_error = std::move(callback);
  }

  void
  RtspClient::Impl::OnStateChange(StateChangeCallback callback)
  {
    _on_state_change = std::move(callback);
  }

  void
  RtspClient::Impl::set_state(State state)
  {
    {
      std::lock_guard<std::mutex> const lock(_state_mutex);
      if (_state == state)
        {
          return;
        }
      _state = state;
    }
    if (_on_state_change)
      {
        _on_state_change(state);
      }
  }

  void
  RtspClient::Impl::handle_error(int code, const std::string& message)
  {
    LOG_ERROR("RtspClient error ({}): {}", code, message);
    set_state(State::FAILED);

    const ifm3d::Error error(code, message);
    resolve_start_with_error(error);

    if (_on_error)
      {
        _on_error(error);
      }
  }

  void
  RtspClient::Impl::resolve_start()
  {
    if (_start_resolved)
      {
        return;
      }
    _start_resolved = true;
    try
      {
        _start_promise.set_value();
      }
    catch (const std::future_error&)
      {
        // IGNORE: promise already satisfied
      }
  }

  void
  RtspClient::Impl::resolve_start_with_error(const ifm3d::Error& error)
  {
    if (_start_resolved)
      {
        return;
      }
    _start_resolved = true;
    try
      {
        _start_promise.set_exception(std::make_exception_ptr(error));
      }
    catch (const std::future_error&)
      {
        // IGNORE: promise already satisfied
      }
  }

  void
  RtspClient::Impl::handle_sdp(const ifm3d::rtsp::SdpInfo& info)
  {
    _depacketizer = std::make_shared<ifm3d::rtsp::H264Depacketizer>();

    _depacketizer->on_nal_unit = [this](const ifm3d::NalUnit& nal) {
      if (_on_nal_unit)
        {
          _on_nal_unit(nal);
        }
    };

    _depacketizer->on_sei_unregistered_user_data =
      [this](const std::array<std::uint8_t, 16>& uuid,
             const std::vector<std::uint8_t>& data) {
        if (auto rgb_info = ifm3d::rtsp::parse_rgb_info(uuid, data))
          {
            _pending_metadata = ifm3d::rtsp::rgb_info_to_json(*rgb_info);
          }
      };

    _depacketizer->on_access_unit =
      [this](const std::vector<std::uint8_t>& access_unit,
             std::uint64_t pts_us) {
        if (_decoder_host && _decoder_host->HasDecoder())
          {
            _decoder_host->SubmitAccessUnit(access_unit,
                                            pts_us,
                                            _pending_metadata);
          }
      };

    if (!info.sprop_parameter_sets.empty())
      {
        _depacketizer->SeedFromSprop(info.sprop_parameter_sets);
      }

    _session->GetRtpClient().RegisterDecoder(info.payload_type, _depacketizer);

    set_state(State::READY);
  }

} // namespace ifm3d
