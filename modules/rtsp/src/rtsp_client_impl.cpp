/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtsp_client_impl.hpp"

#include <algorithm>
#include <array>
#include <asio/executor_work_guard.hpp>
#include <asio/post.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ifm3d/common/err.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/frame.h>
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
    /* Upper bound on access units parked waiting for their decoded frame.
     * Real decoder lag is a handful of frames (2 for the O3R/O3C SPS), so
     * anything beyond this is an access unit that will never decode and is
     * evicted to keep the wait bounded. */
    constexpr std::size_t MAX_PENDING_FRAMES = 32;

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

    bool
    contains(const RtspClient::BufferIdList& buffers, ifm3d::buffer_id id)
    {
      return std::find(buffers.begin(), buffers.end(), id) != buffers.end();
    }

    bool
    is_supported(ifm3d::buffer_id id)
    {
      return id == ifm3d::buffer_id::COMPRESSED_H264_FRAME ||
             id == ifm3d::buffer_id::RGB_IMAGE ||
             id == ifm3d::buffer_id::YUV420_IMAGE ||
             id == ifm3d::buffer_id::RGB_INFO;
    }

    ifm3d::Buffer
    make_blob(const std::vector<std::uint8_t>& data, ifm3d::buffer_id id)
    {
      ifm3d::Buffer buffer(static_cast<std::uint32_t>(data.size()),
                           1,
                           1,
                           ifm3d::PixelFormat::FORMAT_8U,
                           std::nullopt,
                           id);
      std::copy(data.begin(), data.end(), buffer.Ptr<std::uint8_t>(0));
      return buffer;
    }

    std::uint64_t
    system_timestamp_ns()
    {
      return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
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
  RtspClient::Impl::Start(const BufferIdList& buffers)
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
    _requested_buffers =
      buffers.empty() ? BufferIdList{ifm3d::buffer_id::COMPRESSED_H264_FRAME,
                                     ifm3d::buffer_id::RGB_IMAGE,
                                     ifm3d::buffer_id::YUV420_IMAGE,
                                     ifm3d::buffer_id::RGB_INFO} :
                        buffers;
    _pending_rgb_info.reset();
    _pending_frames.clear();
    _decode_pts = 0;
    _warned_unmatched_frame = false;
    _frame_count = 0;

    const auto unsupported =
      std::find_if(_requested_buffers.begin(),
                   _requested_buffers.end(),
                   [](ifm3d::buffer_id id) { return !is_supported(id); });
    if (unsupported != _requested_buffers.end())
      {
        _running.store(false);
        handle_error(
          IFM3D_RTSP_ERROR,
          "unsupported RTSP buffer id: " +
            std::to_string(static_cast<std::uint64_t>(*unsupported)));
        return _start_future;
      }

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
      const bool request_rgb =
        contains(_requested_buffers, ifm3d::buffer_id::RGB_IMAGE);
      const bool request_yuv =
        contains(_requested_buffers, ifm3d::buffer_id::YUV420_IMAGE);
      _decoder_host =
        std::make_unique<ifm3d::rtsp::DecoderHost>(_config.decoder,
                                                   request_rgb,
                                                   request_yuv);
      _decoder_host->on_frame = [this](ifm3d::rtsp::BufferMap buffers,
                                       std::uint64_t pts) {
        const auto match = _pending_frames.find(pts);
        if (match == _pending_frames.end())
          {
            // Only reachable if the decoder invented a pts we never sent, or
            // if we already evicted this one as too old to still be in
            // flight. Either way the image cannot be labelled, so drop it.
            if (!_warned_unmatched_frame)
              {
                _warned_unmatched_frame = true;
                LOG_WARNING("RtspClient: decoded frame carries pts {}, which "
                            "matches no outstanding access unit; dropping it",
                            pts);
              }
            return;
          }
        PendingFrame pending = std::move(match->second);
        _pending_frames.erase(match);
        pending.buffers.merge(buffers);
        emit_frame(std::move(pending.buffers),
                   pending.timestamp_ns,
                   pending.frame_count);
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
    _pending_frames.clear();

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
  RtspClient::Impl::emit_frame(ifm3d::rtsp::BufferMap buffers,
                               std::uint64_t timestamp_ns,
                               std::uint32_t frame_count)
  {
    if (!_on_new_frame)
      {
        return;
      }

    const std::vector<ifm3d::TimePointT> timestamps = {
      ifm3d::TimePointT{std::chrono::nanoseconds{timestamp_ns}}};
    ifm3d::BufferDataListMap frame_buffers;
    for (auto& [id, buffer] : buffers)
      {
        frame_buffers.emplace(id, ifm3d::BufferList{std::move(buffer)});
      }
    _on_new_frame(
      std::make_shared<ifm3d::Frame>(frame_buffers,
                                     timestamps,
                                     static_cast<std::uint64_t>(frame_count)));
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
            _pending_rgb_info = std::move(*rgb_info);
          }
      };

    // The SEI belongs to the access unit it arrived in, so it must not
    // outlive it. Without this a gap that discards an access unit after its
    // SEI would hand that timestamp and frame counter to whichever access
    // unit is emitted next.
    _depacketizer->on_access_unit_cancelled = [this] {
      _pending_rgb_info.reset();
    };

    _depacketizer->on_access_unit =
      [this](const std::vector<std::uint8_t>& access_unit,
             std::uint64_t /*pts_us*/) {
        // The RTP presentation timestamp is deliberately unused. It is a
        // 90 kHz counter starting from a random base (RFC 3550 5.1) that
        // wraps every ~13.25 hours, and mapping it onto a wall clock needs
        // the NTP pairing from an RTCP sender report, which is not parsed.
        // The device confirms this: its first access unit arrives at roughly
        // 28455 s, some 56 years short of the epoch. The frame timestamp
        // therefore comes from the RGB-info SEI, or from the host clock when
        // there is none, and decoded images are paired with their metadata
        // through the monotonic _decode_pts counter below.
        PendingFrame pending;
        pending.timestamp_ns = system_timestamp_ns();
        pending.frame_count = ++_frame_count;

        if (contains(_requested_buffers,
                     ifm3d::buffer_id::COMPRESSED_H264_FRAME))
          {
            pending.buffers.emplace(
              ifm3d::buffer_id::COMPRESSED_H264_FRAME,
              make_blob(access_unit, ifm3d::buffer_id::COMPRESSED_H264_FRAME));
          }

        if (_pending_rgb_info)
          {
            pending.timestamp_ns = _pending_rgb_info->timestamp_ns;
            pending.frame_count = _pending_rgb_info->frame_counter;
            if (contains(_requested_buffers, ifm3d::buffer_id::RGB_INFO))
              {
                pending.buffers.emplace(ifm3d::buffer_id::RGB_INFO,
                                        make_blob(_pending_rgb_info->data,
                                                  ifm3d::buffer_id::RGB_INFO));
              }
          }
        _pending_rgb_info.reset();

        if (_decoder_host && _decoder_host->ProducesFrames())
          {
            // The decoder may hand this access unit's frame back several
            // submissions from now, so the metadata is parked under the pts
            // it was submitted with and reunited with the image in
            // `on_frame`. Access units that never decode at all -- the ones
            // preceding the first keyframe when a stream is joined mid-GOP,
            // or ones left incomplete by packet loss -- would otherwise sit
            // here forever, so the oldest entries are evicted once the map
            // grows past any plausible decoder lag.
            const std::uint64_t pts = ++_decode_pts;
            _pending_frames.emplace(pts, std::move(pending));
            while (_pending_frames.size() > MAX_PENDING_FRAMES)
              {
                LOG_WARNING(
                  "RtspClient: dropping the access unit with pts {}: more "
                  "than {} access units are waiting to be decoded. The "
                  "decoder is not keeping up or the stream cannot be decoded "
                  "(no keyframe yet, or packet loss)",
                  _pending_frames.begin()->first,
                  MAX_PENDING_FRAMES);
                _pending_frames.erase(_pending_frames.begin());
              }
            _decoder_host->SubmitAccessUnit(access_unit, pts);
          }
        else
          {
            emit_frame(std::move(pending.buffers),
                       pending.timestamp_ns,
                       pending.frame_count);
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
