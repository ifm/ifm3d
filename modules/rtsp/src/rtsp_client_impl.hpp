/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTSP_CLIENT_IMPL_HPP
#define IFM3D_RTSP_RTSP_CLIENT_IMPL_HPP

/** @file
 * @brief Private implementation of ifm3d::RtspClient.
 *
 * Owns a single asio io_context running on a dedicated worker thread (the
 * FrameGrabber threading model), drives the RTSP state machine, and wires the
 * control session to the RTP client, the H.264 depacketizer and the decoder
 * host.
 */

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <asio.hpp>

#include <ifm3d/rtsp/rtsp_client.h>

#include "decoder_host.hpp"
#include "h264_depacketizer.hpp"
#include "rtsp_session.hpp"
#include "sdp_parser.hpp"
#include "sei_parser.hpp"

namespace ifm3d
{
  class RtspClient::Impl
  {
  public:
    explicit Impl(ifm3d::Device::Ptr device, Config config);
    ~Impl();

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    std::shared_future<void> Start(const BufferIdList& buffers);
    std::shared_future<void> Stop();

    [[nodiscard]] bool IsRunning() const;
    [[nodiscard]] State GetState() const;

    void OnNewFrame(NewFrameCallback callback);
    void OnNalUnit(NalUnitCallback callback);
    void OnError(ErrorCallback callback);
    void OnStateChange(StateChangeCallback callback);

  private:
    using WorkGuard =
      asio::executor_work_guard<asio::io_context::executor_type>;

    void set_state(State state);
    void handle_error(int code, const std::string& message);
    void handle_sdp(const ifm3d::rtsp::SdpInfo& info);
    void resolve_start();
    void resolve_start_with_error(const ifm3d::Error& error);
    void emit_frame(ifm3d::rtsp::BufferMap buffers,
                    std::uint64_t timestamp_ns,
                    std::uint32_t frame_count);

    /** Resolve the host, port and stream path from the configuration. */
    void resolve_endpoint(std::string& addr,
                          std::uint16_t& port,
                          std::string& path) const;

    ifm3d::Device::Ptr _device;
    Config _config;

    asio::io_context _ctx;
    std::optional<WorkGuard> _work_guard;
    std::thread _worker;

    std::unique_ptr<ifm3d::rtsp::RtspSession> _session;
    std::shared_ptr<ifm3d::rtsp::H264Depacketizer> _depacketizer;
    std::unique_ptr<ifm3d::rtsp::DecoderHost> _decoder_host;

    struct PendingFrame
    {
      ifm3d::rtsp::BufferMap buffers;
      std::uint64_t timestamp_ns = 0;
      std::uint32_t frame_count = 0;
    };

    BufferIdList _requested_buffers;
    std::optional<ifm3d::rtsp::RgbInfoPayload> _pending_rgb_info;

    /**
     * Metadata of the access units handed to the decoder, keyed by the pts
     * they were submitted with, awaiting the frame they decode into.
     */
    std::map<std::uint64_t, PendingFrame> _pending_frames;
    std::uint64_t _decode_pts = 0;
    bool _warned_unmatched_frame = false;
    std::uint32_t _frame_count = 0;

    mutable std::mutex _state_mutex;
    State _state = State::IDLE;
    std::atomic<bool> _running{false};

    std::promise<void> _start_promise;
    std::shared_future<void> _start_future;
    bool _start_resolved = false;

    std::promise<void> _stop_promise;
    std::shared_future<void> _stop_future;

    NewFrameCallback _on_new_frame;
    NalUnitCallback _on_nal_unit;
    ErrorCallback _on_error;
    StateChangeCallback _on_state_change;
  };

} // namespace ifm3d

#endif // IFM3D_RTSP_RTSP_CLIENT_IMPL_HPP
