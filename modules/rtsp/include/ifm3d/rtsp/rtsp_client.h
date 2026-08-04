/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTSP_CLIENT_H
#define IFM3D_RTSP_RTSP_CLIENT_H

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ifm3d/common/err.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/rtsp/module_rtsp.h>
#include <ifm3d/rtsp/nal_unit.h>

namespace ifm3d
{
  /** @ingroup RTSP
   *
   * @brief RTSP/1.0 client that streams H.264 video from an ifm device and,
   * optionally, decodes it into `ifm3d::Frame` instances using a compiled-in
   * video decoder.
   *
   * The client mirrors the threading model of `ifm3d::FrameGrabber`: it owns a
   * single asio `io_context` running on a dedicated worker thread. All
   * user-supplied callbacks are invoked on that thread, so callbacks must not
   * block.
   *
   * Decoding is delegated to a `VideoDecoder` selected by the
   * `ifm3d::DecoderManager`; the core RTSP module never links against an H.264
   * decoder. The null decoder is selected automatically when no decoded image
   * buffer is requested.
   */
  class IFM3D_EXPORT RtspClient
  {
  public:
    using Ptr = std::shared_ptr<RtspClient>;

    /** RTP transport used for the media stream. */
    enum class Transport : std::uint8_t
    {
      /** RTP/AVP/TCP, interleaved on the RTSP control connection. */
      INTERLEAVED = 0,
      /** RTP/AVP over UDP. */
      UDP = 1,
    };

    /** Lifecycle state of the RTSP session. */
    enum class State : std::uint8_t
    {
      IDLE = 0,
      CONNECTING = 1,
      READY = 2,
      PLAYING = 3,
      STOPPED = 4,
      FAILED = 5,
    };

    /** Configuration for an `RtspClient` instance. */
    struct Config
    {
      /**
       * Full RTSP URL override (e.g. `rtsp://192.168.0.69:8554/port1`).
       * When unset the URL is built from the device IP, `port` and
       * `stream_path`.
       */
      std::optional<std::string> url;

      /** RTSP server port, used when `url` is unset. */
      std::uint16_t port = 8554;

      /** Stream path appended to the device IP, used when `url` is unset. */
      std::string stream_path = "port1";

      /** RTP transport selection. */
      Transport transport = Transport::INTERLEAVED;

      /**
       * Override the decoder name used for decoding h264 video into buffers.
       * When unset, the first available decoder is used.
       */
      std::optional<std::string> decoder;
    };

    using BufferIdList = std::vector<ifm3d::buffer_id>;
    using NewFrameCallback = std::function<void(ifm3d::Frame::Ptr)>;
    using NalUnitCallback = std::function<void(const ifm3d::NalUnit&)>;
    using ErrorCallback = std::function<void(const ifm3d::Error&)>;
    using StateChangeCallback = std::function<void(State)>;

    /**
     * Constructs a client that streams from the given device using the default
     * configuration.
     *
     * @param[in] device device used to discover the stream IP.
     */
    explicit RtspClient(ifm3d::Device::Ptr device);

    /**
     * Constructs a client that streams from the given device.
     *
     * @param[in] device device used to discover the stream IP (unless
     * `config.url` is set).
     * @param[in] config session configuration.
     */
    RtspClient(ifm3d::Device::Ptr device, Config config);

    /**
     * Constructs a client for a specific port, deriving the RTSP port and
     * stream path from the port's advertised `RtspInfo` when available.
     *
     * @param[in] device device used to discover the stream IP.
     * @param[in] port port whose RTSP endpoint should be streamed.
     */
    RtspClient(ifm3d::Device::Ptr device, const ifm3d::PortInfo& port);

    /**
     * Constructs a client for a specific port, deriving the RTSP port and
     * stream path from the port's advertised `RtspInfo` when available.
     *
     * @param[in] device device used to discover the stream IP (unless
     * `config.url` is set).
     * @param[in] port port whose RTSP endpoint should be streamed.
     * @param[in] config session configuration.
     */
    RtspClient(ifm3d::Device::Ptr device,
               const ifm3d::PortInfo& port,
               Config config);

    virtual ~RtspClient();

    // copy-disabled, move-enabled (like FrameGrabber's PImpl)
    RtspClient(const RtspClient&) = delete;
    RtspClient& operator=(const RtspClient&) = delete;
    RtspClient(RtspClient&&) noexcept;
    RtspClient& operator=(RtspClient&&) noexcept;

    /**
     * Performs the RTSP handshake and starts streaming.
     *
     * @param[in] buffers buffer IDs to receive. Supported IDs are
     * `COMPRESSED_H264_FRAME`, `RGB_IMAGE`, `YUV420_IMAGE` and `RGB_INFO`
     *
     * @return a future that resolves once the stream is playing, or that
     * holds an `ifm3d::Error` on failure.
     */
    std::shared_future<void> Start(const BufferIdList& buffers = {});

    /**
     * Sends `TEARDOWN`, stops the worker thread and closes the socket.
     *
     * @return a future that resolves once the client has stopped.
     */
    std::shared_future<void> Stop();

    /** @return true while the worker thread is running. */
    [[nodiscard]] bool IsRunning() const;

    /** @return the current session state. */
    [[nodiscard]] State GetState() const;

    /**
     * Registers a callback invoked for each received frame.
     */
    void OnNewFrame(NewFrameCallback callback = nullptr);

    /** Registers a callback invoked for each reassembled NAL unit. */
    void OnNalUnit(NalUnitCallback callback = nullptr);

    /** Registers a callback invoked on terminal errors. */
    void OnError(ErrorCallback callback = nullptr);

    /** Registers a callback invoked on every session state transition. */
    void OnStateChange(StateChangeCallback callback = nullptr);

  private:
    class Impl;
    std::unique_ptr<Impl> _impl;
  }; // end: class RtspClient

} // namespace ifm3d

#endif // IFM3D_RTSP_RTSP_CLIENT_H
