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
#include <ifm3d/fg/buffer.h>
#include <ifm3d/rtsp/frame_metadata.h>
#include <ifm3d/rtsp/module_rtsp.h>
#include <ifm3d/rtsp/nal_unit.h>

namespace ifm3d
{
  /** @ingroup RTSP
   *
   * @brief RTSP/1.0 client that streams H.264 video from an ifm device and,
   * optionally, decodes it into `ifm3d::Buffer` frames using a compiled-in
   * video decoder.
   *
   * The client mirrors the threading model of `ifm3d::FrameGrabber`: it owns a
   * single asio `io_context` running on a dedicated worker thread. All
   * user-supplied callbacks are invoked on that thread, so callbacks must not
   * block.
   *
   * Decoding is delegated to a `VideoDecoder` selected by the
   * `ifm3d::DecoderManager`; the core RTSP module never links against an H.264
   * decoder. When no decoder is available, or when the `"null"` decoder is
   * selected via `Config::decoder`, the client operates in NAL-only
   * mode and only the `OnNalUnit` callback fires.
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

    /** Pixel layout of decoded frames delivered to `OnNewFrame`. */
    enum class OutputFormat : std::uint8_t
    {
      /**
       * 3-channel RGB. The decoded YUV420P frame is converted to a
       * width x height x 3 `ifm3d::Buffer` of 8-bit RGB.
       */
      RGB = 0,
      /**
       * Raw planar I420 (YUV420P): a single-channel `ifm3d::Buffer` of
       * width x (height * 3 / 2), with the full-resolution Y plane followed
       * by the half-resolution U and V planes.
       */
      YUV420 = 1,
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
       * Optional decoder selection. When unset, the first available decoder is
       * used. Set to a decoder name substring (e.g. `"ffmpeg"`) to prefer a
       * specific decoder, or to `"null"` to explicitly disable decoding
       * (NAL-only mode).
       */
      std::optional<std::string> decoder;

      /**
       * Pixel layout of decoded frames delivered to `OnNewFrame`. Defaults to
       * `OutputFormat::RGB`.
       */
      OutputFormat output_format = OutputFormat::RGB;
    };

    using NewFrameCallback = std::function<void(ifm3d::Buffer)>;
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
     * @return a future that resolves once the stream is playing, or that
     * holds an `ifm3d::Error` on failure.
     */
    std::shared_future<void> Start();

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
     * Registers a callback invoked for each decoded frame. The `Buffer`
     * carries any RGB_INFO SEI metadata in its JSON metadata.
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
