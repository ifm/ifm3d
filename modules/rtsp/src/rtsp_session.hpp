/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTSP_SESSION_HPP
#define IFM3D_RTSP_RTSP_SESSION_HPP

/** @file
 * @brief RTSP/1.0 control session over asio TCP (RFC 2326).
 *
 * Performs the DESCRIBE/SETUP/PLAY/TEARDOWN handshake, frames interleaved
 * RTP/RTCP data, and keeps the session alive with periodic GET_PARAMETER
 * requests. UDP and interleaved-TCP transports are both supported.
 */

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <asio.hpp>

#include "byte_span.hpp"
#include "rtcp_client.hpp"
#include "rtp_client.hpp"
#include "sdp_parser.hpp"

namespace ifm3d::rtsp
{
  /** A parsed RTSP response message. */
  struct RtspMessage
  {
    std::string rtsp_version;
    int status_code = -1;
    std::string status_message;
    std::map<std::string, std::string> headers; // lower-cased keys
    std::vector<std::uint8_t> content;
  };

  class RtspSession
  {
  public:
    enum class TransportType : std::uint8_t
    {
      INTERLEAVED = 0,
      UDP = 1,
    };

    using ChannelListener =
      std::function<void(std::uint8_t, const std::vector<std::uint8_t>&)>;

    explicit RtspSession(asio::io_context& ctx);
    ~RtspSession();

    RtspSession(const RtspSession&) = delete;
    RtspSession& operator=(const RtspSession&) = delete;
    RtspSession(RtspSession&&) = delete;
    RtspSession& operator=(RtspSession&&) = delete;

    /** Connect to the server and run the RTSP handshake. */
    void InitConnection(const std::string& addr,
                        std::uint16_t port,
                        TransportType transport,
                        const std::string& path);

    /** Send TEARDOWN (if a session is active) and close the connection. */
    void Teardown();

    RtpClient&
    GetRtpClient()
    {
      return _rtp_client;
    }
    RtcpClient&
    GetRtcpClient()
    {
      return _rtcp_client;
    }

    /** Write an interleaved RTP/RTCP channel frame back to the server. */
    void WriteChannelData(std::uint8_t channel, ByteSpan data);

    /** Register a listener for interleaved channel data. */
    void AddChannelListener(int channel, ChannelListener listener);

    // Callbacks (invoked on the io_context thread).
    std::function<void(int)> on_error;
    std::function<void(const SdpInfo&)> on_sdp;
    std::function<void()> on_playing;

  private:
    void send_message(std::string msg,
                      std::function<void(const RtspMessage&)> cb = nullptr);
    void do_setup(TransportType transport, const std::string& track_uri);
    void start_read();
    void process_buffer();
    void close_connection();
    bool report_failed_status(const RtspMessage& resp,
                              const std::string& request);
    bool find_consecutive_free_ports(std::uint16_t& p1, std::uint16_t& p2);
    RtspMessage parse_message(const std::string& msg_text) const;
    void schedule_keep_alive();

    asio::io_context& _ctx;
    asio::ip::tcp::socket _socket;
    asio::steady_timer _keep_alive_timer;

    std::string _address;
    std::uint16_t _port = 0;
    std::string _rtsp_uri = "stream";
    std::string _session;
    int _cseq = 0;
    bool _closing = false;

    std::vector<std::uint8_t> _unused_data;
    std::array<std::uint8_t, 4096> _read_buffer{};
    std::function<void(const RtspMessage&)> _pending_callback;
    std::map<int, ChannelListener> _channel_listeners;

    RtpClient _rtp_client;
    RtcpClient _rtcp_client;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_RTSP_SESSION_HPP
