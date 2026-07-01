/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtsp_session.hpp"

#include <algorithm>
#include <asio/buffer.hpp>
#include <asio/error.hpp>
#include <asio/error_code.hpp>
#include <asio/impl/connect.hpp>
#include <asio/impl/write.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/ip/udp.hpp>
#include <asio/socket_base.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <regex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ifm3d/common/err.h>
#include <ifm3d/common/logging/log.h>

#include "bit_reader_writer.hpp"
#include "byte_span.hpp"
#include "rtp_connection_interleaved.hpp"
#include "rtp_connection_udp.hpp"
#include "rtsp_util.hpp"
#include "sdp_parser.hpp"

namespace ifm3d::rtsp
{
  namespace
  {
    const std::string DESCRIBE_MSG =
      "DESCRIBE %STREAM_URI RTSP/1.0\r\nCSeq: %CSEQ\r\n"
      "Accept: application/sdp\r\n\r\n";

    const std::string SETUP_MSG_UDP =
      "SETUP %TRACK_URI RTSP/1.0\r\nCSeq: %CSEQ\r\n"
      "Transport: RTP/AVP;unicast;client_port=%1-%2\r\n\r\n";

    const std::string SETUP_MSG_INTERLEAVED =
      "SETUP %TRACK_URI RTSP/1.0\r\nCSeq: %CSEQ\r\n"
      "Transport: RTP/AVP/TCP;unicast;interleaved=%1-%2\r\n\r\n";

    const std::string PLAY_MSG =
      "PLAY %STREAM_URI RTSP/1.0\r\nCSeq: %CSEQ\r\nSession: %SESSION\r\n\r\n";

    const std::string TEARDOWN_MSG =
      "TEARDOWN %STREAM_URI RTSP/1.0\r\nCSeq: %CSEQ\r\n"
      "Session: %SESSION\r\n\r\n";

    const std::string KEEPALIVE_MSG =
      "GET_PARAMETER %STREAM_URI RTSP/1.0\r\nCSeq: %CSEQ\r\n"
      "Session: %SESSION\r\n\r\n";

    constexpr std::chrono::milliseconds KEEPALIVE_INTERVAL{5000};
    constexpr int INTERLEAVED_HEADER_SIZE = 4;
    constexpr std::size_t MAX_UNUSED_DATA = std::size_t{10} * 1024 * 1024;

    const std::regex SERVER_PORT_REGEX(R"(server_port=(\d+)-(\d+))");
    const std::regex SINGLE_SERVER_PORT_REGEX(R"(server_port=(\d+))");
    const std::regex HEADER_REGEX(R"(([\w-]+): ?(.*))");
    const std::regex RTSP_RESPONSE_REGEX(R"(RTSP\/([0-9\.]+) (\d+) ([\w ]*))");

    std::string
    fmt2(std::string s, const std::string& a1, const std::string& a2)
    {
      s = replace_all(std::move(s), "%1", a1);
      s = replace_all(std::move(s), "%2", a2);
      return s;
    }
  } // namespace

  RtspSession::RtspSession(asio::io_context& ctx)
    : _ctx(ctx),
      _socket(ctx),
      _keep_alive_timer(ctx)
  {}

  RtspSession::~RtspSession() { close_connection(); }

  void
  RtspSession::InitConnection(const std::string& addr,
                              std::uint16_t port,
                              TransportType transport,
                              const std::string& path)
  {
    _address = addr;
    _port = port;
    _rtsp_uri = "rtsp://" + addr + ":" + std::to_string(port) + "/" + path;

    asio::ip::tcp::resolver resolver(_ctx);
    asio::error_code ec;
    auto endpoints = resolver.resolve(addr, std::to_string(port), ec);
    if (ec)
      {
        LOG_ERROR("RtspSession: DNS resolution failed for {}:{} - {}",
                  addr,
                  port,
                  ec.message());
        if (on_error)
          {
            on_error(IFM3D_RTSP_CONNECTION_ERROR);
          }
        return;
      }

    asio::async_connect(
      _socket,
      endpoints,
      [this, transport](const asio::error_code& connect_ec,
                        const asio::ip::tcp::endpoint&) {
        if (connect_ec)
          {
            LOG_ERROR("RtspSession: connect failed: {}", connect_ec.message());
            if (on_error)
              {
                on_error(IFM3D_RTSP_CONNECTION_ERROR);
              }
            return;
          }

        start_read();
        schedule_keep_alive();

        send_message(
          DESCRIBE_MSG,
          [this, transport](const RtspMessage& desc_resp) {
            if (desc_resp.status_code != 200)
              {
                LOG_ERROR("RtspSession: DESCRIBE failed with status {}",
                          desc_resp.status_code);
                if (on_error)
                  {
                    on_error(IFM3D_RTSP_REQUEST_FAILED);
                  }
                close_connection();
                return;
              }

            const std::string sdp(desc_resp.content.begin(),
                                  desc_resp.content.end());
            const SdpInfo info = parse_sdp(sdp, _rtsp_uri);

            if (!info.has_h264)
              {
                LOG_ERROR("RtspSession: no H.264 video track found in SDP");
                if (on_error)
                  {
                    on_error(IFM3D_RTSP_NO_VIDEO_TRACK);
                  }
                close_connection();
                return;
              }

            LOG_DEBUG("RtspSession: track URL = {}, payload type = {}",
                      info.track_url,
                      info.payload_type);

            if (on_sdp)
              {
                on_sdp(info);
              }

            do_setup(transport, info.track_url);
          });
      });
  }

  void
  RtspSession::do_setup(TransportType transport, const std::string& track_uri)
  {
    if (transport == TransportType::UDP)
      {
        std::uint16_t rtp_port = 0;
        std::uint16_t rtcp_port = 0;
        if (!find_consecutive_free_ports(rtp_port, rtcp_port))
          {
            if (on_error)
              {
                on_error(IFM3D_RTSP_ERROR);
              }
            close_connection();
            return;
          }

        auto msg = fmt2(SETUP_MSG_UDP,
                        std::to_string(rtp_port),
                        std::to_string(rtcp_port));
        msg = replace_all(std::move(msg), "%TRACK_URI", track_uri);

        send_message(
          std::move(msg),
          [this, rtp_port, rtcp_port](const RtspMessage& resp) {
            if (auto sit = resp.headers.find("session");
                sit != resp.headers.end())
              {
                _session = sit->second.substr(0, sit->second.find(';'));
              }

            std::string transport_hdr;
            if (auto it = resp.headers.find("transport");
                it != resp.headers.end())
              {
                transport_hdr = it->second;
              }

            std::smatch sm;
            std::uint16_t srv_rtp = 0;
            std::uint16_t srv_rtcp = 0;
            if (std::regex_search(transport_hdr, sm, SERVER_PORT_REGEX))
              {
                srv_rtp = parse_port(sm[1].str());
                srv_rtcp = parse_port(sm[2].str());
              }
            else if (std::regex_search(transport_hdr,
                                       sm,
                                       SINGLE_SERVER_PORT_REGEX))
              {
                srv_rtp = parse_port(sm[1].str());
                srv_rtcp = srv_rtp;
              }

            if (_session.empty() || srv_rtp == 0)
              {
                LOG_ERROR(
                  "RtspSession: missing session/transport in SETUP (status "
                  "{})",
                  resp.status_code);
                if (on_error)
                  {
                    on_error(IFM3D_RTSP_REQUEST_FAILED);
                  }
                close_connection();
                return;
              }

            auto rtp_con = std::make_shared<RtpConnectionUdp>(_ctx,
                                                              rtp_port,
                                                              _address,
                                                              srv_rtp);
            rtp_con->EnablePiercingWatchDog();
            _rtp_client.InitConnection(rtp_con);

            _rtcp_client.InitConnection(
              std::make_shared<RtpConnectionUdp>(_ctx,
                                                 rtcp_port,
                                                 _address,
                                                 srv_rtcp));

            auto play_msg = replace_all(PLAY_MSG, "%SESSION", _session);
            send_message(std::move(play_msg), [this](const RtspMessage&) {
              if (on_playing)
                {
                  on_playing();
                }
            });
          });
      }
    else // INTERLEAVED
      {
        const int rtp_channel = 0;
        const int rtcp_channel = 1;

        _rtp_client.InitConnection(
          std::make_shared<RtpConnectionInterleaved>(*this, rtp_channel));
        _rtcp_client.InitConnection(
          std::make_shared<RtpConnectionInterleaved>(*this, rtcp_channel));

        auto msg = fmt2(SETUP_MSG_INTERLEAVED,
                        std::to_string(rtp_channel),
                        std::to_string(rtcp_channel));
        msg = replace_all(std::move(msg), "%TRACK_URI", track_uri);

        send_message(std::move(msg), [this](const RtspMessage& resp) {
          if (auto sit = resp.headers.find("session");
              sit != resp.headers.end())
            {
              _session = sit->second.substr(0, sit->second.find(';'));
            }

          if (resp.status_code == 461)
            {
              LOG_ERROR("RtspSession: server rejected interleaved transport");
              if (on_error)
                {
                  on_error(IFM3D_RTSP_TRANSPORT_UNSUPPORTED);
                }
              close_connection();
              return;
            }

          if (_session.empty())
            {
              LOG_ERROR("RtspSession: no session in SETUP (status {})",
                        resp.status_code);
              if (on_error)
                {
                  on_error(IFM3D_RTSP_REQUEST_FAILED);
                }
              close_connection();
              return;
            }

          auto play_msg = replace_all(PLAY_MSG, "%SESSION", _session);
          send_message(std::move(play_msg), [this](const RtspMessage&) {
            if (on_playing)
              {
                on_playing();
              }
          });
        });
      }
  }

  void
  RtspSession::send_message(std::string msg,
                            std::function<void(const RtspMessage&)> cb)
  {
    msg = replace_all(std::move(msg), "%CSEQ", std::to_string(_cseq++));
    msg = replace_all(std::move(msg), "%STREAM_URI", _rtsp_uri);

    if (msg.rfind("GET_PARAMETER", 0) != 0)
      {
        LOG_DEBUG("RtspSession: sending:\n{}", msg);
      }

    if (cb)
      {
        _pending_callback = std::move(cb);
      }

    auto buf = std::make_shared<std::string>(std::move(msg));
    asio::async_write(_socket,
                      asio::buffer(*buf),
                      [buf](const asio::error_code& ec, std::size_t) {
                        if (ec)
                          {
                            LOG_WARNING("RtspSession: write error: {}",
                                        ec.message());
                          }
                      });
  }

  void
  RtspSession::start_read()
  {
    _socket.async_read_some(
      asio::buffer(_read_buffer),
      [this](const asio::error_code& ec, std::size_t bytes_read) {
        if (ec)
          {
            if (ec != asio::error::operation_aborted)
              {
                LOG_WARNING("RtspSession: read error: {}", ec.message());
              }
            return;
          }

        _unused_data.insert(_unused_data.end(),
                            _read_buffer.begin(),
                            _read_buffer.begin() +
                              static_cast<std::ptrdiff_t>(bytes_read));
        process_buffer();
        start_read();
      });
  }

  void
  RtspSession::process_buffer()
  {
    bool progress = true;
    while (progress)
      {
        progress = false;

        if (_unused_data.empty())
          {
            break;
          }

        if (_unused_data[0] == '$' &&
            _unused_data.size() >
              static_cast<std::size_t>(INTERLEAVED_HEADER_SIZE))
          {
            const std::uint8_t channel = _unused_data[1];
            const std::uint16_t length =
              big_endian::read_u_int16(_unused_data, 2);

            const std::size_t total_len = INTERLEAVED_HEADER_SIZE + length;
            if (_unused_data.size() >= total_len)
              {
                std::vector<std::uint8_t> const payload(
                  _unused_data.begin() + INTERLEAVED_HEADER_SIZE,
                  _unused_data.begin() +
                    static_cast<std::ptrdiff_t>(total_len));

                for (auto& [ch, listener] : _channel_listeners)
                  {
                    listener(channel, payload);
                  }

                _unused_data.erase(_unused_data.begin(),
                                   _unused_data.begin() +
                                     static_cast<std::ptrdiff_t>(total_len));
                progress = true;
              }
          }
        else
          {
            static const std::vector<std::uint8_t> HEADER_END = {'\r',
                                                                 '\n',
                                                                 '\r',
                                                                 '\n'};
            auto it = std::search(_unused_data.begin(),
                                  _unused_data.end(),
                                  HEADER_END.begin(),
                                  HEADER_END.end());

            if (it != _unused_data.end())
              {
                const std::size_t header_end =
                  static_cast<std::size_t>(it - _unused_data.begin());
                const std::size_t header_len = header_end + HEADER_END.size();

                std::string const header_text(
                  _unused_data.begin(),
                  _unused_data.begin() +
                    static_cast<std::ptrdiff_t>(header_len));

                RtspMessage msg = parse_message(header_text);

                int content_length = 0;
                if (auto cit = msg.headers.find("content-length");
                    cit != msg.headers.end())
                  {
                    try
                      {
                        content_length = std::stoi(cit->second);
                      }
                    catch (...)
                      {
                        // IGNORE: malformed Content-Length left as zero
                      }
                  }

                const std::size_t total_msg_len =
                  header_len + static_cast<std::size_t>(content_length);
                if (_unused_data.size() >= total_msg_len)
                  {
                    if (content_length > 0)
                      {
                        msg.content.assign(
                          _unused_data.begin() +
                            static_cast<std::ptrdiff_t>(header_len),
                          _unused_data.begin() +
                            static_cast<std::ptrdiff_t>(total_msg_len));
                      }

                    if (_pending_callback)
                      {
                        auto cb = std::move(_pending_callback);
                        _pending_callback = nullptr;
                        cb(msg);
                      }

                    _unused_data.erase(
                      _unused_data.begin(),
                      _unused_data.begin() +
                        static_cast<std::ptrdiff_t>(total_msg_len));
                    progress = true;
                  }
              }
          }
      }

    if (_unused_data.size() > MAX_UNUSED_DATA)
      {
        LOG_WARNING("RtspSession: receive buffer overflow, clearing");
        if (on_error)
          {
            on_error(IFM3D_RTSP_ERROR);
          }
        _unused_data.clear();
      }
  }

  void
  RtspSession::WriteChannelData(std::uint8_t channel, ByteSpan data)
  {
    const auto len = static_cast<std::uint16_t>(data.Size());

    auto buf = std::make_shared<std::vector<std::uint8_t>>();
    buf->reserve(4 + data.Size());
    buf->push_back('$');
    buf->push_back(channel);
    buf->push_back(static_cast<std::uint8_t>(len >> 8));
    buf->push_back(static_cast<std::uint8_t>(len & 0xFF));
    buf->insert(buf->end(), data.Begin(), data.End());

    asio::async_write(_socket,
                      asio::buffer(*buf),
                      [buf](const asio::error_code& ec, std::size_t) {
                        if (ec)
                          {
                            LOG_WARNING(
                              "RtspSession: writeChannelData error: {}",
                              ec.message());
                          }
                      });
  }

  void
  RtspSession::AddChannelListener(int channel, ChannelListener listener)
  {
    _channel_listeners[channel] = std::move(listener);
  }

  void
  RtspSession::Teardown()
  {
    if (!_session.empty() && _socket.is_open())
      {
        auto msg = replace_all(TEARDOWN_MSG, "%SESSION", _session);
        send_message(std::move(msg));
      }
    close_connection();
  }

  void
  RtspSession::close_connection()
  {
    asio::error_code ec;
    _keep_alive_timer.cancel();
    std::ignore = _socket.close(ec);
  }

  void
  RtspSession::schedule_keep_alive()
  {
    _keep_alive_timer.expires_after(KEEPALIVE_INTERVAL);
    _keep_alive_timer.async_wait([this](const asio::error_code& ec) {
      if (ec)
        {
          return;
        }
      if (!_session.empty())
        {
          auto msg = replace_all(KEEPALIVE_MSG, "%SESSION", _session);
          send_message(std::move(msg));
        }
      schedule_keep_alive();
    });
  }

  bool
  RtspSession::find_consecutive_free_ports(std::uint16_t& p1,
                                           std::uint16_t& p2)
  {
    asio::error_code ec;
    asio::ip::udp::socket probe(_ctx, asio::ip::udp::v4());
    std::ignore =
      probe.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), 0), ec);
    if (ec)
      {
        LOG_ERROR("RtspSession: probe socket bind failed: {}", ec.message());
        return false;
      }
    const std::uint16_t start_port = probe.local_endpoint().port();
    std::ignore = probe.close(ec);

    static constexpr int TRY_COUNT = 100;
    for (int i = 0; i < TRY_COUNT; ++i)
      {
        p1 = static_cast<std::uint16_t>(start_port + (2 * i));
        p2 = static_cast<std::uint16_t>(start_port + (2 * i) + 1);

        asio::ip::udp::socket s1(_ctx, asio::ip::udp::v4());
        asio::ip::udp::socket s2(_ctx, asio::ip::udp::v4());
        std::ignore =
          s1.set_option(asio::socket_base::reuse_address(true), ec);
        std::ignore =
          s2.set_option(asio::socket_base::reuse_address(true), ec);
        std::ignore =
          s1.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), p1), ec);
        if (ec)
          {
            continue;
          }
        std::ignore =
          s2.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), p2), ec);
        if (ec)
          {
            continue;
          }
        return true;
      }
    LOG_ERROR("RtspSession: could not find 2 consecutive free ports");
    return false;
  }

  RtspMessage
  RtspSession::parse_message(const std::string& msg_text) const
  {
    RtspMessage message;

    const std::size_t first_cr = msg_text.find('\r');
    const std::string first_line = msg_text.substr(0, first_cr);

    std::smatch sm;
    if (std::regex_search(first_line, sm, RTSP_RESPONSE_REGEX))
      {
        message.rtsp_version = sm[1].str();
        message.status_code = std::stoi(sm[2].str());
        message.status_message = trim(sm[3].str());

        if (message.status_code != 200)
          {
            LOG_WARNING("RtspSession: non-OK response: {} {}",
                        message.status_code,
                        message.status_message);
          }
      }
    else
      {
        message.rtsp_version = "UNKNOWN";
        message.status_code = -1;
        message.status_message = "INVALID MESSAGE";
        LOG_ERROR("RtspSession: invalid RTSP response: {}", first_line);
        if (on_error)
          {
            on_error(IFM3D_RTSP_ERROR);
          }
      }

    auto begin =
      std::sregex_iterator(msg_text.begin(), msg_text.end(), HEADER_REGEX);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it)
      {
        message.headers[to_lower(trim((*it)[1].str()))] = trim((*it)[2].str());
      }

    return message;
  }

} // namespace ifm3d::rtsp
