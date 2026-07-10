/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTP_CONNECTION_UDP_HPP
#define IFM3D_RTSP_RTP_CONNECTION_UDP_HPP

/** @file
 * @brief RTP/RTCP transport over UDP using standalone asio.
 */

#include <array>
#include <cstdint>
#include <string>

#include <asio.hpp>

#include "rtp_connection.hpp"

namespace ifm3d::rtsp
{
  /**
   * @brief RTP/RTCP transport over a bound UDP socket.
   */
  class RtpConnectionUdp : public RtpConnection
  {
  public:
    /**
     * @param ctx         asio io_context for all async operations.
     * @param local_port  UDP port to bind locally (0 = let the OS choose).
     * @param server_addr Remote server address ("" / "0.0.0.0" = any).
     * @param server_port Remote server port (0 = accept from any port).
     */
    RtpConnectionUdp(asio::io_context& ctx,
                     std::uint16_t local_port,
                     std::string server_addr = "",
                     std::uint16_t server_port = 0);

    ~RtpConnectionUdp() override;

    RtpConnectionUdp(const RtpConnectionUdp&) = delete;
    RtpConnectionUdp& operator=(const RtpConnectionUdp&) = delete;
    RtpConnectionUdp(RtpConnectionUdp&&) = delete;
    RtpConnectionUdp& operator=(RtpConnectionUdp&&) = delete;

    void Send(ByteSpan data) override;

    /**
     * Enable a NAT-piercing watchdog that periodically broadcasts a tiny
     * packet when no data has been received, to keep the firewall port open.
     */
    void EnablePiercingWatchDog();

  private:
    void start_receive();
    void send_piercing_packets();
    void schedule_piercing();

    asio::io_context& _ctx;
    asio::ip::udp::socket _socket;
    asio::steady_timer _piercing_timer;

    std::string _server_address;
    std::uint16_t _server_port;
    std::uint16_t _local_port;
    bool _piercing_enabled{false};
    bool _data_received_since_piercing{false};

    static constexpr int RECV_BUFFER_SIZE = 65536;
    std::array<std::uint8_t, RECV_BUFFER_SIZE> _recv_buffer{};
    asio::ip::udp::endpoint _sender_endpoint;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_RTP_CONNECTION_UDP_HPP
