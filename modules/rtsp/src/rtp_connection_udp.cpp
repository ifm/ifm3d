/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtp_connection_udp.hpp"

#include <array>
#include <asio/buffer.hpp>
#include <asio/error.hpp>
#include <asio/error_code.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/address.hpp>
#include <asio/ip/address_v4.hpp>
#include <asio/ip/udp.hpp>
#include <asio/socket_base.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ifm3d/common/logging/log.h>

#include "byte_span.hpp"

namespace ifm3d::rtsp
{
  namespace
  {
    constexpr std::chrono::milliseconds UDP_PIERCING_INTERVAL{2000};
    constexpr std::array<std::uint8_t, 1> UDP_PIERCING_PAYLOAD = {0x00};
  } // namespace

  RtpConnectionUdp::RtpConnectionUdp(asio::io_context& ctx,
                                     std::uint16_t local_port,
                                     std::string server_addr,
                                     std::uint16_t server_port)
    : _ctx(ctx),
      _socket(ctx, asio::ip::udp::v4()),
      _piercing_timer(ctx),
      _server_address(std::move(server_addr)),
      _server_port(server_port),
      _local_port(local_port)
  {
    asio::error_code ec;

    std::ignore =
      _socket.set_option(asio::socket_base::reuse_address(true), ec);

    std::ignore =
      _socket.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), local_port),
                   ec);
    if (ec)
      {
        LOG_WARNING("RtpConnectionUdp: bind to port {} failed: {}",
                    local_port,
                    ec.message());
      }
    else
      {
        _local_port = _socket.local_endpoint().port();
      }

    const asio::socket_base::receive_buffer_size recv_buf(42 * 1024 * 1024);
    std::ignore =
      _socket.set_option(recv_buf, ec); // best-effort; OS may cap it

    start_receive();
  }

  RtpConnectionUdp::~RtpConnectionUdp()
  {
    asio::error_code ec;
    _piercing_timer.cancel();
    std::ignore = _socket.close(ec);
  }

  void
  RtpConnectionUdp::Send(ByteSpan data)
  {
    if (_server_address.empty() || _server_port == 0)
      {
        return;
      }

    asio::error_code ec;
    const asio::ip::udp::endpoint target(
      asio::ip::make_address(_server_address, ec),
      _server_port);
    if (ec)
      {
        LOG_WARNING("RtpConnectionUdp: invalid server address {}: {}",
                    _server_address,
                    ec.message());
        return;
      }

    _socket.async_send_to(asio::buffer(data.Data(), data.Size()),
                          target,
                          [](const asio::error_code& send_ec, std::size_t) {
                            if (send_ec)
                              {
                                LOG_WARNING("RtpConnectionUdp: send error: {}",
                                            send_ec.message());
                              }
                          });
  }

  void
  RtpConnectionUdp::EnablePiercingWatchDog()
  {
    _piercing_enabled = true;
    send_piercing_packets();
    schedule_piercing();
  }

  void
  RtpConnectionUdp::start_receive()
  {
    _socket.async_receive_from(
      asio::buffer(_recv_buffer),
      _sender_endpoint,
      [this](const asio::error_code& ec, std::size_t bytes_received) {
        if (ec)
          {
            if (ec != asio::error::operation_aborted)
              {
                LOG_WARNING("RtpConnectionUdp: receive error: {}",
                            ec.message());
              }
            return;
          }

        const std::string sender_addr = _sender_endpoint.address().to_string();
        const std::uint16_t sender_port = _sender_endpoint.port();

        const bool addr_ok = _server_address.empty() ||
                             _server_address == "0.0.0.0" ||
                             _server_address == sender_addr;
        const bool port_ok = _server_port == 0 || sender_port == _server_port;

        if (addr_ok && port_ok)
          {
            _data_received_since_piercing = true;
            if (on_data_received)
              {
                std::vector<std::uint8_t> datagram(
                  _recv_buffer.begin(),
                  _recv_buffer.begin() +
                    static_cast<std::ptrdiff_t>(bytes_received));
                on_data_received(std::move(datagram));
              }
          }

        start_receive();
      });
  }

  void
  RtpConnectionUdp::schedule_piercing()
  {
    _piercing_timer.expires_after(UDP_PIERCING_INTERVAL);
    _piercing_timer.async_wait([this](const asio::error_code& ec) {
      if (ec)
        {
          return;
        }
      if (!_data_received_since_piercing)
        {
          send_piercing_packets();
        }
      _data_received_since_piercing = false;
      schedule_piercing();
    });
  }

  void
  RtpConnectionUdp::send_piercing_packets()
  {
    const std::uint16_t target_port =
      (_server_port != 0) ? _server_port : _local_port;

    const asio::ip::udp::endpoint bcast(asio::ip::address_v4::broadcast(),
                                        target_port);

    asio::error_code ec;
    std::ignore = _socket.set_option(asio::socket_base::broadcast(true), ec);

    _socket.async_send_to(
      asio::buffer(UDP_PIERCING_PAYLOAD, sizeof(UDP_PIERCING_PAYLOAD)),
      bcast,
      [](const asio::error_code&, std::size_t) {});
  }

} // namespace ifm3d::rtsp
