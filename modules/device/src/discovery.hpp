// -*- c++ -*-
/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_CAMERA_DISCOVERY_HPP
#define IFM3D_CAMERA_DISCOVERY_HPP

#include <asio.hpp>
#include <cstdint>
#include <ifm3d/device/ifm_network_device.h>
#include <map>
#include <vector>

namespace ifm3d
{
  /** Broadcast default port. */
  constexpr uint16_t BCAST_DEFAULT_PORT = 3321;

  /** Broadcast reply magic number. */
  constexpr uint32_t BCAST_MAGIC_REPLY = 0x19111981;

  /** Broadcast request magic number. */
  constexpr uint32_t BCAST_MAGIC_REQUEST = 0x1020efcf;

  /** Broadcast ipchange magic number. */
  constexpr uint32_t BCAST_MAGIC_IPCHANGE = 0x1020efce;

  /** It signalize that network device is not in same subnet. */
  constexpr uint32_t BCAST_FLAG_WRONGSUBNET = 0x0001;

  /** It signalize that network device interface ip address is set via
   * dhcp. */
  constexpr uint32_t BCAST_FLAG_BYDHCP = 0x0002;

  /** It signalize that network device interface ip address is
   * temporary. */
  constexpr uint32_t BCAST_FLAG_ISTEMPORARY = 0x0004;

  /** Reserved. */
  constexpr uint32_t BCAST_FLAG_WITHPRGTYPE = 0x8000;

  /** Broadcast request packet structure. */
  struct BcastRequest
  {
    /** Default magic number 0x1020EFCF. */
    uint32_t magic;
    /** Port to send reply to. */
    uint16_t reply_port;

    /** Unused. */
    uint16_t reserved{};

    BcastRequest()
      : magic(htonl(BCAST_MAGIC_REQUEST)),
        reply_port(htons(BCAST_DEFAULT_PORT))
    {}
  };

  /** Broadcast reply packet structure. */
  struct BcastReply
  {
    /** Default magic number 0x19111981. */
    uint32_t magic;

    /** Network device ip address. */
    uint32_t ip_address;

    /** Network device gateway address. */
    uint32_t ip_gateway;

    /** Network device netmask. */
    uint32_t ip_netmask;

    /** Network device port this is send from. */
    uint16_t port;

    /** Vendor ID */
    uint16_t vendor_id;

    /** Unused. */
    uint16_t device_id;

    /** Unused. */
    uint16_t reserved1;

    /** Unused. */
    uint32_t reserved2;

    /** Unused. */
    uint32_t reserved3;

    /** Network device interface mac address. */
    std::array<uint8_t, 6> mac;

    /** Network device flags. */
    uint16_t flags;

    /** Network device hostname */
    std::array<char, 64> name;

    /** @device name defined by the user */
    std::array<char, 256> devicename;
  };

  /** Broadcast ipchange packet structure. */
  struct BcastIPChange
  {
    /** Default magic number 0x1020efce. */
    uint32_t magic;

    /** Port to send reply to. */
    uint16_t reply_port;

    /** Unused. */
    uint16_t reserved1;

    /** Temporary ip address that should be set. */
    uint32_t temp_ip;

    /** Unused. */
    uint16_t reserved2;

    /** Mac address of interface to change. */
    std::array<uint8_t, 6> mac;
  };

  constexpr size_t MAX_UDP_PACKET_SIZE = 65535;
  using Data = std::vector<unsigned char>;
  constexpr size_t TIMEOUT_FOR_EACH_SEARCH_IN_MS = 3000;

  /* This class provides an interface  for send and receive
   * over UDP socket.
   **/
  class UDPConnection : public std::enable_shared_from_this<UDPConnection>
  {
  public:
    UDPConnection(asio::io_context& context,
                  asio::ip::udp::endpoint& local_endpoint);

    /*Sends the data on the endpoint
     *@param data Data to be send on endpoint
     *@param Endpoint to send the data.
     **/
    void Send(const Data& data, asio::ip::udp::endpoint& remote_endpoint);

    /*Sends the data on the endpoint
     *@param data Data to be send on endpoint
     *@param Endpoint to send the data.
     **/
    void SendIPChangeCall(Data& data,
                          asio::ip::udp::endpoint& remote_endpoint);

    /*Grab data on the local endpoint till the timeout occur*/
    void GrabData();

    void Close();

    /*Provides interface to register callback on data received */
    void RegisterOnReceive(
      std::function<void(asio::ip::udp::endpoint const&, Data, size_t)>
        on_recvieve);

    /*Provides interface to register callback on timeout or for any
     * error*/
    void RegisterOnClose(
      std::function<void(std::shared_ptr<UDPConnection>)> on_close);

  private:
    void handle_send(const asio::error_code& error,
                     std::size_t /*bytes_transferred*/);

    void receive();
    void handle_receive(asio::ip::udp::endpoint const& sender,
                        const asio::error_code& error,
                        size_t bytes_transferred);

    void close();

    void check_timeout();

  private:
    asio::ip::udp::socket _socket;
    asio::system_timer _timer;
    Data _data;
    std::function<void(asio::ip::udp::endpoint const&, Data, size_t)>
      _on_receive;
    std::function<void(std::shared_ptr<UDPConnection>)> _on_close;
  };

  const std::map<size_t, std::function<size_t(size_t)>> PACKAGE_VALIDATION{
    {BCAST_MAGIC_REQUEST,
     [](size_t size) -> size_t { return sizeof(BcastRequest) - size; }},
    {BCAST_MAGIC_REPLY,
     [](size_t size) -> size_t { return sizeof(BcastReply) - size; }},
    {BCAST_MAGIC_IPCHANGE,
     [](size_t size) -> size_t { return sizeof(BcastIPChange) - size; }}};

  const unsigned int THREADS_FOR_IO_OPERATIONS = 3;

  class IFMDeviceDiscovery
  {
  public:
    IFMDeviceDiscovery();

    ~IFMDeviceDiscovery();

    IFMDeviceDiscovery(IFMDeviceDiscovery&&) = delete;
    IFMDeviceDiscovery& operator=(IFMDeviceDiscovery&&) = delete;
    IFMDeviceDiscovery(IFMDeviceDiscovery&) = delete;
    IFMDeviceDiscovery& operator=(IFMDeviceDiscovery&) = delete;

    std::vector<IFMNetworkDevice> NetworkSearch();

    void SetTemporaryIP(const std::string& mac_address,
                        const std::string& temp_ip);

  private:
    std::vector<std::string> get_all_interface_address();
    /* broadcast the Bcast request on interface*/
    void broadcast_on_interface(const std::string& interface_ip);

    /*Checks the data for corresponding request on magic entry. */
    std::tuple<bool, uint32_t, size_t> is_response_complete(const Data& data,
                                                            size_t byte_recv);

    /* Handles the data packet, parse the reply, creates the
     * IFMNetworkDevice */
    void on_receive(asio::ip::udp::endpoint const& sender,
                    Data data,
                    size_t bytes_transferred);
    /* return the discovered device list*/
    std::vector<IFMNetworkDevice> get_device_list();

    /* remove the connection from the connection list */
    void remove_connection(std::shared_ptr<UDPConnection> con);

    /* Broadcast the IP change request */
    void send_ip_change_broadcast(const std::string& mac_address,
                                  const std::string& temp_ip,
                                  const std::string& interface_ip);

    asio::io_context _io_context;
    asio::executor_work_guard<asio::io_service::executor_type> _work_guard;
    std::mutex _con_mutex;
    std::condition_variable _cv;
    std::vector<std::shared_ptr<UDPConnection>> _connection_list;
    std::vector<ifm3d::IFMNetworkDevice> _device_list;
    std::vector<std::thread> _thread_pool;
    std::mutex _device_list_lock;
  };

} // namespace ifm3d

#endif // IFM3D_CAMERA_DISCOVERY_HPP