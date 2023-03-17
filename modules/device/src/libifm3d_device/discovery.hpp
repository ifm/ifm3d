// -*- c++ -*-
/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_CAMERA_DISCOVERY_HPP
#define IFM3D_CAMERA_DISCOVERY_HPP

#include <asio.hpp>
#include <vector>
#include <string>
#include <tuple>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <ifm3d/device/ifm_network_device.h>
#ifdef __unix__
#  include <arpa/inet.h>
#  include <sys/socket.h>
#  include <ifaddrs.h>
#endif

namespace ifm3d
{
  /** Broadcast default port. */
  constexpr uint16_t BCAST_DEFAULT_PORT = 3321;

  /** Broadcast reply magic number. */
  constexpr uint32_t BCAST_MAGIC_REPLY = 0x19111981;

  /** Broadcast request magic number. */
  constexpr uint32_t BCAST_MAGIC_REQUEST = 0x1020efcf;

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
    const uint32_t magic;
    /** Port to send reply to. */
    const uint16_t reply_port;

    /** Unused. */
    const uint16_t reserved;

    BcastRequest()
      : magic(htonl(BCAST_MAGIC_REQUEST)),
        reply_port(htons(BCAST_DEFAULT_PORT)),
        reserved(0)
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
    uint8_t mac[6];

    /** Network device flags. */
    uint16_t flags;

    /** Network device hostname */
    char name[64];

    /** @device name defined by the user */
    char devicename[256];
  };

  namespace
  {
    inline const std::string
    address2Str(const uint32_t addr)
    {
      std::stringstream ss;
      ss << ((addr >> 24) & 0xff) << "." << ((addr >> 16) & 0xff) << "."
         << ((addr >> 8) & 0xff) << "." << ((addr >> 0) & 0xff);
      return ss.str();
    }

    inline const std::string
    uint8Mac2macStr(uint8_t* mac, size_t size = 6)
    {
      std::stringstream ss;
      auto converttoHexandAppend = [&](const uint8_t value) {
        ss << std::hex << std::setfill('0') << std::setw(2) << int(value);
      };

      for (int i = 0; i < size - 1; i++)
        {
          converttoHexandAppend(mac[i]);
          ss << "::";
        }
      converttoHexandAppend(mac[size - 1]);
      return ss.str();
    }
  }

  IFMNetworkDevice::IFMNetworkDevice(
    Data& data,
    const std::string& ip_address_via_interface)
  {
    BcastReply reply;
    std::memcpy(&reply, data.data(), sizeof(BcastReply));

    ip_address_ = address2Str(ntohl(reply.ip_address));
    gateway_ = address2Str(ntohl(reply.ip_gateway));
    subnet_ = address2Str(ntohl(reply.ip_netmask));
    mac_ = uint8Mac2macStr((uint8_t*)reply.mac);
    port_ = ntohs(reply.port);
    flags_ = ntohs(reply.flags);
    device_id_ = ntohs(reply.device_id);
    vendor_id_ = ntohs(reply.vendor_id);
    hostname_ = std::string(reply.name);
    device_name_ = std::string(reply.devicename);
    found_via_ = ip_address_via_interface;
  }

  std::string
  IFMNetworkDevice::GetIPAddress() const
  {
    return ip_address_;
  }

  std::string
  IFMNetworkDevice::GetMACAddress() const
  {
    return mac_;
  }

  std::string
  IFMNetworkDevice::GetNetmask() const
  {
    return subnet_;
  }

  std::string
  IFMNetworkDevice::GetGateway() const
  {
    return gateway_;
  }

  uint16_t
  IFMNetworkDevice::GetPort() const
  {
    return port_;
  }

  uint16_t
  IFMNetworkDevice::GetFlag() const
  {
    return flags_;
  }

  std::string
  IFMNetworkDevice::GetHostName() const
  {
    return hostname_;
  }

  std::string
  IFMNetworkDevice::GetDeviceName() const
  {
    return device_name_;
  }

  uint16_t
  IFMNetworkDevice::GetVendorId() const
  {
    return vendor_id_;
  }

  uint16_t
  IFMNetworkDevice::GetDeviceId() const
  {
    return device_id_;
  }

  std::string
  IFMNetworkDevice::GetFoundVia() const
  {
    return found_via_;
  }

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
                  asio::ip::udp::endpoint& local_endpoint)
      : socket_(context),
        timer_(context)
    {
      socket_.open(asio::ip::udp::v4());
      data_.resize(MAX_UDP_PACKET_SIZE);
      socket_.set_option(asio::socket_base::broadcast(true));
      socket_.set_option(asio::ip::udp::socket::reuse_address(true));
      socket_.bind(local_endpoint);
    }

    /*Sends the data on the endpoint
     *@param data Data to be send on endpoint
     *@param Endpoint to send the data.
     **/
    void
    Send(const Data& data, asio::ip::udp::endpoint& remote_endpoint)
    {
      socket_.async_send_to(asio::buffer((char*)data.data(), data.size()),
                            remote_endpoint,
                            std::bind(&UDPConnection::_HandleSend,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
    }

    /*Grab data on the local endpoint till the timeout occur*/
    void
    GrabData()
    {
      _Receive();
      _CheckTimeout();
    }

    void
    Close()
    {
      socket_.close();
    }

    /*Provides interface to register callback on data received */
    void
    RegisterOnReceive(
      std::function<void(asio::ip::udp::endpoint&, Data, size_t)> on_recvieve)
    {
      this->on_receive_ = on_recvieve;
    }

    /*Provides interface to register callback on timeout or for any
     * error*/
    void
    RegisterOnClose(
      std::function<void(std::shared_ptr<UDPConnection>)> on_close)
    {
      this->on_close_ = on_close;
    }

  private:
    void
    _HandleSend(const asio::error_code& error,
                std::size_t /*bytes_transferred*/)
    {
      /* udp we send and forget so nothing to do in this callback*/
    }

    void
    _Receive()
    {
      /* start the timer for timeout of 3 secs, if data do not come
         within timeout then connection will be closed and notified to parent
       */
      timer_.expires_from_now(
        std::chrono::milliseconds(TIMEOUT_FOR_EACH_SEARCH_IN_MS));
      data_.clear();
      data_.resize(MAX_UDP_PACKET_SIZE);

      try
        {
          asio::ip::udp::endpoint sender_endpoint;
          socket_.async_receive_from(asio::buffer(data_.data(), data_.size()),
                                     sender_endpoint,
                                     std::bind(&UDPConnection::_HandleReceive,
                                               this,
                                               socket_.local_endpoint(),
                                               std::placeholders::_1,
                                               std::placeholders::_2));
        }
      catch (std::exception& e)
        {
          std::cerr << e.what() << std::endl;
        }
    }

    void
    _HandleReceive(asio::ip::udp::endpoint& sender,
                   const asio::error_code& error,
                   size_t bytes_transferred)
    {
      if (error)
        {
          return;
        }
      on_receive_(sender, data_, bytes_transferred);
      _Receive();
    }

    void
    close()
    {
      socket_.close();
      // notify the caller through registered callback
      on_close_(shared_from_this());
    }

    void
    _CheckTimeout()
    {
      if (timer_.expires_at() <= std::chrono::system_clock::now())
        {
          close();
        }
      else
        {
          auto shared_this = this->shared_from_this();
          timer_.async_wait([shared_this](const asio::error_code&) {
            shared_this->_CheckTimeout();
          });
        }
    }

  private:
    asio::ip::udp::socket socket_;
    asio::system_timer timer_;
    Data data_;
    std::function<void(asio::ip::udp::endpoint&, Data, size_t)> on_receive_;
    std::function<void(std::shared_ptr<UDPConnection>)> on_close_;
  };

  const std::map<size_t, std::function<size_t(size_t)>> package_validation{
    {BCAST_MAGIC_REQUEST,
     [](size_t size) -> size_t { return sizeof(BcastRequest) - size; }},
    {BCAST_MAGIC_REPLY,
     [](size_t size) -> size_t { return sizeof(BcastReply) - size; }}};

  const unsigned int THREADS_FOR_IO_OPERATIONS = 3;

  class IFMDeviceDiscovery
  {
  public:
    IFMDeviceDiscovery() : work_guard_(asio::make_work_guard(io_context_))
    {
      for (auto i = 0; i < THREADS_FOR_IO_OPERATIONS; i++)
        {
          thread_pool_.push_back(
            std::thread(std::bind([&] { io_context_.run(); })));
        }
    }
    IFMDeviceDiscovery(IFMDeviceDiscovery&&) = delete;
    IFMDeviceDiscovery& operator=(IFMDeviceDiscovery&&) = delete;
    IFMDeviceDiscovery(IFMDeviceDiscovery&) = delete;
    IFMDeviceDiscovery& operator=(IFMDeviceDiscovery&) = delete;

    std::vector<IFMNetworkDevice>
    NetworkSearch()
    {
      device_list_.clear();

      try
        {
          auto addresses = _GetAllInterfaceAddress();

          /* broadcast ifm discovery request on all interfaces */
          for (const auto& address : addresses)
            {
              _BroadcastOnInterface(address);
            }
          /* get the device list  this will block the thread
             till all NIC are scanned for devices */
          auto devices = _GetDeviceList();

          /* stop the io_context after getting list */
          io_context_.stop();

          // wait for all threads to complete
          for (std::thread& thread : thread_pool_)
            {
              if (thread.joinable())
                {
                  thread.join();
                }
            }
        }
      catch (std::exception& e)
        {
          std::cerr << e.what() << std::endl;
        }
      return device_list_;
    }

  private:
    std::vector<std::string>
    _GetAllInterfaceAddress()
    {
      std::vector<std::string> addresses;
#ifdef __unix__
      struct ifaddrs *ifap, *ifa;
      struct sockaddr_in* sa;
      char* addr;

      getifaddrs(&ifap);
      for (ifa = ifap; ifa; ifa = ifa->ifa_next)
        {
          if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET)
            {
              sa = (struct sockaddr_in*)ifa->ifa_addr;
              addr = inet_ntoa(sa->sin_addr);
              std::string ip_address = std::string(addr);
              addresses.push_back(ip_address);
            }
        }
      freeifaddrs(ifap);
#else

      asio::ip::udp::resolver resolver(io_context_);
      std::string h = asio::ip::host_name();
      auto re_list = resolver.resolve({h, ""});

      for (const auto& re : re_list)
        {
          if (re.endpoint().address().is_v4())
            {
              addresses.push_back(re.endpoint().address().to_string());
            }
        }
#endif
      return addresses;
    }
    /* broadcast the Bcast request on interface*/
    void
    _BroadcastOnInterface(std::string interface_ip)
    {
      const BcastRequest request;
      Data data;
      data.resize(sizeof(BcastRequest));
      std::memcpy(data.data(), &request, sizeof(BcastRequest));

      /* create a local endpoints */
      asio::ip::udp::endpoint local_endpoint =
        asio::ip::udp::endpoint(asio::ip::address::from_string(interface_ip),
                                BCAST_DEFAULT_PORT);

      /** getting the netmask of the interface*/
      auto netmask = asio::ip::address_v4::netmask(
        asio::ip::address_v4::from_string(interface_ip));

      /**broadcast of the interface*/
      auto broadcast_address_of_interface =
        asio::ip::address_v4::broadcast(
          asio::ip::address_v4::from_string(interface_ip),
          netmask)
          .to_string();

      /* create a local broadcast endpoints */
      asio::ip::udp::endpoint broadcast_endpoint = asio::ip::udp::endpoint(
        asio::ip::address::from_string(broadcast_address_of_interface),
        BCAST_DEFAULT_PORT);

      auto con = std::make_shared<UDPConnection>(io_context_, local_endpoint);
      con->RegisterOnReceive(std::bind(&IFMDeviceDiscovery::_OnReceive,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3));
      con->RegisterOnClose(std::bind(&IFMDeviceDiscovery::_RemoveConnection,
                                     this,
                                     std::placeholders::_1));

      /* send BCAST_REQUEST on all interfaces */
      con->Send(data, broadcast_endpoint);

      connection_list_.push_back(con);
    }

    /*Checks the data for corresponding request on magic entry. */
    std::tuple<bool, uint32_t, size_t>
    _IsResponseComplete(const Data& data, const size_t byte_recv)
    {
      uint32_t magic_value = 0;
      std::memcpy(&magic_value, data.data(), sizeof(uint32_t));

      magic_value = ntohl(magic_value);

      if (package_validation.find(magic_value) != package_validation.end())
        {
          auto required_length = package_validation.at(magic_value)(byte_recv);
          return std::make_tuple(required_length == 0,
                                 magic_value,
                                 required_length);
        }
      else
        {
          std::cerr << "unknown response for discovery" << std::endl;
          return {};
        }
    }

    /* Handles the data packet, parse the reply, creates the
     * IFMNetworkDevice */
    void
    _OnReceive(asio::ip::udp::endpoint& sender,
               Data data,
               size_t bytes_transferred)
    {
      bool iscomplete;
      size_t required;
      uint32_t magic_value;
      std::tie(iscomplete, magic_value, required) =
        _IsResponseComplete(data, bytes_transferred);

      if (iscomplete)
        {
          if (magic_value == BCAST_MAGIC_REPLY)
            {
              IFMNetworkDevice ifm_device(data, sender.address().to_string());
              std::lock_guard<std::mutex> lock(device_list_lock_);
              device_list_.push_back(ifm_device);
            }
        }
    }
    /* return the discovered device list*/
    std::vector<IFMNetworkDevice>
    _GetDeviceList()
    {
      auto connection_list = connection_list_;
      for (auto& con : connection_list)
        {
          con->GrabData();
        }
      /*block the main thread till the search is complete
      which is the condition when all the there is no respone on any interface
      for more than  TIMEOUT_FOR_EACH_SEARCH_IN_MS*/
      std::unique_lock<std::mutex> lock_to_complete(con_mutex_);
      cv_.wait(lock_to_complete, [&] { return connection_list_.size() == 0; });
      return device_list_;
    }

    /* remove the connection from the connection list */
    void
    _RemoveConnection(std::shared_ptr<UDPConnection> con)
    {
      std::lock_guard<std::mutex> lock(con_mutex_);
      connection_list_.erase(std::remove(std::begin(connection_list_),
                                         std::end(connection_list_),
                                         con),
                             std::end(connection_list_));
      for (const auto& con : connection_list_)
        {
          con->Close();
        }
      connection_list_.resize(0);
      cv_.notify_one();
    }

    asio::io_context io_context_;
    asio::executor_work_guard<asio::io_service::executor_type> work_guard_;
    std::mutex con_mutex_;
    std::condition_variable cv_;
    std::vector<std::shared_ptr<UDPConnection>> connection_list_;
    std::vector<ifm3d::IFMNetworkDevice> device_list_;
    std::vector<std::thread> thread_pool_;
    std::mutex device_list_lock_;
  };
}
#endif // IFM3D_CAMERA_DISCOVERY_HPP