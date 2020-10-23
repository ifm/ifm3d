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
#include <ifm3d/camera/ifm_network_device.h>
#ifdef __unix__
#  include <arpa/inet.h>
#  include <sys/socket.h>
#  include <ifaddrs.h>
#endif

namespace ifm3d
{
  /** @brief Broadcast default port. */
  constexpr uint16_t BCAST_DEFAULT_PORT = 3321;

  /** @brief Broadcast reply magic number. */
  constexpr uint32_t BCAST_MAGIC_REPLY = 0x19111981;

  /** @brief Broadcast request magic number. */
  constexpr uint32_t BCAST_MAGIC_REQUEST = 0x1020efcf;

  /** @brief It signalize that network device is not in same subnet. */
  constexpr uint32_t BCAST_FLAG_WRONGSUBNET = 0x0001;

  /** @brief It signalize that network device interface ip address is set via
   * dhcp. */
  constexpr uint32_t BCAST_FLAG_BYDHCP = 0x0002;

  /** @brief It signalize that network device interface ip address is
   * temporary. */
  constexpr uint32_t BCAST_FLAG_ISTEMPORARY = 0x0004;

  /** @brief Reserved. */
  constexpr uint32_t BCAST_FLAG_WITHPRGTYPE = 0x8000;

  /** @brief Broadcast request packet structure. */
  struct BcastRequest
  {
    /** @brief Default magic number 0x1020EFCF. */
    const uint32_t magic;
    /** @brief Port to send reply to. */
    const uint16_t reply_port;

    /** @brief Unused. */
    const uint16_t reserved;

    BcastRequest()
      : magic(htonl(BCAST_MAGIC_REQUEST)),
        reply_port(htons(BCAST_DEFAULT_PORT)),
        reserved(0)
    { }
  };

  /** @brief Broadcast reply packet structure. */
  struct BcastReply
  {
    /** @brief Default magic number 0x19111981. */
    uint32_t magic;

    /** @brief Network device ip address. */
    uint32_t ip_address;

    /** @brief Network device gateway address. */
    uint32_t ip_gateway;

    /** @brief Network device netmask. */
    uint32_t ip_netmask;

    /** @brief Network device port this is send from. */
    uint16_t port;

    /** @brief Vendor ID */
    uint16_t vendor_id;

    /** @brief Unused. */
    uint16_t device_id;

    /** @brief Unused. */
    uint16_t reserved1;

    /** @brief Unused. */
    uint32_t reserved2;

    /** @brief Unused. */
    uint32_t reserved3;

    /** @brief Network device interface mac address. */
    uint8_t mac[6];

    /** @brief Network device flags. */
    uint16_t flags;

    /** @brief Network device hostname */
    char name[64];

    /** @brief device name defined by the user */
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

  uint16_t IFMNetworkDevice::GetVendorId() const
  {
    return vendor_id_;
  }

  uint16_t IFMNetworkDevice::GetDeviceId() const
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
  /*@brief This class provides an interface  for send and receive
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
      asio::socket_base::broadcast option(true);
      socket_.set_option(option);
      socket_.set_option(asio::ip::udp::socket::reuse_address(true));
      socket_.bind(local_endpoint);
    }

    /*@brief sends the data on the endpoint
     *@param data Data to be send on endpoint
     *@param Endpoint to send the data.
     **/
    void
    Send(const Data& data, asio::ip::udp::endpoint& remote_endpoint)
    {
      socket_.async_send_to(asio::buffer((char*)data.data(), data.size()),
                            remote_endpoint,
                            std::bind(&UDPConnection::handle_send,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
    }
    /*@brief grab data on the local endpoint till the timeout occur*/
    void
    GrabData()
    {
      recieve();
      check_timeout();
    }

    void
    Close()
    {
      socket_.close();
    }

    /*@brief provides interface to register callback on data received */
    void
    RegisterOnRecieve(
      std::function<void(asio::ip::udp::endpoint&, Data, size_t)> on_recvieve)
    {
      this->on_recieve_ = on_recvieve;
    }

    /*@brief provides interface to register callback on timeout or for any
     * error*/
    void
    RegisterOnClose(
      std::function<void(std::shared_ptr<UDPConnection>)> on_close)
    {
      this->on_close_ = on_close;
    }

  protected:
    void
    handle_send(const asio::error_code& error,
                std::size_t /*bytes_transferred*/)
    {
      /* udp we send and forget so nothing to do in this callback*/
    }

    void
    recieve()
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
                                     std::bind(&UDPConnection::handle_receive,
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
    handle_receive(asio::ip::udp::endpoint& sender,
                   const asio::error_code& error,
                   size_t bytes_transferred)
    {
      if (error)
        {
          return;
        }
      on_recieve_(sender, data_, bytes_transferred);
      recieve();
    }

    void
    close()
    {
      socket_.close();
      // notify the caller through registered callback
      on_close_(shared_from_this());
    }

    void
    check_timeout()
    {
      if (timer_.expires_at() <= std::chrono::system_clock::now())
        {
          close();
        }
      else
        {
          timer_.async_wait(std::bind(&UDPConnection::check_timeout, this));
        }
    }

  private:
    asio::ip::udp::socket socket_;
    asio::system_timer timer_;
    Data data_;
    std::function<void(asio::ip::udp::endpoint&, Data, size_t)> on_recieve_;
    std::function<void(std::shared_ptr<UDPConnection>)> on_close_;
  };

  const std::map<uint32_t, std::function<size_t(size_t)>> package_validation{
    {BCAST_MAGIC_REQUEST,
     [](size_t size) -> uint32_t { return sizeof(BcastRequest) - size; }},
    {BCAST_MAGIC_REPLY,
     [](size_t size) -> uint32_t { return sizeof(BcastReply) - size; }}};

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

    std::vector<std::string>
    getAllInterface_address()
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
          addresses.push_back(re.endpoint().address().to_string());
        }
#endif
      return addresses;
    }

    std::vector<IFMNetworkDevice>
    NetworkSearch()
    {
      device_list_.clear();

      /* Creating universal listener udp connection */
      asio::ip::udp::endpoint endpoint(asio::ip::address_v4::any(),
                                       BCAST_DEFAULT_PORT);
      auto con = std::make_shared<UDPConnection>(io_context_, endpoint);
      con->RegisterOnRecieve(std::bind(&IFMDeviceDiscovery::on_receive,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3));
      con->RegisterOnClose(std::bind(&IFMDeviceDiscovery::remove_connection,
                                     this,
                                     std::placeholders::_1));

      connection_list_.push_back(con);

      try
        {
          auto addresses = getAllInterface_address();

          /* broadcast ifm discovery request on all interfaces */
          for (const auto& re : addresses)
            {
              broadcast_on_interface(re);
            }
          /* get the device list  this will block the thread
             till all NIC are scanned for devices */
          auto devices = getDeviceList();

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

          return device_list_;
        }
      catch (std::exception& e)
        {
          std::cerr << e.what() << std::endl;
        }
    }

  private:
    /* @brief broadcast the Bcast request on all interfaces*/
    void
    broadcast_on_interface(std::string interface_ip)
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
      con->RegisterOnRecieve(std::bind(&IFMDeviceDiscovery::on_receive,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3));
      con->RegisterOnClose(std::bind(&IFMDeviceDiscovery::remove_connection,
                                     this,
                                     std::placeholders::_1));

      /* send BCAST_REQUEST on all interfaces */
      con->Send(data, broadcast_endpoint);

      connection_list_.push_back(con);
    }

    /*@brief checks the data for corresponding request on magic entry. */
    std::tuple<bool, uint32_t, size_t>
    is_response_complete(const Data& data, const size_t byte_recv)
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
          std::cerr << "invalid response for discovery" << std::endl;
        }
    }
    /*@brief Handles the data packet, parse the reply, creates the
     * IFMNetworkDevice */
    void
    on_receive(asio::ip::udp::endpoint& sender,
               Data data,
               size_t bytes_transferred)
    {
      bool iscomplete;
      size_t required;
      uint32_t magic_value;
      std::tie(iscomplete, magic_value, required) =
        is_response_complete(data, bytes_transferred);

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
    getDeviceList()
    {
      /** start grab on universal brodcast socket*/
      connection_list_[0]->GrabData();
      /*block the main thread till the search is complete
      which is the condition when all the connection on
      interface is removed from connection pool */
      std::unique_lock<std::mutex> lock_to_complete(con_mutex_);
      cv_.wait(lock_to_complete, [&] { return connection_list_.size() == 0; });
      return device_list_;
    }

    /*@brief remove the connection from the connection list */
    void
    remove_connection(std::shared_ptr<UDPConnection> con)
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

/* Sample usage program */
#if 0

int main()
{
  ifm3d::IFMDeviceDiscovery ifm_discovery;
  auto devices = ifm_discovery.NetworkSearch();

  for (auto & device : devices)
    {
      device.Display();
    }
}

#endif
