#include "discovery.hpp"

#include <algorithm>
#include <array>
#include <asio/buffer.hpp>
#include <asio/error.hpp>
#include <asio/error_code.hpp>
#include <asio/executor_work_guard.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/address_v4.hpp>
#include <asio/ip/udp.hpp>
#include <asio/socket_base.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <ifm3d/device/ifm_network_device.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>
#ifdef __unix__
#  include <arpa/inet.h>
#  include <ifaddrs.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#endif

namespace
{
  inline std::string
  address_to_str(const uint32_t addr)
  {
    std::stringstream ss;
    ss << ((addr >> 24) & 0xff) << "." << ((addr >> 16) & 0xff) << "."
       << ((addr >> 8) & 0xff) << "." << ((addr >> 0) & 0xff);
    return ss.str();
  }

  inline uint32_t
  str_to_address(const std::string& addr)
  {
    uint32_t str_addr = 0;
    std::stringstream ss(addr);
    std::string octet;
    int shift = 24;

    while (std::getline(ss, octet, '.'))
      {
        const uint32_t num = std::stoi(octet);
        str_addr |= (num << shift);
        shift -= 8;
      }

    return str_addr;
  }

  inline std::string
  mac_uint8_to_str(uint8_t* mac, size_t size = 6)
  {
    std::stringstream ss;
    auto convert_to_hex_and_append = [&](const uint8_t value) {
      ss << std::hex << std::setfill('0') << std::setw(2) << int(value);
    };

    for (size_t i = 0; i + 1 < size; i++)
      {
        convert_to_hex_and_append(mac[i]);
        ss << ":";
      }
    convert_to_hex_and_append(mac[size - 1]);
    return ss.str();
  }

  inline std::array<uint8_t, 6>
  mac_str_to_uint8(const std::string& mac_str)
  {
    std::array<uint8_t, 6> mac{};
    std::stringstream ss(mac_str);
    std::string byte_str;
    int i = 0;

    while (std::getline(ss, byte_str, ':') && i < 6)
      {
        mac[i] = static_cast<uint8_t>(std::stoul(byte_str, nullptr, 16));
        ++i;
      }
    return mac;
  }
}

ifm3d::IFMNetworkDevice::IFMNetworkDevice(Data& data,
                                          std::string ip_address_via_interface)
  : _found_via(std::move(ip_address_via_interface))
{
  BcastReply reply{};
  std::memcpy(&reply, data.data(), sizeof(BcastReply));

  _ip_address = address_to_str(ntohl(reply.ip_address));
  _gateway = address_to_str(ntohl(reply.ip_gateway));
  _subnet = address_to_str(ntohl(reply.ip_netmask));
  _mac = mac_uint8_to_str((uint8_t*)reply.mac.data());
  _port = ntohs(reply.port);
  _flags = ntohs(reply.flags);
  _device_id = ntohs(reply.device_id);
  _vendor_id = ntohs(reply.vendor_id);
  _hostname = std::string(reply.name.data());
  _device_name = std::string(reply.devicename.data());
}

std::string
ifm3d::IFMNetworkDevice::GetIPAddress() const
{
  return _ip_address;
}

std::string
ifm3d::IFMNetworkDevice::GetMACAddress() const
{
  return _mac;
}

std::string
ifm3d::IFMNetworkDevice::GetNetmask() const
{
  return _subnet;
}

std::string
ifm3d::IFMNetworkDevice::GetGateway() const
{
  return _gateway;
}

uint16_t
ifm3d::IFMNetworkDevice::GetPort() const
{
  return _port;
}

uint16_t
ifm3d::IFMNetworkDevice::GetFlag() const
{
  return _flags;
}

std::string
ifm3d::IFMNetworkDevice::GetHostName() const
{
  return _hostname;
}

std::string
ifm3d::IFMNetworkDevice::GetDeviceName() const
{
  return _device_name;
}

uint16_t
ifm3d::IFMNetworkDevice::GetVendorId() const
{
  return _vendor_id;
}

uint16_t
ifm3d::IFMNetworkDevice::GetDeviceId() const
{
  return _device_id;
}

std::string
ifm3d::IFMNetworkDevice::GetFoundVia() const
{
  return _found_via;
}

constexpr size_t MAX_UDP_PACKET_SIZE = 65535;
using Data = std::vector<unsigned char>;
constexpr size_t TIMEOUT_FOR_EACH_SEARCH_IN_MS = 3000;

ifm3d::UDPConnection::UDPConnection(asio::io_context& context,
                                    asio::ip::udp::endpoint& local_endpoint)
  : _socket(context),
    _timer(context)
{
  _socket.open(asio::ip::udp::v4());
  _data.resize(MAX_UDP_PACKET_SIZE);
  _socket.set_option(asio::socket_base::broadcast(true));
  _socket.set_option(asio::ip::udp::socket::reuse_address(true));
  _socket.bind(local_endpoint);
}

/*Sends the data on the endpoint
 *@param data Data to be send on endpoint
 *@param Endpoint to send the data.
 **/
void
ifm3d::UDPConnection::Send(const Data& data,
                           asio::ip::udp::endpoint& remote_endpoint)
{
  _socket.async_send_to(
    asio::buffer(reinterpret_cast<const char*>(data.data()), data.size()),
    remote_endpoint,
    [this](auto&& error, auto&& bytes_transferred) {
      handle_send(
        std::forward<decltype(error)>(error),
        std::forward<decltype(bytes_transferred)>(bytes_transferred));
    });
}

/*Sends the data on the endpoint
 *@param data Data to be send on endpoint
 *@param Endpoint to send the data.
 **/
void
ifm3d::UDPConnection::SendIPChangeCall(
  Data& data,
  asio::ip::udp::endpoint& remote_endpoint)
{
  const int offset_port_in_data = offsetof(ifm3d::BcastIPChange, reply_port);

  if (offset_port_in_data > 0 && offset_port_in_data < data.size())
    {
      const unsigned short local_port = ntohs(_socket.local_endpoint().port());
      data[offset_port_in_data] = (local_port & 0xFF);
      data[offset_port_in_data + 1] = ((local_port >> 8) & 0xFF);
    }

  _socket.async_send_to(
    asio::buffer(reinterpret_cast<const char*>(data.data()), data.size()),
    remote_endpoint,
    [this](auto&& error, auto&& bytes_transferred) {
      handle_send(
        std::forward<decltype(error)>(error),
        std::forward<decltype(bytes_transferred)>(bytes_transferred));
    });
}

/*Grab data on the local endpoint till the timeout occur*/
void
ifm3d::UDPConnection::GrabData()
{
  receive();
  check_timeout();
}

void
ifm3d::UDPConnection::Close()
{
  _socket.close();
}

/*Provides interface to register callback on data received */
void
ifm3d::UDPConnection::RegisterOnReceive(
  std::function<void(asio::ip::udp::endpoint const&, Data, size_t)>
    on_recvieve)
{
  this->_on_receive = std::move(on_recvieve);
}

/*Provides interface to register callback on timeout or for any
 * error*/
void
ifm3d::UDPConnection::RegisterOnClose(
  std::function<void(std::shared_ptr<UDPConnection>)> on_close)
{
  this->_on_close = std::move(on_close);
}

void
ifm3d::UDPConnection::handle_send(const asio::error_code& error,
                                  std::size_t /*bytes_transferred*/)
{
  /* udp we send and forget so nothing to do in this callback*/
}

void
ifm3d::UDPConnection::receive()
{
  /* start the timer for timeout of 3 secs, if data do not come
     within timeout then connection will be closed and notified to parent
   */
  _timer.expires_from_now(
    std::chrono::milliseconds(TIMEOUT_FOR_EACH_SEARCH_IN_MS));
  _data.clear();
  _data.resize(MAX_UDP_PACKET_SIZE);

  try
    {
      asio::ip::udp::endpoint sender_endpoint;
      _socket.async_receive_from(
        asio::buffer(_data.data(), _data.size()),
        sender_endpoint,
        [this, sender = _socket.local_endpoint()](auto&& error,
                                                  auto&& bytes_transferred) {
          handle_receive(
            sender,
            std::forward<decltype(error)>(error),
            std::forward<decltype(bytes_transferred)>(bytes_transferred));
        });
    }
  catch (std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
}

void
ifm3d::UDPConnection::handle_receive(asio::ip::udp::endpoint const& sender,
                                     const asio::error_code& error,
                                     size_t bytes_transferred)
{
  if (error)
    {
      return;
    }
  _on_receive(sender, _data, bytes_transferred);
  receive();
}

void
ifm3d::UDPConnection::close()
{
  _socket.close();
  // notify the caller through registered callback
  _on_close(shared_from_this());
}

void
ifm3d::UDPConnection::check_timeout()
{
  if (_timer.expires_at() <= std::chrono::system_clock::now())
    {
      close();
    }
  else
    {
      auto shared_this = this->shared_from_this();
      _timer.async_wait([shared_this](const asio::error_code&) {
        try
          {
            shared_this->check_timeout();
          }
        catch (const std::system_error& err)
          {
            if (err.code().category() == asio::system_category() &&
                err.code().value() == asio::error::bad_descriptor)
              {
                // Socket already closed
              }
            else
              {
                std::cout << err.code() << ": " << err.what() << '\n';
              }
          }
      });
    }
}

const std::map<size_t, std::function<size_t(size_t)>> PACKAGE_VALIDATION{
  {ifm3d::BCAST_MAGIC_REQUEST,
   [](size_t size) -> size_t { return sizeof(ifm3d::BcastRequest) - size; }},
  {ifm3d::BCAST_MAGIC_REPLY,
   [](size_t size) -> size_t { return sizeof(ifm3d::BcastReply) - size; }},
  {ifm3d::BCAST_MAGIC_IPCHANGE,
   [](size_t size) -> size_t { return sizeof(ifm3d::BcastIPChange) - size; }}};

const unsigned int THREADS_FOR_IO_OPERATIONS = 3;

ifm3d::IFMDeviceDiscovery::IFMDeviceDiscovery()
  : _work_guard(asio::make_work_guard(_io_context))
{
  for (unsigned int i = 0; i < THREADS_FOR_IO_OPERATIONS; i++)
    {
      _thread_pool.emplace_back([&] { _io_context.run(); });
    }
}

ifm3d::IFMDeviceDiscovery::~IFMDeviceDiscovery()
{
  /* stop the io_context */
  _io_context.stop();

  // wait for all threads to complete
  for (std::thread& thread : _thread_pool)
    {
      if (thread.joinable())
        {
          thread.join();
        }
    }
}

std::vector<ifm3d::IFMNetworkDevice>
ifm3d::IFMDeviceDiscovery::NetworkSearch()
{
  _device_list.clear();

#ifdef __unix__

  /* Creating universal listener udp connecton */
  asio::ip::udp::endpoint endpoint(asio::ip::address_v4::any(),
                                   BCAST_DEFAULT_PORT);
  auto con = std::make_shared<UDPConnection>(_io_context, endpoint);

  con->RegisterOnReceive(
    [this](auto&& sender, auto&& data, auto&& bytes_transferred) {
      on_receive(std::forward<decltype(sender)>(sender),
                 std::forward<decltype(data)>(data),
                 std::forward<decltype(bytes_transferred)>(bytes_transferred));
    });
  con->RegisterOnClose([this](auto&& sender) {
    remove_connection(std::forward<decltype(sender)>(sender));
  });

  _connection_list.push_back(con);
#endif

  try
    {
      auto addresses = get_all_interface_address();

      /* broadcast ifm discovery request on all interfaces */
      for (const auto& address : addresses)
        {
          broadcast_on_interface(address);
        }
      /* get the device list  this will block the thread
         till all NIC are scanned for devices */
      auto devices = get_device_list();
    }
  catch (std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  return _device_list;
}

void
ifm3d::IFMDeviceDiscovery::SetTemporaryIP(const std::string& mac_address,
                                          const std::string& temp_ip)
{
  auto addresses = get_all_interface_address();

  /* broadcast IP change request on all interfaces */
  for (const auto& address : addresses)
    {
      send_ip_change_broadcast(mac_address, temp_ip, address);
    }
}

std::vector<std::string>
ifm3d::IFMDeviceDiscovery::get_all_interface_address()
{
  std::vector<std::string> addresses;
#ifdef __unix__
  struct ifaddrs* ifap = nullptr;
  struct ifaddrs* ifa = nullptr;
  struct sockaddr_in* sa = nullptr;
  char* addr = nullptr;

  getifaddrs(&ifap);
  for (ifa = ifap; ifa; ifa = ifa->ifa_next)
    {
      if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET)
        {
          sa = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
          addr = inet_ntoa(sa->sin_addr);
          const std::string ip_address = std::string(addr);
          addresses.push_back(ip_address);
        }
    }
  freeifaddrs(ifap);
#else

  asio::ip::udp::resolver resolver(_io_context);
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
ifm3d::IFMDeviceDiscovery::broadcast_on_interface(
  const std::string& interface_ip)
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

  auto con = std::make_shared<UDPConnection>(_io_context, local_endpoint);
  con->RegisterOnReceive(
    [this](auto&& sender, auto&& data, auto&& bytes_transferred) {
      on_receive(std::forward<decltype(sender)>(sender),
                 std::forward<decltype(data)>(data),
                 std::forward<decltype(bytes_transferred)>(bytes_transferred));
    });
  con->RegisterOnClose([this](auto&& con) {
    remove_connection(std::forward<decltype(con)>(con));
  });

  /* send BCAST_REQUEST on all interfaces */
  con->Send(data, broadcast_endpoint);

  _connection_list.push_back(con);
}

/*Checks the data for corresponding request on magic entry. */
std::tuple<bool, uint32_t, size_t>
ifm3d::IFMDeviceDiscovery::is_response_complete(const Data& data,
                                                const size_t byte_recv)
{
  uint32_t magic_value = 0;
  std::memcpy(&magic_value, data.data(), sizeof(uint32_t));

  magic_value = ntohl(magic_value);

  if (PACKAGE_VALIDATION.find(magic_value) != PACKAGE_VALIDATION.end())
    {
      auto required_length = PACKAGE_VALIDATION.at(magic_value)(byte_recv);
      return std::make_tuple(required_length == 0,
                             magic_value,
                             required_length);
    }

  std::cerr << "unknown response for discovery" << '\n';
  return {};
}

/* Handles the data packet, parse the reply, creates the
 * IFMNetworkDevice */
void
ifm3d::IFMDeviceDiscovery::on_receive(asio::ip::udp::endpoint const& sender,
                                      Data data,
                                      size_t bytes_transferred)
{
  bool iscomplete = false;
  size_t required = 0;
  uint32_t magic_value = 0;
  std::tie(iscomplete, magic_value, required) =
    is_response_complete(data, bytes_transferred);

  if (iscomplete)
    {
      if (magic_value == BCAST_MAGIC_REPLY)
        {
          const IFMNetworkDevice ifm_device(data,
                                            sender.address().to_string());
          const std::lock_guard<std::mutex> lock(_device_list_lock);
          _device_list.push_back(ifm_device);
        }
      if (magic_value == BCAST_MAGIC_IPCHANGE)
        {
          std::cout << '\n' << "BCAST_IPCHANGE Packet received";
        }
    }
}
/* return the discovered device list*/
std::vector<ifm3d::IFMNetworkDevice>
ifm3d::IFMDeviceDiscovery::get_device_list()
{
  auto connection_list = _connection_list;
  for (auto& con : connection_list)
    {
      con->GrabData();
    }
  /*block the main thread till the search is complete
  which is the condition when all the there is no respone on any interface
  for more than  TIMEOUT_FOR_EACH_SEARCH_IN_MS*/
  std::unique_lock<std::mutex> lock_to_complete(_con_mutex);
  _cv.wait(lock_to_complete, [&] { return _connection_list.empty(); });
  return _device_list;
}

/* remove the connection from the connection list */
void
ifm3d::IFMDeviceDiscovery::remove_connection(
  std::shared_ptr<UDPConnection> con)
{
  const std::lock_guard<std::mutex> lock(_con_mutex);
  _connection_list.erase(
    std::remove(std::begin(_connection_list), std::end(_connection_list), con),
    std::end(_connection_list));
  for (const auto& con : _connection_list)
    {
      con->Close();
    }
  _connection_list.resize(0);
  _cv.notify_one();
}

/* Broadcast the IP change request */
void
ifm3d::IFMDeviceDiscovery::send_ip_change_broadcast(
  const std::string& mac_address,
  const std::string& temp_ip,
  const std::string& interface_ip)
{
  try
    {
      BcastIPChange ipc{};
      Data data;
      data.resize(sizeof(BcastIPChange));
      ipc.magic = htonl(BCAST_MAGIC_IPCHANGE);
      ipc.temp_ip = htonl(str_to_address(temp_ip));
      std::array<uint8_t, 6> mac_array = mac_str_to_uint8(mac_address);
      uint8_t* mac = mac_array.data();
      memcpy(ipc.mac.data(), mac, ipc.mac.size());
      std::memcpy(data.data(), &ipc, sizeof(BcastIPChange));

      /** getting the netmask of the interface */
      auto netmask = asio::ip::address_v4::netmask(
        asio::ip::address_v4::from_string(interface_ip));

      /**broadcast of the interface */
      auto broadcast_address_of_interface =
        asio::ip::address_v4::broadcast(
          asio::ip::address_v4::from_string(interface_ip),
          netmask)
          .to_string();

      /* create a local broadcast endpoint */
      asio::ip::udp::endpoint broadcast_endpoint = asio::ip::udp::endpoint(
        asio::ip::address::from_string(broadcast_address_of_interface),
        BCAST_DEFAULT_PORT);

      /* create a local endpoint */
      asio::ip::udp::endpoint local_endpoint =
        asio::ip::udp::endpoint(asio::ip::udp::v4(), BCAST_DEFAULT_PORT);

      auto con = std::make_shared<UDPConnection>(_io_context, local_endpoint);
      con->RegisterOnReceive(
        [this](auto&& sender, auto&& data, auto&& bytes_transferred) {
          on_receive(
            std::forward<decltype(sender)>(sender),
            std::forward<decltype(data)>(data),
            std::forward<decltype(bytes_transferred)>(bytes_transferred));
        });
      con->RegisterOnClose([this](auto&& sender) {
        remove_connection(std::forward<decltype(sender)>(sender));
      });

      /* send BcastIPChange to device */
      con->SendIPChangeCall(data, broadcast_endpoint);

      _connection_list.push_back(con);
    }
  catch (std::exception& e)
    {
      std::cerr << '\n' << "ifm3d exception: " << e.what() << '\n';
    }
}
