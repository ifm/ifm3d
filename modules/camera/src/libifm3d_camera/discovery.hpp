// -*- c++ -*-
/*
 * Copyright (C) 2020 ifm electronics, gmbh
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

  /** @brief It signalize that network device interface ip address is set via dhcp. */
  constexpr uint32_t BCAST_FLAG_BYDHCP = 0x0002;

  /** @brief It signalize that network device interface ip address is temporary. */
  constexpr uint32_t BCAST_FLAG_ISTEMPORARY = 0x0004;

  /** @brief Reserved. */
  constexpr uint32_t BCAST_FLAG_WITHPRGTYPE = 0x8000;


  /** @brief Broadcast request packet structure. */
  struct BcastRequest {
    /** @brief Default magic number 0x1020EFCF. */
    const uint32_t magic;
    /** @brief Port to send reply to. */
    const uint16_t reply_port;

    /** @brief Unused. */
    const uint16_t reserved;

    BcastRequest():magic(htonl(BCAST_MAGIC_REQUEST)), reply_port(htons(BCAST_DEFAULT_PORT)), reserved(0)
    {}
  };

  /** @brief Broadcast reply packet structure. */
 struct BcastReply {
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
    uint8_t  mac[6];

    /** @brief Network device flags. */
    uint16_t flags;

    /** @brief Network device hostname */
    char     name[64];

    /** @brief device name defined by the user */
    char     devicename[256];

    void NetworktoHost()
    {
      ip_address = ntohl(ip_address);
      ip_gateway = ntohl(ip_gateway);
      ip_netmask = ntohl(ip_netmask);
      port = ntohs(port);
      flags = ntohs(flags);
      device_id = ntohs(device_id);
      vendor_id = ntohs(vendor_id);
    }
 };

 namespace
 {
   inline const std::string address2Str(const uint32_t addr)
   {
     std::stringstream ss;
     ss << ((addr >> 24) & 0xff) << "."
        << ((addr >> 16) & 0xff) << "."
        << ((addr >> 8) & 0xff)  << "."
        << ((addr >> 0) & 0xff);
     return ss.str();
   }

   inline const std::string uint8Mac2macStr(uint8_t *mac, size_t size = 6)
   {
     std::stringstream ss;
     auto converttoHexandAppend = [&](const uint8_t value)
     {
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

 struct IFMNetworkDevice
 {
   const std::string ip_address;
   const std::string mac;
   const std::string subnet;
   const std::string gateway;
   const uint16_t port;
   /** @brief the device gives some additional information via those flags */
   const uint16_t flags;
   const std::string hostname;
   /** @brief IP-address of local-interface on which the NetDevice was found */
   const std::string found_via;
   const std::string device_name;
   const uint16_t vendor_id;
   const uint16_t device_id;

   IFMNetworkDevice(const BcastReply& reply, const std::string& interface)
     : ip_address(address2Str(reply.ip_address))
     , gateway(address2Str(reply.ip_gateway))
     , subnet(address2Str(reply.ip_netmask))
     , mac(uint8Mac2macStr((uint8_t*)reply.mac))
     , port(reply.port)
     , flags(reply.flags)
     , device_id(reply.device_id)
     , vendor_id(reply.vendor_id)
     , hostname(std::string(reply.name))
     , device_name(std::string(reply.devicename))
     , found_via(interface)
   {}

   void Display()
   {
     std::cout << std::endl;
     std::cout << "ip-address : " << ip_address << std::endl;
     std::cout << "gateway : " << gateway << std::endl;
     std::cout << "subnet : " << subnet << std::endl;
     std::cout << "mac : " << mac << std::endl;
     std::cout << "port : " << port << std::endl;
     std::cout << "flags : " << flags << std::endl;
     std::cout << "device-id : " << device_id << std::endl;
     std::cout << "vendor_id : " << vendor_id << std::endl;
     std::cout << "hostname : " << hostname << std::endl;
     std::cout << "device-name : " << device_name << std::endl;
     std::cout << "found_via : " << found_via << std::endl;
   }
 };

}
#endif // IFM3D_CAMERA_DISCOVERY_HPP