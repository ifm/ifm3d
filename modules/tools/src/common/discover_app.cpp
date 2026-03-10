/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fmt/base.h> // NOLINT(misc-header-include-cycle)
#include <fmt/format.h>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3c.h>
#include <ifm3d/device/o3d.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/o3x.h>
#include <ifm3d/tools/common/discover_app.h>
#include <ifm3d/tools/common/set_temp_ip_app.h>
#include <ifm3d/tools/legacy/o3d3xx_app.h>
#include <ifm3d/tools/legacy/o3x_app.h>
#include <ifm3d/tools/o3cxxx/o3cxxx_app.h>
#include <ifm3d/tools/ovp8xx/ovp8xx_app.h>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ifm3d::literals;

namespace
{
  constexpr uint32_t BCAST_FLAG_WRONGSUBNET = 0x0001;

  struct TableEntry
  {
    std::string ip_address;
    std::string mac_address;
    std::optional<std::string> device_name;
    std::optional<std::string> device_identifier;
    std::string notes;
  };

  void
  print_device_table(std::vector<TableEntry>& table_entries)
  {
    static auto CALC_COLUMN_WIDTH = [](const auto& table_entries,
                                       const auto& accessor) {
      size_t width = 0;
      for (const auto& entry : table_entries)
        {
          width = std::max(width, accessor(entry).size());
        }
      return width;
    };

    static auto FORMAT_NAME_COLUMN = [](const auto& entry) {
      if (entry.device_name.has_value() && entry.device_identifier.has_value())
        {
          return fmt::format("{}/{}",
                             entry.device_name.value(),
                             entry.device_identifier.value());
        }

      if (entry.device_name.has_value())
        {
          return entry.device_name.value();
        }

      if (entry.device_identifier.has_value())
        {
          return entry.device_identifier.value();
        }

      return std::string{"Unknown"};
    };

    static const std::string HEADER_IP = "IP";
    static const std::string HEADER_MAC = "MAC";
    static const std::string HEADER_NAME = "Identifier";
    static const std::string HEADER_NOTES = "Notes";

    auto ip_width =
      std::max(CALC_COLUMN_WIDTH(table_entries,
                                 [](const auto& e) { return e.ip_address; }),
               HEADER_IP.size());
    auto mac_width =
      std::max(CALC_COLUMN_WIDTH(table_entries,
                                 [](const auto& e) { return e.mac_address; }),
               HEADER_MAC.size());
    auto identifier_width = std::max(
      CALC_COLUMN_WIDTH(table_entries,
                        [](const auto& e) { return FORMAT_NAME_COLUMN(e); }),
      HEADER_NAME.size());
    auto notes_width = std::max(
      CALC_COLUMN_WIDTH(table_entries, [](const auto& e) { return e.notes; }),
      HEADER_NOTES.size());

    fmt::print("┌{0:─^{1}}┬{0:─^{2}}┬{0:─^{3}}┬{0:─^{4}}┐\n",
               "",
               ip_width + 2,
               mac_width + 2,
               identifier_width + 2,
               notes_width + 2);

    fmt::print("│ {:^{}} │ {:^{}} │ {:^{}} │ {:^{}} |\n",
               HEADER_IP,
               ip_width,
               HEADER_MAC,
               mac_width,
               HEADER_NAME,
               identifier_width,
               HEADER_NOTES,
               notes_width);

    fmt::print("├{0:─^{1}}┼{0:─^{2}}┼{0:─^{3}}┼{0:─^{4}}┤\n",
               "",
               ip_width + 2,
               mac_width + 2,
               identifier_width + 2,
               notes_width + 2);

    for (const auto& entry : table_entries)
      {
        fmt::print("| {:<{}} | {:<{}} | {:<{}} | {:<{}} |\n",
                   entry.ip_address,
                   ip_width,
                   entry.mac_address,
                   mac_width,
                   FORMAT_NAME_COLUMN(entry),
                   identifier_width,
                   entry.notes,
                   notes_width);
      }

    fmt::print("└{0:─^{1}}┴{0:─^{2}}┴{0:─^{3}}┴{0:─^{4}}┘\n",
               "",
               ip_width + 2,
               mac_width + 2,
               identifier_width + 2,
               notes_width + 2);
  }
}

ifm3d::DiscoverApp::~DiscoverApp() = default;

void
ifm3d::DiscoverApp::Execute(CLI::App* /*app*/)
{
  if (!(this->subcmd_set_temp_ip->parsed()))
    {
      auto devices = ifm3d::Device::DeviceDiscovery();

      std::unordered_map<std::string, decltype(devices)::value_type>
        unique_devices;

      for (const auto& device : devices)
        {
          auto mac = device.GetMACAddress();
          auto existing = unique_devices.find(mac);

          if (existing == unique_devices.end() ||
              (existing->second.HasFlag(BCAST_FLAG_WRONGSUBNET) &&
               !device.HasFlag(BCAST_FLAG_WRONGSUBNET)))
            {
              unique_devices.insert_or_assign(mac, device);
            }
        }

      auto table_entries = std::vector<TableEntry>{};
      if (!unique_devices.empty())
        {
          bool subnet_notice = false;
          for (const auto& [mac, device] : unique_devices)
            {
              auto ip_address = device.GetIPAddress();
              auto mac_address = device.GetMACAddress();
              auto device_name = device.GetDeviceName().empty() ?
                                   std::nullopt :
                                   std::make_optional(device.GetDeviceName());
              auto device_identifier =
                device.GetHostName().empty() ?
                  std::nullopt :
                  std::make_optional(device.GetHostName());

              auto notes = std::string{};

              try
                {
                  if (device.HasFlag(BCAST_FLAG_WRONGSUBNET))
                    {
                      notes = " (1)";
                      subnet_notice = true;
                    }
                  else
                    {
                      auto cam = ifm3d::Device::MakeShared(ip_address);
                      auto device_family = cam->WhoAmI();
                      std::optional<Device::DeviceFamily> device_command{};

                      if (Parent<O3D3XX>())
                        {
                          device_command = Device::DeviceFamily::O3D;
                        }
                      else if (Parent<O3X1XX_O3X2XX>())
                        {
                          device_command = Device::DeviceFamily::O3X;
                        }
                      else if (Parent<OVP8xx>())
                        {
                          device_command = Device::DeviceFamily::O3R;
                        }
                      else if (Parent<O3Cxxx>())
                        {
                          device_command = Device::DeviceFamily::O3C;
                        }

                      if (device_command.has_value() &&
                          device_family != device_command.value())
                        {
                          // This device is not compatible with the current
                          // subcommand, so we skip it.
                          continue;
                        }

                      switch (device_family)
                        {
                        case Device::DeviceFamily::UNKNOWN:
                          device_identifier = "Unknown device";
                          break;

                        case Device::DeviceFamily::O3D:
                          if (auto s =
                                std::static_pointer_cast<ifm3d::O3D>(cam)
                                  ->DeviceParameter("Name");
                              !s.empty())
                            {
                              device_name = s;
                            }
                          else
                            {
                              device_identifier = "O3D";
                            }
                          break;

                        case Device::DeviceFamily::O3X:
                          if (auto s =
                                std::static_pointer_cast<ifm3d::O3X>(cam)
                                  ->DeviceParameter("Name");
                              !s.empty())
                            {
                              device_name = s;
                            }
                          else
                            {
                              device_identifier = "O3X";
                            }
                          break;

                        case Device::DeviceFamily::O3R:
                          if (auto s =
                                std::static_pointer_cast<ifm3d::O3R>(cam)
                                  ->ResolveConfig(
                                    "/device/info/name"_json_pointer)
                                  .get<std::string>();
                              !s.empty())
                            {
                              device_name = s;
                            }
                          else
                            {
                              device_identifier = "O3R";
                            }
                          break;

                        case Device::DeviceFamily::O3C:
                          if (auto s =
                                std::static_pointer_cast<ifm3d::O3C>(cam)
                                  ->ResolveConfig(
                                    "/device/info/name"_json_pointer)
                                  .get<std::string>();
                              !s.empty())
                            {
                              device_name = s;
                            }
                          else
                            {
                              device_identifier = "O3C";
                            }
                          break;
                        }
                    }
                }
              catch (ifm3d::Error& e)
                {
                  if (e.code() == ifm3d::Error(IFM3D_XMLRPC_TIMEOUT).code() ||
                      e.code() == ifm3d::Error(IFM3D_CURL_TIMEOUT).code())
                    {
                      device_identifier = "Unable to identify";
                    }
                }
              catch (std::exception& exp)
                {
                  // IGNORE: ignore this device and search for next devices..
                  LOG_ERROR("Error occured while detecting device: {}",
                            exp.what());
                  continue;
                }

              table_entries.push_back({ip_address,
                                       mac_address,
                                       device_name,
                                       device_identifier,
                                       notes});
            }

          print_device_table(table_entries);

          if (subnet_notice)
            {
              std::cout << "\n(1) Device is in a different subnet. Use the "
                           "'set-temporary-ip' subcommand to assign a "
                           "temporary IP for access."
                        << '\n';
            }
        }
      if (devices.empty())
        {
          std::cout << "Info: No devices available" << '\n';
        }
    }
}

CLI::App*
ifm3d::DiscoverApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("discover", "Discover ifm devices on the network.")
      ->require_subcommand(0, 1);

  subcmd_set_temp_ip =
    RegisterSubcommand<ifm3d::SetTemporaryIPApp>(command)->GetSubcommandApp();

  return command;
}

bool
ifm3d::DiscoverApp::CheckCompatibility()
{
  return true;
}
