/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H
#define IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <ifm3d/common/json.hpp>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/err.h>
#include <ifm3d/device/legacy_device.h>
#include <memory>
#include <mutex>
#include <string>
#include <swupdater_impl.hpp>
#include <thread>
#include <utility>
#include <websocketpp/client.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

using client = websocketpp::client<websocketpp::config::asio_client>;

namespace ifm3d
{
  const ifm3d::SemVer MIN_O3R_FIRMWARE_RECOVERY_UPDATE =
    ifm3d::SemVer(1, 0, 0);

  const std::string SWUPDATER_V2_UPLOAD_URL_SUFFIX = "/upload";
  const std::string SWUPDATER_V2_REBOOT_URL_SUFFIX = "/restart";
  const std::string SWUPDATER_V2_STATUS_URL_SUFFIX = "/ws";

  /* types */
  const std::string SWUPATER_V2_TYPE_STEP = "step";
  const std::string SWUPATER_V2_TYPE_INFO = "info";
  const std::string SWUPATER_V2_TYPE_MESSAGE = "message";
  const std::string SWUPATER_V2_TYPE_STATUS = "status";
  const std::string SWUPATER_V2_TYPE_SOURCE = "source";

  /* message level */
  const int SWUPATER_V2_MESSAGE_LEVEL_ERROR = 3;
  const int SWUPATER_V2_MESSAGE_LEVEL_INFO = 6;

  /* Status values */
  const std::string SWUPATER_V2_STATUS_IDLE = "IDLE";
  const std::string SWUPATER_V2_STATUS_START = "START";
  const std::string SWUPATER_V2_STATUS_RUN = "RUN";
  const std::string SWUPATER_V2_STATUS_SUCCESS = "SUCCESS";
  const std::string SWUPATER_V2_STATUS_DONE = "DONE";

  /* curl parameters */
  const int SWUPDATE_V2_TIMEOUT_FOR_UPLOAD =
    std::getenv("IFM3D_SWUPDATE_CURL_TIMEOUT") == nullptr ?
      600 :
      std::stoi(std::getenv("IFM3D_SWUPDATE_CURL_TIMEOUT"));
  const int CURL_MAX_REDIR = 50;
  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT ImplV2 : public SWUpdater::Impl
  {
  public:
    ImplV2(const ImplV2&) = delete;
    ImplV2(ImplV2&&) = delete;
    ImplV2& operator=(const ImplV2&) = delete;
    ImplV2& operator=(ImplV2&&) = delete;
    ImplV2(ifm3d::Device::Ptr cam,
           const ifm3d::SWUpdater::FlashStatusCb& cb,
           const std::string& swupdate_recovery_port);
    ~ImplV2() override = default;

    void RebootToRecovery() override;
    void RebootToProductive() override;
    bool FlashFirmware(const std::string& swu_file,
                       long timeout_millis) override;

  private:
    bool check_recovery() override;
    bool check_productive() override;
    void upload_firmware(const std::string& swu_file,
                         long timeout_millis) override;
    void on_web_socket_data(const std::string& json_string);

    /* Wrapper over websocketpp*/
    class WebSocketEndpoint
    {
    public:
      WebSocketEndpoint()
      {
        _endpoint.clear_access_channels(websocketpp::log::alevel::all);
        _endpoint.clear_error_channels(websocketpp::log::elevel::all);

        _endpoint.init_asio();
        _endpoint.start_perpetual();

        _thread =
          std::make_shared<websocketpp::lib::thread>(&client::run, &_endpoint);
      }

      WebSocketEndpoint(const WebSocketEndpoint&) = delete;
      WebSocketEndpoint(WebSocketEndpoint&&) = delete;
      WebSocketEndpoint& operator=(const WebSocketEndpoint&) = delete;
      WebSocketEndpoint& operator=(WebSocketEndpoint&&) = delete;
      ~WebSocketEndpoint()
      {
        _endpoint.stop_perpetual();
        websocketpp::lib::error_code ec;
        if (!_hdl.expired())
          {
            _endpoint.close(_hdl,
                            websocketpp::close::status::going_away,
                            "",
                            ec);
            if (ec)
              {
                // Ignore because the device will force close the connection
                // after the update finished
              }
          }
        _thread->join();
      }

      void
      Close(websocketpp::close::status::value code)
      {
        websocketpp::lib::error_code ec;
        _endpoint.close(_hdl, code, "", ec);
        if (ec)
          {
            LOG_ERROR("> Error initiating close: {}", ec.message());
          }
      }

      int
      Connect(std::string const& uri)
      {
        websocketpp::lib::error_code ec;

        client::connection_ptr con = _endpoint.get_connection(uri, ec);

        if (ec)
          {
            LOG_INFO("> Connect initialization error: {}", ec.message());
            return -1;
          }
        _hdl = con->get_handle();

        con->set_open_handler([this, client = &_endpoint](auto&& conn) {
          on_open(client, std::forward<decltype(conn)>(conn));
        });
        con->set_fail_handler([this, client = &_endpoint](auto&& conn) {
          on_fail(client, std::forward<decltype(conn)>(conn));
        });
        con->set_message_handler([this](auto&& conn, auto&& msg) {
          on_message(std::forward<decltype(conn)>(conn),
                     std::forward<decltype(msg)>(msg));
        });

        _endpoint.connect(con);
        return 0;
      }

      void
      RegisterOnDataRecv(std::function<void(const std::string&)> on_data_recv)
      {
        _cb_data_recv = std::move(on_data_recv);
      }

    private:
      void
      on_open(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(std::move(hdl));
        std::string server = con->get_response_header("Server");
        LOG_INFO(server.c_str());
      }

      void
      on_fail(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(std::move(hdl));
        std::string server = con->get_response_header("Server");
        std::string error_msg = con->get_ec().message();
        LOG_INFO("{}: {}", server.c_str(), error_msg.c_str());
      }
      void
      on_message(const websocketpp::connection_hdl& /*hdl*/,
                 client::message_ptr msg)
      {
        std::string status;
        if (msg->get_opcode() == websocketpp::frame::opcode::text)
          {
            status = msg->get_payload();
          }
        else
          {
            status = websocketpp::utility::to_hex(msg->get_payload());
          }
        this->_cb_data_recv(status);
        // VLOG(IFM3D_TRACE_DEEP) << status.c_str();
      }
      void
      on_close(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(std::move(hdl));
        LOG_INFO(
          "close code: {} ({}), close reason: {}",
          con->get_remote_close_code(),
          websocketpp::close::status::get_string(con->get_remote_close_code()),
          con->get_remote_close_reason());
      }

      client _endpoint;
      websocketpp::lib::shared_ptr<websocketpp::lib::thread> _thread;
      websocketpp::connection_hdl _hdl;
      std::function<void(const std::string&)> _cb_data_recv;
    };

    std::unique_ptr<WebSocketEndpoint> _websocket;
    std::mutex _m;
    std::condition_variable _cv;
    std::string _sw_status;
    bool _upload_error{};
  }; // end: class ImplV2

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor
//-------------------------------------
inline ifm3d::ImplV2::ImplV2(ifm3d::Device::Ptr cam,
                             const ifm3d::SWUpdater::FlashStatusCb& cb,
                             const std::string& swupdate_recovery_port)
  : ifm3d::SWUpdater::Impl(cam, cb, swupdate_recovery_port),
    _websocket(std::make_unique<WebSocketEndpoint>()),
    _sw_status(SWUPATER_V2_STATUS_IDLE)

{
  this->_client.set_write_timeout(ifm3d::SWUPDATE_V2_TIMEOUT_FOR_UPLOAD);

  _websocket->RegisterOnDataRecv([this](auto&& data) {
    on_web_socket_data(std::forward<decltype(data)>(data));
  });
}

//-------------------------------------
// "Public" interface
//-------------------------------------
inline void
ifm3d::ImplV2::RebootToRecovery()
{
  if (this->_cam->FirmwareVersion() >= MIN_O3R_FIRMWARE_RECOVERY_UPDATE)
    {
      this->_cam->Reboot(ifm3d::Device::BootMode::RECOVERY);
    }
}

inline void
ifm3d::ImplV2::RebootToProductive()
{
  try
    {
      auto res = this->_client.Post(SWUPDATER_V2_REBOOT_URL_SUFFIX,
                                    "",
                                    "application/x-www-form-urlencoded");

      ifm3d::check_http_result(res);
    }
  catch (const ifm3d::Error& e)
    {
      // Rethrow unless the code is one that indicates the camera is not
      // currently reachable (occurs during the reboot process)
      if (e.code() != IFM3D_CURL_TIMEOUT &&
          e.code() != IFM3D_RECOVERY_CONNECTION_ERROR)
        {
          throw;
        }
    }
}

inline bool
ifm3d::ImplV2::FlashFirmware(const std::string& swu_file, long timeout_millis)
{
  auto t_start = std::chrono::system_clock::now();
  long remaining_time = timeout_millis;
  auto get_remaining_time = [&t_start, timeout_millis]() -> long {
    auto t_now = std::chrono::system_clock::now();
    auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start);
    return timeout_millis - static_cast<long>(elapsed.count());
  };
  /* connect to websocket */
  _websocket->Connect(fmt::format("ws://{}:{}{}",
                                  _client.host(),
                                  SWUPDATER_RECOVERY_PORT,
                                  SWUPDATER_V2_STATUS_URL_SUFFIX));
  /*upload buffer */
  this->upload_firmware(swu_file, remaining_time);
  std::unique_lock<std::mutex> lk(_m);
  _cv.wait(lk, [&] {
    return (this->_sw_status == SWUPATER_V2_STATUS_SUCCESS ||
            this->_sw_status == SWUPATER_V2_STATUS_DONE) ||
           this->_upload_error;
  });

  _websocket->Close(websocketpp::close::status::going_away);

  /* give  10 sec to execute restart*/
  if (this->_sw_status == SWUPATER_V2_STATUS_SUCCESS ||
      this->_sw_status == SWUPATER_V2_STATUS_DONE)
    {
      std::this_thread::sleep_for(std::chrono::seconds(10));
    }
  return !this->_upload_error;
}

//-------------------------------------
// "Private" helpers
//-------------------------------------
inline bool
ifm3d::ImplV2::check_recovery()
{
  auto res = this->_client.Head("/");

  if (!res)
    {
      if (res.error() == httplib::Error::Connection ||
          res.error() == httplib::Error::ConnectionTimeout)
        {
          return false;
        }
      throw ifm3d::Error(IFM3D_RECOVERY_CONNECTION_ERROR);
    }

  return res->status == 200;
}

inline bool
ifm3d::ImplV2::check_productive()
{
  try
    {
      if (this->_cam->DeviceParameter("OperatingMode") != "")
        {
          return true;
        }
    }
  catch (const ifm3d::Error& e)
    {
      // Rethrow unless the code is one that indicates the camera is not
      // currently reachable by XML-RPC (occurs during the reboot process)
      if (e.code() != IFM3D_XMLRPC_TIMEOUT &&
          e.code() != IFM3D_XMLRPC_OBJ_NOT_FOUND &&
          e.code() != IFM3D_CURL_ERROR)
        {
          throw;
        }
    }

  return false;
}

inline void
ifm3d::ImplV2::upload_firmware(const std::string& swu_file,
                               long /*timeout_millis*/)
{
  std::FILE* input = fopen_read(swu_file);

  std::array<char, 4096> buffer{};
  httplib::MultipartFormDataProviderItems items = {
    {SWUPDATER_MIME_PART_NAME,
     [&](size_t /*offset*/, httplib::DataSink& sink) -> bool {
       auto c = fread(buffer.data(), 1, buffer.size(), input);

       if (c > 0)
         {
           return sink.write(buffer.data(), c);
         }

       if (feof(input))
         {
           sink.done();
         }

       return ferror(input) == 0;
     },
     SWUPDATER_FILENAME,
     SWUPDATER_CONTENT_TYPE_HEADER}};

  auto res = this->_client.Post(SWUPDATER_V2_UPLOAD_URL_SUFFIX, {}, {}, items);

  fclose_read(input);

  // Ignore Error::Write because the device will force close the connection
  // after the update finishes
  if (res.error() != httplib::Error::Write)
    {
      ifm3d::check_http_result(res);
    }
}

inline void
ifm3d::ImplV2::on_web_socket_data(const std::string& json_string)
{
  ifm3d::json json;
  try
    {
      json = ifm3d::json::parse(json_string.c_str());
      const auto type = json["type"].get<std::string>();

      if (type == SWUPATER_V2_TYPE_STEP)
        {
          const auto number_of_steps =
            std::stoi(json["number"].get<std::string>());
          const auto current_step = std::stoi(json["step"].get<std::string>());

          const auto percent_of_current_step =
            std::stoi(json["percent"].get<std::string>());

          const auto filename = json["name"].get<std::string>();

          const auto overall_percent =
            (static_cast<float>(current_step) /
             static_cast<float>(number_of_steps)) *
            (static_cast<float>(percent_of_current_step) / 100.0F);

          this->_cb(overall_percent, filename);
        }

      if (type == SWUPATER_V2_TYPE_MESSAGE)
        {
          const auto level = std::stoi(json["level"].get<std::string>());
          const auto message = json["text"].get<std::string>();
          this->_cb(1.0F, "Message : " + message);
        }

      if (type == SWUPATER_V2_TYPE_STATUS)
        {
          auto const status = json["status"].get<std::string>();
          if (status == SWUPATER_V2_STATUS_SUCCESS)
            {
              this->_sw_status = status;
              _cv.notify_all();
            }
          if (status == SWUPATER_V2_STATUS_DONE)
            {
              this->_sw_status = status;
              _cv.notify_all();
            }
          this->_cb(1.0F, "Status : " + status);
        }
    }
  catch (std::exception& e)
    {
      // ignore any status message parsing or json error
      // type info : shaowing json parser error
    }
}

#endif // IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H
