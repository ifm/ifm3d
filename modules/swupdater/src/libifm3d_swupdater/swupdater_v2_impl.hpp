/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H
#define IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H

#include <cstdlib>
#include <map>
#include <sstream>
#include <chrono>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <limits>
#include <iostream>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/common/json.hpp>
#include <swupdater_impl.hpp>

using client = websocketpp::client<websocketpp::config::asio_client>;

namespace ifm3d
{
  const ifm3d::SemVer MIN_O3R_FIRMWARE_RECOVERY_UPDATE =
    ifm3d::SemVer(1, 0, 0);

  const std::string SWUPDATER_V2_UPLOAD_URL_SUFFIX = "/upload";
  const std::string SWUPDATER_V2_REBOOT_URL_SUFFIX = "/restart";
  const std::string SWUPDATER_V2_STATUS_URL_SUFFIX = "/ws";
  const std::string SWUPATER_V2_FILENAME = "swupdate.swu";

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
  class ImplV2 : public SWUpdater::Impl
  {
  public:
    ImplV2(ifm3d::Device::Ptr cam,
           const ifm3d::SWUpdater::FlashStatusCb& cb,
           const std::string& swupdate_recovery_port);
    ~ImplV2() = default;

    void RebootToRecovery() override;
    void RebootToProductive() override;
    bool FlashFirmware(const std::string& swu_file,
                       long timeout_millis) override;

  private:
    bool CheckRecovery() override;
    bool CheckProductive() override;
    void UploadFirmware(const std::string& swu_file,
                        long timeout_millis) override;
    void OnWebSocketData(const std::string json_string);

    /* Wrapper over websocketpp*/
    class WebSocketEndpoint
    {
    public:
      WebSocketEndpoint()
      {
        endpoint_.clear_access_channels(websocketpp::log::alevel::all);
        endpoint_.clear_error_channels(websocketpp::log::elevel::all);

        endpoint_.init_asio();
        endpoint_.start_perpetual();

        thread_.reset(new websocketpp::lib::thread(&client::run, &endpoint_));
      }

      ~WebSocketEndpoint()
      {
        endpoint_.stop_perpetual();
        websocketpp::lib::error_code ec;
        if (!hdl_.expired())
          {
            endpoint_.close(hdl_,
                            websocketpp::close::status::going_away,
                            "",
                            ec);
            if (ec)
              {
                LOG_ERROR("> Error closing connection ");
              }
          }
        thread_->join();
      }

      void
      Close(websocketpp::close::status::value code)
      {
        websocketpp::lib::error_code ec;
        endpoint_.close(hdl_, code, "", ec);
        if (ec)
          {
            LOG_ERROR("> Error initiating close: {}", ec.message());
          }
      }

      int
      connect(std::string const& uri)
      {
        websocketpp::lib::error_code ec;

        client::connection_ptr con = endpoint_.get_connection(uri, ec);

        if (ec)
          {
            LOG_INFO("> Connect initialization error: {}", ec.message());
            return -1;
          }
        hdl_ = con->get_handle();

        con->set_open_handler(
          websocketpp::lib::bind(&WebSocketEndpoint::OnOpen,
                                 this,
                                 &endpoint_,
                                 websocketpp::lib::placeholders::_1));
        con->set_fail_handler(
          websocketpp::lib::bind(&WebSocketEndpoint::OnFail,
                                 this,
                                 &endpoint_,
                                 websocketpp::lib::placeholders::_1));
        con->set_message_handler(
          websocketpp::lib::bind(&WebSocketEndpoint::OnMessage,
                                 this,
                                 websocketpp::lib::placeholders::_1,
                                 websocketpp::lib::placeholders::_2));

        endpoint_.connect(con);
        return 0;
      }

      void
      RegisterOnDataRecv(std::function<void(const std::string&)> on_data_recv)
      {
        cb_data_recv = on_data_recv;
      }

    private:
      void
      OnOpen(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        std::string server = con->get_response_header("Server");
        LOG_INFO(server.c_str());
      }

      void
      OnFail(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        std::string server = con->get_response_header("Server");
        std::string error_msg = con->get_ec().message();
        LOG_INFO("{}: {}", server.c_str(), error_msg.c_str());
      }
      void
      OnMessage(websocketpp::connection_hdl hdl, client::message_ptr msg)
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
        this->cb_data_recv(status);
        // VLOG(IFM3D_TRACE_DEEP) << status.c_str();
      }
      void
      OnClose(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        LOG_INFO(
          "close code: {} ({}), close reason: {}",
          con->get_remote_close_code(),
          websocketpp::close::status::get_string(con->get_remote_close_code()),
          con->get_remote_close_reason());
      }

      client endpoint_;
      websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread_;
      websocketpp::connection_hdl hdl_;
      std::function<void(const std::string&)> cb_data_recv;
    };

    std::unique_ptr<WebSocketEndpoint> websocket_;
    std::mutex m_;
    std::condition_variable cv_;
    std::string sw_status_;
    std::string main_url_;
    bool upload_error_;
  }; // end: class ImplV2

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor
//-------------------------------------
ifm3d::ImplV2::ImplV2(ifm3d::Device::Ptr cam,
                      const ifm3d::SWUpdater::FlashStatusCb& cb,
                      const std::string& swupdate_recovery_port)
  : ifm3d::SWUpdater::Impl(cam, cb, swupdate_recovery_port),
    websocket_(std::make_unique<WebSocketEndpoint>()),
    sw_status_(SWUPATER_V2_STATUS_IDLE),
    upload_error_(false)
{
  main_url_ = ("http://" + cam->IP() + ":" + swupdate_recovery_port);

  upload_url_ = main_url_ + SWUPDATER_V2_UPLOAD_URL_SUFFIX;
  reboot_url_ = main_url_ + SWUPDATER_V2_REBOOT_URL_SUFFIX;
  status_url_ = ("ws://" + cam->IP() + ":" + swupdate_recovery_port +
                 SWUPDATER_V2_STATUS_URL_SUFFIX);

  websocket_->RegisterOnDataRecv(
    std::bind(&ifm3d::ImplV2::OnWebSocketData, this, std::placeholders::_1));
}

//-------------------------------------
// "Public" interface
//-------------------------------------
void
ifm3d::ImplV2::RebootToRecovery()
{
  if (this->cam_->FirmwareVersion() >= MIN_O3R_FIRMWARE_RECOVERY_UPDATE)
    {
      this->cam_->Reboot(ifm3d::Device::boot_mode::RECOVERY);
    }
}

void
ifm3d::ImplV2::RebootToProductive()
{
  try
    {
      auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();
      c->Call(curl_easy_setopt, CURLOPT_URL, this->reboot_url_.c_str());
      c->Call(curl_easy_setopt, CURLOPT_POST, true);
      c->Call(curl_easy_setopt, CURLOPT_POSTFIELDSIZE, 0);
      c->Call(curl_easy_setopt,
              CURLOPT_WRITEFUNCTION,
              &ifm3d::SWUpdater::Impl::StatusWriteCallbackIgnore);
      c->Call(curl_easy_setopt,
              CURLOPT_CONNECTTIMEOUT,
              ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
      c->Call(curl_easy_setopt,
              CURLOPT_TIMEOUT,
              ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);
      c->Call(curl_easy_perform);
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

bool
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
  websocket_->connect(status_url_);
  /*upload buffer */
  this->UploadFirmware(swu_file, remaining_time);
  std::unique_lock<std::mutex> lk(m_);
  cv_.wait(lk, [&] {
    return (this->sw_status_ == SWUPATER_V2_STATUS_SUCCESS ||
            this->sw_status_ == SWUPATER_V2_STATUS_DONE) ||
           this->upload_error_;
  });

  websocket_->Close(websocketpp::close::status::going_away);

  /* give  10 sec to execute restart*/
  if (this->sw_status_ == SWUPATER_V2_STATUS_SUCCESS ||
      this->sw_status_ == SWUPATER_V2_STATUS_DONE)
    {
      std::this_thread::sleep_for(std::chrono::seconds(10));
    }
  return !this->upload_error_;
}

//-------------------------------------
// "Private" helpers
//-------------------------------------
bool
ifm3d::ImplV2::CheckRecovery()
{
  auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();
  c->Call(curl_easy_setopt, CURLOPT_URL, this->main_url_.c_str());
  c->Call(curl_easy_setopt, CURLOPT_NOBODY, true);
  c->Call(curl_easy_setopt,
          CURLOPT_CONNECTTIMEOUT,
          ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
  c->Call(curl_easy_setopt,
          CURLOPT_TIMEOUT,
          ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);

  long status_code;
  try
    {
      c->Call(curl_easy_perform);
      c->Call(curl_easy_getinfo, CURLINFO_RESPONSE_CODE, &status_code);
    }
  catch (const ifm3d::Error& e)
    {
      if (e.code() == IFM3D_RECOVERY_CONNECTION_ERROR ||
          e.code() == IFM3D_CURL_TIMEOUT)
        {
          return false;
        }
      throw;
    }

  return status_code == 200;
}

bool
ifm3d::ImplV2::CheckProductive()
{
  try
    {
      if (this->cam_->DeviceParameter("OperatingMode") != "")
        {
          return true;
        }
    }
  catch (const ifm3d::Error& e)
    {
      // Rethrow unless the code is one that indicates the camera is not
      // currently reachable by XML-RPC (occurs during the reboot process)
      if (e.code() != IFM3D_XMLRPC_TIMEOUT &&
          e.code() != IFM3D_XMLRPC_OBJ_NOT_FOUND)
        {
          throw;
        }
    }

  return false;
}

void
ifm3d::ImplV2::UploadFirmware(const std::string& swu_file, long timeout_millis)
{
  curl_global_init(CURL_GLOBAL_ALL);
  auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();

  mime_ctx mime_ctx;
  mime_ctx.fp = fopen_read(swu_file);

  if (!mime_ctx.fp)
    {
      throw ifm3d::Error(IFM3D_UPDATE_ERROR,
                         fmt::format("Unable to open file: {}", swu_file));
    }

  curl_mimepart* mimepart = c->AddMimePart();
  curl_mime_data_cb(mimepart,
                    (std::numeric_limits<int32_t>::max)(),
                    mime_read,
                    NULL,
                    mime_free,
                    &mime_ctx);
  curl_mime_name(mimepart, SWUPDATER_MIME_PART_NAME.c_str());
  curl_mime_type(mimepart, SWUPDATER_CONTENT_TYPE_HEADER.c_str());

  c->Call(curl_easy_setopt, CURLOPT_URL, this->upload_url_.c_str());
  c->Call(curl_easy_setopt, CURLOPT_TIMEOUT, SWUPDATE_V2_TIMEOUT_FOR_UPLOAD);
  c->Call(curl_easy_setopt, CURLOPT_TCP_KEEPALIVE, 1);
  c->Call(curl_easy_setopt, CURLOPT_MAXREDIRS, CURL_MAX_REDIR);
  c->Call(curl_easy_setopt,
          CURLOPT_CONNECTTIMEOUT,
          ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);

  // `curl_easy_perform` will return CURLE_ABORTED_BY_CALLBACK
  // due to the above workaround. Handle and squash that error.
  try
    {
      c->Call(curl_easy_perform);
    }
  catch (const ifm3d::Error& e)
    {
      if (e.code() != IFM3D_CURL_ABORTED)
        {
          throw;
        }
    }
}

void
ifm3d::ImplV2::OnWebSocketData(const std::string json_string)
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
            ((float)current_step / number_of_steps) *
            (percent_of_current_step / 100.0f);

          this->cb_(overall_percent, filename);
        }

      if (type == SWUPATER_V2_TYPE_MESSAGE)
        {
          const auto level = std::stoi(json["level"].get<std::string>());
          if (level == SWUPATER_V2_MESSAGE_LEVEL_ERROR)
            {
              // report error at this point
              this->upload_error_ = true;
              cv_.notify_all();
            }
          const auto message = json["text"].get<std::string>();
          this->cb_(1.0f, "Message : " + message);
        }

      if (type == SWUPATER_V2_TYPE_STATUS)
        {
          auto const status = json["status"].get<std::string>();
          if (status == SWUPATER_V2_STATUS_SUCCESS)
            {
              this->sw_status_ = status;
              cv_.notify_all();
            }
          if (status == SWUPATER_V2_STATUS_DONE)
            {
              this->sw_status_ = status;
              cv_.notify_all();
            }
          this->cb_(1.0f, "Status : " + status);
        }
    }
  catch (std::exception& e)
    {
      // ignore any status message parsing or json error
      // type info : shaowing json parser error
    }
}

#endif // IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H
