/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H__
#define __IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H__

#include <chrono>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <glog/logging.h>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/contrib/nlohmann/json.hpp>
#include <swupdater_impl.hpp>
#include <mutex>
#include <condition_variable>

#include <iostream>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>

namespace ifm3d
{
  const std::string SWUPDATER_V2_UPLOAD_URL_SUFFIX = "/upload";
  const std::string SWUPDATER_V2_REBOOT_URL_SUFFIX = "/restart";
  const std::string SWUPDATER_V2_STATUS_URL_SUFFIX = "/ws";
  const std::string SWUPATER_V2_FILENAME = "swupdate.swu";

  // types
  const std::string SWUPATER_V2_TYPE_STEP = "step";
  const std::string SWUPATER_V2_TYPE_INFO = "info";
  const std::string SWUPATER_V2_TYPE_MESSAGE = "message";
  const std::string SWUPATER_V2_TYPE_STATUS = "status";
  const std::string SWUPATER_V2_TYPE_SOURCE = "source";

  // message level
  const int SWUPATER_V2_MESSAGE_LEVEL_ERROR = 3;
  const int SWUPATER_V2_MESSAGE_LEVEL_INFO = 6;

  // Status values
  const std::string SWUPATER_V2_STATUS_IDLE = "IDLE";
  const std::string SWUPATER_V2_STATUS_START = "START";
  const std::string SWUPATER_V2_STATUS_RUN = "RUN";
  const std::string SWUPATER_V2_STATUS_SUCCESS = "SUCCESS";
  const std::string SWUPATER_V2_STATUS_DONE = "DONE";

  // curl parameters
  const int SWUPDATE_V2_TIMEOUT_FOR_UPLOAD = 50; // sec
  const int CURL_MAX_REDIR = 50;
  //============================================================
  // Impl interface
  //============================================================
  class ImplV2 : public SWUpdater::Impl
  {
  public:
    ImplV2(ifm3d::Camera::Ptr cam,
           const ifm3d::SWUpdater::FlashStatusCb& cb,
           const std::string& swupdate_recovery_port);
    ~ImplV2() = default;

    virtual void RebootToRecovery();

    virtual void RebootToProductive();

    virtual bool FlashFirmware(const std::vector<std::uint8_t>& bytes,
                               long timeout_millis);

  private:
    virtual bool CheckRecovery();

    virtual bool CheckProductive();

    virtual void UploadFirmware(const std::vector<std::uint8_t>& bytes,
                                long timeout_millis);

    void OnWebSocketData(const std::string json_string);

     static int
    XferInfoCallback(void* clientp,
                     curl_off_t dltotal,
                     curl_off_t dlnow,
                     curl_off_t ultotal,
                     curl_off_t ulnow)
    {
      if (ultotal > 0 && ulnow >= ultotal)
        {
          // Signal to 'abort' the transfer once all the data has been
          // transferred. This is a workaround to 'complete' the curl
          // transaction because the camera does not terminate the connection.
          return 1;
        }
      else
        {
          return 0;
        }
    }

    class WebSocketEndpoint
    {
    public:
      WebSocketEndpoint()
      {
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

        m_endpoint.init_asio();
        m_endpoint.start_perpetual();

        m_thread.reset(
          new websocketpp::lib::thread(&client::run, &m_endpoint));
      }

      ~WebSocketEndpoint()
      {
        m_endpoint.stop_perpetual();
        websocketpp::lib::error_code ec;
        if (!m_hdl.expired())
          {
            m_endpoint.close(m_hdl,
                             websocketpp::close::status::going_away,
                             "",
                             ec);
            if (ec)
              {
                std::cout << "> Error closing connection " << std::endl;
              }
          }
        m_thread->join();
      }

      void
      close(websocketpp::close::status::value code)
      {
        websocketpp::lib::error_code ec;
        m_endpoint.close(m_hdl, code, "", ec);
        if (ec)
          {
            std::cout << "> Error initiating close: " << ec.message()
                      << std::endl;
          }
      }

      int
      connect(std::string const& uri)
      {
        websocketpp::lib::error_code ec;

        client::connection_ptr con = m_endpoint.get_connection(uri, ec);

        if (ec)
          {
            std::cout << "> Connect initialization error: " << ec.message()
                      << std::endl;
            return -1;
          }
        m_hdl = con->get_handle();

        con->set_open_handler(
          websocketpp::lib::bind(&WebSocketEndpoint::on_open,
                                 this,
                                 &m_endpoint,
                                 websocketpp::lib::placeholders::_1));
        con->set_fail_handler(
          websocketpp::lib::bind(&WebSocketEndpoint::on_fail,
                                 this,
                                 &m_endpoint,
                                 websocketpp::lib::placeholders::_1));
        con->set_message_handler(
          websocketpp::lib::bind(&WebSocketEndpoint::on_message,
                                 this,
                                 websocketpp::lib::placeholders::_1,
                                 websocketpp::lib::placeholders::_2));

        m_endpoint.connect(con);

        return 0;
      }

      void
      RegisterOnDataRecv(std::function<void(const std::string&)> on_data_recv)
      {
        cb_data_recv = on_data_recv;
      }

    private:
      void
      on_open(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        std::string server = con->get_response_header("Server");
      }

      void
      on_fail(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        std::string server = con->get_response_header("Server");
        std::string error_msg = con->get_ec().message();
      }
      void
      on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
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
      }
      void
      on_close(client* c, websocketpp::connection_hdl hdl)
      {
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        std::stringstream s;
        s << "close code: " << con->get_remote_close_code() << " ("
          << websocketpp::close::status::get_string(
               con->get_remote_close_code())
          << "), close reason: " << con->get_remote_close_reason();
      }

      client m_endpoint;
      websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
      websocketpp::connection_hdl m_hdl;
      std::function<void(const std::string&)> cb_data_recv;
    };

    std::unique_ptr<WebSocketEndpoint> websocket_;
    std::mutex m;
    std::condition_variable cv;
    std::string sw_status;
    bool upload_error;
  }; // end: class ImplV2

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor
//-------------------------------------
ifm3d::ImplV2::ImplV2(ifm3d::Camera::Ptr cam,
                      const ifm3d::SWUpdater::FlashStatusCb& cb,
                      const std::string& swupdate_recovery_port)
  : ifm3d::SWUpdater::Impl(cam, cb, swupdate_recovery_port),
    websocket_(std::make_unique<WebSocketEndpoint>()),
    sw_status(SWUPATER_V2_STATUS_IDLE),
    upload_error(false)
{

  upload_url_ = ("http://" + cam->IP() + ":" + swupdate_recovery_port +
                 SWUPDATER_V2_UPLOAD_URL_SUFFIX);
  reboot_url_ = ("http://" + cam->IP() + ":" + swupdate_recovery_port +
                 SWUPDATER_V2_REBOOT_URL_SUFFIX);
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
  return;
  // this->cam_->Reboot(ifm3d::Camera::boot_mode::RECOVERY);
}

void
ifm3d::ImplV2::RebootToProductive()
{
  return;
}

bool
ifm3d::ImplV2::FlashFirmware(const std::vector<std::uint8_t>& bytes,
                             long timeout_millis)
{
  auto t_start = std::chrono::system_clock::now();
  long remaining_time = timeout_millis;
  auto get_remaining_time = [&t_start, timeout_millis]() -> long {
    auto t_now = std::chrono::system_clock::now();
    auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start);
    return timeout_millis - static_cast<long>(elapsed.count());
  };

  websocket_->connect(status_url_);

  this->UploadFirmware(bytes, remaining_time);

  remaining_time = get_remaining_time();

  std::unique_lock<std::mutex> lk(m);
  cv.wait(lk, [&] {
    return (this->sw_status == SWUPATER_V2_STATUS_SUCCESS ||
           this->sw_status == SWUPATER_V2_STATUS_DONE) ||
           this->upload_error;
  });

 /* wait for sometime to restart*/
  std::this_thread::sleep_for(std::chrono::seconds(5));
  websocket_->close(websocketpp::close::status::going_away);

  return !this->upload_error;
}

//-------------------------------------
// "Private" helpers
//-------------------------------------
bool
ifm3d::ImplV2::CheckRecovery()
{
  return true;
}

bool
ifm3d::ImplV2::CheckProductive()
{
  return true;
}

void
ifm3d::ImplV2::UploadFirmware(const std::vector<std::uint8_t>& bytes,
                              long timeout_millis)
{
  curl_global_init(CURL_GLOBAL_ALL);
  struct curl_httppost* httppost = NULL;
  struct curl_httppost* last_post = NULL;

  curl_formadd(&httppost,
               &last_post,
               CURLFORM_COPYNAME,
               "upload",
               CURLFORM_BUFFER,
               ifm3d::SWUPATER_V2_FILENAME.c_str(),
               CURLFORM_BUFFERPTR,
               bytes.data(),
               CURLFORM_BUFFERLENGTH,
               bytes.size(),
               CURLFORM_END);

  auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();

  c->Call(curl_easy_setopt, CURLOPT_URL, this->upload_url_.c_str());
  c->Call(curl_easy_setopt, CURLOPT_HTTPPOST, httppost);
  c->Call(curl_easy_setopt, CURLOPT_TIMEOUT, SWUPDATE_V2_TIMEOUT_FOR_UPLOAD);
  c->Call(curl_easy_setopt, CURLOPT_TCP_KEEPALIVE, 1);
  c->Call(curl_easy_setopt, CURLOPT_MAXREDIRS, CURL_MAX_REDIR);
  c->Call(curl_easy_setopt,
          CURLOPT_CONNECTTIMEOUT,
          ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);

  // Workaround -- device does not close the connection after the firmware has
  // been transferred. Register an xfer callback and terminate the transaction
  // after all the data has been sent.
  c->Call(curl_easy_setopt,
          CURLOPT_XFERINFOFUNCTION,
          &ifm3d::ImplV2::XferInfoCallback);
  c->Call(curl_easy_setopt, CURLOPT_XFERINFODATA, this);
  c->Call(curl_easy_setopt, CURLOPT_NOPROGRESS, 0);

  // `curl_easy_perform` will return CURLE_ABORTED_BY_CALLBACK
  // due to the above workaround. Handle and squash that error.
  try
    {
      c->Call(curl_easy_perform);
      curl_formfree(httppost);
    }
  catch (const ifm3d::error_t& e)
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
  nlohmann::json json;
  try
    {
   json = nlohmann::json::parse(json_string.c_str());
  const auto type = json["type"].get<std::string>();

  if (type == SWUPATER_V2_TYPE_STEP)
    {
      const auto number_of_steps =
        std::stoi(json["number"].get<std::string>());
      const auto current_step = std::stoi(json["step"].get<std::string>());

      const auto percent_of_current_step =
        std::stoi(json["percent"].get<std::string>());

      const auto filename = json["name"].get<std::string>();

      const auto overall_percent = ((float)current_step / number_of_steps) *
                                   (percent_of_current_step / 100.0f);

      this->cb_(overall_percent, filename);
    }

  if (type == SWUPATER_V2_TYPE_MESSAGE)
    {
      const auto level = std::stoi(json["level"].get<std::string>());
      if (level == SWUPATER_V2_MESSAGE_LEVEL_ERROR)
        {
          // report error at this point
          this->upload_error = true;
          cv.notify_all();
        }
      const auto message = json["text"].get<std::string>();
      this->cb_(1.0f,"Message : " + message);
    }

  if (type == SWUPATER_V2_TYPE_STATUS)
  {
      auto const status = json["status"].get<std::string>();
      if (status == SWUPATER_V2_STATUS_SUCCESS)
      {
          this->sw_status = status;
          cv.notify_all();
      }
      if (status == SWUPATER_V2_STATUS_DONE)
        {
          this->sw_status = status;
          cv.notify_all();
        }
      this->cb_(1.0f, "Status : " + status);

  }
  }
  catch (std::exception &e)
    {
      // ignore any status message parsing or json error
      // type info : shaowing json parser error
    }
}

#endif // __IFM3D_SWUPDATER_SWUPDATER_V2_IMPL_H__
