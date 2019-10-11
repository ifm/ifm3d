/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IFM3D_SWUPDATER_SWUPDATER_IMPL_H__
#define __IFM3D_SWUPDATER_SWUPDATER_IMPL_H__

#include <chrono>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <curl/curl.h>
#include <glog/logging.h>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/contrib/nlohmann/json.hpp>

namespace ifm3d
{
  const std::string SWUPDATER_UPLOAD_URL_SUFFIX = "/handle_post_request";
  const std::string SWUPDATER_REBOOT_URL_SUFFIX = "/reboot_to_live";
  const std::string SWUPDATER_STATUS_URL_SUFFIX = "/getstatus.json";
  const std::string SWUPDATER_CHECK_RECOVERY_URL_SUFFIX = "/id.lp";
  const std::string SWUPDATER_RECOVERY_PORT = "8080";
  const std::string SWUPDATER_FILENAME_HEADER = "X_FILENAME: swupdate.swu";
  const std::string SWUPDATER_CONTENT_TYPE_HEADER =
    "Content-Type: application/octet-stream";

  const int SWUPDATER_STATUS_IDLE = 0;
  const int SWUPDATER_STATUS_START = 1;
  const int SWUPDATER_STATUS_RUN = 2;
  const int SWUPDATER_STATUS_SUCCESS = 3;
  const int SWUPDATER_STATUS_FAILURE = 4;

  // Default timeout values for cURL transactions to the camera
  const long DEFAULT_CURL_CONNECT_TIMEOUT = 3; // seconds
  const long DEFAULT_CURL_TRANSACTION_TIMEOUT = 30; // seconds

  //============================================================
  // Impl interface
  //============================================================
  class SWUpdater::Impl
  {
  public:
    Impl(ifm3d::Camera::Ptr cam, const ifm3d::SWUpdater::FlashStatusCb& cb);
    ~Impl() = default;

    void RebootToRecovery();
    bool WaitForRecovery(long timeout_millis);
    void RebootToProductive();
    bool WaitForProductive(long timeout_millis);
    bool FlashFirmware(
      const std::vector<std::uint8_t>& bytes,
      long timeout_millis);

  private:
    ifm3d::Camera::Ptr cam_;
    ifm3d::SWUpdater::FlashStatusCb cb_;

    std::string upload_url_;
    std::string reboot_url_;
    std::string status_url_;
    std::string check_recovery_url_;

    bool
    CheckRecovery();

    bool
    CheckProductive();

    void
    UploadFirmware(const std::vector<std::uint8_t>& bytes,
                   long timeout_millis);

    bool
    WaitForUpdaterStatus(int desired_state, long timeout_millis);

    std::tuple<int, std::string, int>
    GetUpdaterStatus();

    /**
     * C-style static callbacks for libcurl
     */
    static size_t
    StatusWriteCallbackIgnore(char *ptr, size_t size, size_t nmemb,
                              void* userdata)
    {
      return size * nmemb;
    }

    static size_t
    StatusWriteCallback(char *ptr, size_t size, size_t nmemb,
                        void* userdata)
    {
      std::string* body = static_cast<std::string*>(userdata);
      body->append(ptr, size * nmemb);
      return size * nmemb;
    }

    static int
    XferInfoCallback(void* clientp, curl_off_t dltotal, curl_off_t dlnow,
                     curl_off_t ultotal, curl_off_t ulnow)
    {
      ifm3d::SWUpdater::Impl* swu =
        static_cast<ifm3d::SWUpdater::Impl*>(clientp);
      if (swu->cb_)
        {
          if (ultotal <= 0)
            {
              swu->cb_(0.0, "");
            }
          else
            {
              float percentage_complete =
                (static_cast<float>(ulnow)/static_cast<float>(ultotal));
              swu->cb_(percentage_complete, "");
            }
        }

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

    /**
     * RAII wrapper around libcurl's C API for performing a
     * transaction
     */
    class CURLTransaction
    {
    public:
      CURLTransaction()
      {
        this->header_list_ = nullptr;
        this->curl_ = curl_easy_init();
        if (!this->curl_)
          {
            throw ifm3d::error_t(IFM3D_CURL_ERROR);
          }
      }

      ~CURLTransaction()
      {
        curl_slist_free_all(this->header_list_);
        curl_easy_cleanup(this->curl_);
      }

      // disable copy/move semantics
      CURLTransaction(CURLTransaction&&) = delete;
      CURLTransaction& operator=(CURLTransaction&&) = delete;
      CURLTransaction(CURLTransaction&) = delete;
      CURLTransaction& operator=(const CURLTransaction&) = delete;

      /**
       * Wrapper for calling curl_easy_* APIs, and unified
       * error handling of return codes.
       */
      template<typename F, typename... Args>
      void Call(F f, Args... args)
      {
        CURLcode retcode = f(this->curl_, args...);
        if (retcode != CURLE_OK)
          {
            switch (retcode)
              {
              case CURLE_COULDNT_CONNECT:
                throw ifm3d::error_t(IFM3D_RECOVERY_CONNECTION_ERROR);
              case CURLE_OPERATION_TIMEDOUT:
                throw ifm3d::error_t(IFM3D_CURL_TIMEOUT);
              case CURLE_ABORTED_BY_CALLBACK:
                throw ifm3d::error_t(IFM3D_CURL_ABORTED);
              default:
                throw ifm3d::error_t(IFM3D_CURL_ERROR);
              }
          }
      }

      void AddHeader(const char* str)
      {
        this->header_list_ = curl_slist_append(this->header_list_, str);
        if (!this->header_list_)
          {
            throw ifm3d::error_t(IFM3D_CURL_ERROR);
          }
      }

      void SetHeader()
      {
        this->Call(curl_easy_setopt, CURLOPT_HTTPHEADER, this->header_list_);
      }

    private:
      CURL* curl_;
      struct curl_slist* header_list_;
    };

  }; // end: class SWUpdater::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor
//-------------------------------------
ifm3d::SWUpdater::Impl::Impl(ifm3d::Camera::Ptr cam,
                             const ifm3d::SWUpdater::FlashStatusCb& cb)
  : cam_(cam),
    cb_(cb),
    upload_url_("http://" + cam->IP() + ":" + SWUPDATER_RECOVERY_PORT +
                SWUPDATER_UPLOAD_URL_SUFFIX),
    reboot_url_("http://" + cam->IP() + ":" + SWUPDATER_RECOVERY_PORT +
                SWUPDATER_REBOOT_URL_SUFFIX),
    status_url_("http://" + cam->IP() + ":" + SWUPDATER_RECOVERY_PORT +
                SWUPDATER_STATUS_URL_SUFFIX),
    check_recovery_url_("http://" + cam->IP() + ":" + SWUPDATER_RECOVERY_PORT +
                        SWUPDATER_CHECK_RECOVERY_URL_SUFFIX)
{ }

//-------------------------------------
// "Public" interface
//-------------------------------------
void
ifm3d::SWUpdater::Impl::RebootToRecovery()
{
  this->cam_->Reboot(ifm3d::Camera::boot_mode::RECOVERY);
}

bool
ifm3d::SWUpdater::Impl::WaitForRecovery(long timeout_millis)
{
  if (timeout_millis < 0)
    {
      return this->CheckRecovery();
    }

  auto start = std::chrono::system_clock::now();
  while (!this->CheckRecovery())
    {
      if (timeout_millis > 0)
        {
          auto curr = std::chrono::system_clock::now();
          auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(curr-start);
          if (elapsed.count() > timeout_millis)
            {
              LOG(WARNING) << "Timed out waiting for recovery mode";
              return false;
            }
        }
    }
  return true;
}

void
ifm3d::SWUpdater::Impl::RebootToProductive()
{
  auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();
  c->Call(curl_easy_setopt, CURLOPT_URL, this->reboot_url_.c_str());
  c->Call(curl_easy_setopt, CURLOPT_POST, true);
  c->Call(curl_easy_setopt, CURLOPT_POSTFIELDSIZE, 0);
  c->Call(curl_easy_setopt, CURLOPT_WRITEFUNCTION,
          &ifm3d::SWUpdater::Impl::StatusWriteCallbackIgnore);
  c->Call(curl_easy_setopt, CURLOPT_CONNECTTIMEOUT,
          ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
  c->Call(curl_easy_setopt, CURLOPT_TIMEOUT,
          ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);
  c->Call(curl_easy_perform);
}

bool
ifm3d::SWUpdater::Impl::WaitForProductive(long timeout_millis)
{
  if (timeout_millis < 0)
    {
      return this->CheckProductive();
    }

  auto start = std::chrono::system_clock::now();
  while (!this->CheckProductive())
    {
      if (timeout_millis > 0)
        {
          auto curr = std::chrono::system_clock::now();
          auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(curr-start);
          if (elapsed.count() > timeout_millis)
            {
              // timeout
              LOG(WARNING) << "Timed out waiting for productive mode";
              return false;
            }
        }
    }
  return true;
}

bool
ifm3d::SWUpdater::Impl::FlashFirmware(
    const std::vector<std::uint8_t>& bytes,
    long timeout_millis)
{
  auto t_start = std::chrono::system_clock::now();
  long remaining_time = timeout_millis;
  auto get_remaining_time =
    [&t_start, timeout_millis]() -> long
    {
      auto t_now = std::chrono::system_clock::now();
      auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start);
      return timeout_millis - static_cast<long>(elapsed.count());
    };

  // Firmware updater must be in `idle` status prior to starting a firmware
  // upgrade. In some cases (firmware was flashed but the camera was not
  // rebooted, or the previous flash failed) the message queue must be
  // 'drained' so we query the status several times in quick succession.
  //
  // In practice, 10 iterations is sufficient.
  int retries = 0;
  while (!this->WaitForUpdaterStatus(SWUPDATER_STATUS_IDLE, -1))
    {
      if (++retries >= 10)
        {
          throw ifm3d::error_t(IFM3D_SWUPDATE_BAD_STATE);
        }
    }

  remaining_time = get_remaining_time();
  if (remaining_time <= 0)
    {
      return false;
    }

  this->UploadFirmware(bytes, remaining_time);

  remaining_time = get_remaining_time();
  if (remaining_time <= 0)
    {
      return false;
    }

  return this->WaitForUpdaterStatus(SWUPDATER_STATUS_SUCCESS, remaining_time);
}


//-------------------------------------
// "Private" helpers
//-------------------------------------
bool
ifm3d::SWUpdater::Impl::CheckRecovery()
{
  auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();
  c->Call(curl_easy_setopt, CURLOPT_URL, this->check_recovery_url_.c_str());
  c->Call(curl_easy_setopt, CURLOPT_NOBODY, true);
  c->Call(curl_easy_setopt, CURLOPT_CONNECTTIMEOUT,
          ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
  c->Call(curl_easy_setopt, CURLOPT_TIMEOUT,
          ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);

  long status_code;
  try
  {
    c->Call(curl_easy_perform);
    c->Call(curl_easy_getinfo, CURLINFO_RESPONSE_CODE, &status_code);
  }
  catch(const ifm3d::error_t& e)
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
ifm3d::SWUpdater::Impl::CheckProductive()
{
  try
    {
      if (this->cam_->DeviceParameter("OperatingMode") != "")
        {
          return true;
        }
     }
  catch (const ifm3d::error_t& e)
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
ifm3d::SWUpdater::Impl::UploadFirmware(
    const std::vector<std::uint8_t>& bytes,
    long timeout_millis)
{
  auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();
  c->AddHeader(SWUPDATER_CONTENT_TYPE_HEADER.c_str());
  c->AddHeader(SWUPDATER_FILENAME_HEADER.c_str());
  c->SetHeader();
  c->Call(curl_easy_setopt, CURLOPT_URL, this->upload_url_.c_str());
  c->Call(curl_easy_setopt, CURLOPT_POST, 1);
  c->Call(curl_easy_setopt, CURLOPT_POSTFIELDSIZE,
          static_cast<long>(bytes.size()));
  c->Call(curl_easy_setopt, CURLOPT_POSTFIELDS, bytes.data());
  c->Call(curl_easy_setopt, CURLOPT_WRITEFUNCTION,
          &ifm3d::SWUpdater::Impl::StatusWriteCallbackIgnore);
  c->Call(curl_easy_setopt, CURLOPT_CONNECTTIMEOUT,
          ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
  c->Call(curl_easy_setopt, CURLOPT_TIMEOUT_MS, timeout_millis);

  // Workaround -- device does not close the connection after the firmware has
  // been transferred. Register an xfer callback and terminate the transaction
  // after all the data has been sent.
  c->Call(curl_easy_setopt, CURLOPT_XFERINFOFUNCTION,
          &ifm3d::SWUpdater::Impl::XferInfoCallback);
  c->Call(curl_easy_setopt, CURLOPT_XFERINFODATA, this);
  c->Call(curl_easy_setopt, CURLOPT_NOPROGRESS, 0);

  // `curl_easy_perform` will return CURLE_ABORTED_BY_CALLBACK
  // due to the above workaround. Handle and squash that error.
  try
    {
      c->Call(curl_easy_perform);
    }
  catch(const ifm3d::error_t& e)
    {
      if (e.code() != IFM3D_CURL_ABORTED)
        {
          throw;
        }
    }
}

bool
ifm3d::SWUpdater::Impl::WaitForUpdaterStatus(int desired_status,
                                             long timeout_millis)
{
  int status_id;
  int status_error;
  std::string status_message;

  if (timeout_millis < 0)
    {
      std::tie(status_id, std::ignore, std::ignore) =
        this->GetUpdaterStatus();
      return status_id == desired_status;
    }

  auto start = std::chrono::system_clock::now();
  do
    {
      if (timeout_millis > 0)
        {
          auto curr = std::chrono::system_clock::now();
          auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(curr-start);
          if (elapsed.count() > timeout_millis)
            {
              // timeout
              LOG(WARNING) << "Timed out waiting for updater status: "
                           << desired_status;
              return false;
            }
         }

      std::tie(status_id, status_message, status_error) =
        this->GetUpdaterStatus();

      if (status_message != "")
        {
          if (this->cb_)
            {
              this->cb_(1.0, status_message);
            }
          LOG(INFO) << "[" << status_id << "][" << status_error << "]: "
                    << status_message;
        }

      if (status_id == SWUPDATER_STATUS_FAILURE)
        {
          // Work around a false positive condition
          if (status_message != "ERROR parser/parse_config.c : parse_cfg")
            {
              LOG(ERROR) << "SWUpdate failed with status: " << status_message;
              throw ifm3d::error_t(IFM3D_UPDATE_ERROR);
            }
        }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } while (status_id != desired_status);

  return true;
}

std::tuple<int, std::string, int>
ifm3d::SWUpdater::Impl::GetUpdaterStatus()
{
  std::string status_string;

  int status_id;
  std::string status_message;
  int status_error;

  auto c = std::make_unique<ifm3d::SWUpdater::Impl::CURLTransaction>();
  c->Call(curl_easy_setopt, CURLOPT_URL, this->status_url_.c_str());
  c->Call(curl_easy_setopt,
          CURLOPT_WRITEFUNCTION,
          &ifm3d::SWUpdater::Impl::StatusWriteCallback);
  c->Call(curl_easy_setopt, CURLOPT_WRITEDATA, &status_string);
  c->Call(curl_easy_setopt, CURLOPT_CONNECTTIMEOUT,
          ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
  c->Call(curl_easy_setopt, CURLOPT_TIMEOUT,
          ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);
  c->Call(curl_easy_perform);

  // Parse status
  auto json = nlohmann::json::parse(status_string.c_str());
  status_id = std::stoi(json["Status"].get<std::string>());
  status_error = std::stoi(json["Error"].get<std::string>());
  status_message = json["Msg"];

  return std::make_tuple(status_id, status_message, status_error);
}

#endif // __IFM3D_SWUPDATER_SWUPDATER_IMPL_H__

