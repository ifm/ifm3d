/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_SWUPDATER_SWUPDATER_IMPL_H
#define IFM3D_SWUPDATER_SWUPDATER_IMPL_H
#pragma once

#include <chrono>
#include <string>
#include <thread>
#include <tuple>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/common/json.hpp>
#include <ifm3d/device/util.h>
#include <ifm3d/swupdater/swupdater.h>
#include <httplib.h>

#ifdef _WIN32
#  include <io.h>
#  include <fcntl.h>
#endif

namespace ifm3d
{
  const std::string SWUPDATER_UPLOAD_URL_SUFFIX = "/handle_post_request";
  const std::string SWUPDATER_REBOOT_URL_SUFFIX = "/reboot_to_live";
  const std::string SWUPDATER_STATUS_URL_SUFFIX = "/getstatus.json";
  const std::string SWUPDATER_CHECK_RECOVERY_URL_SUFFIX = "/id.lp";
  const std::string SWUPDATER_FILENAME = "swupdate.swu";
  const std::string SWUPDATER_CONTENT_TYPE_HEADER =
    "Content-Type: application/octet-stream";
  const std::string SWUPDATER_MIME_PART_NAME = "upload";

  const int SWUPDATER_STATUS_IDLE = 0;
  const int SWUPDATER_STATUS_START = 1;
  const int SWUPDATER_STATUS_RUN = 2;
  const int SWUPDATER_STATUS_SUCCESS = 3;
  const int SWUPDATER_STATUS_FAILURE = 4;

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_EXPORT SWUpdater::Impl
  {
  public:
    Impl(ifm3d::Device::Ptr cam,
         const ifm3d::SWUpdater::FlashStatusCb& cb,
         const std::string& swupdate_recovery_port);
    virtual ~Impl() = default;

    virtual void RebootToRecovery();
    bool WaitForRecovery(long timeout_millis);
    virtual void RebootToProductive();
    bool WaitForProductive(long timeout_millis);
    virtual bool FlashFirmware(const std::string& swu_file,
                               long timeout_millis);

  protected:
    ifm3d::Device::Ptr cam_;
    ifm3d::SWUpdater::FlashStatusCb cb_;

    httplib::Client client_;

    virtual bool CheckRecovery();

    virtual bool CheckProductive();

    virtual void UploadFirmware(const std::string& swu_file,
                                long timeout_millis);

    virtual bool WaitForUpdaterStatus(int desired_state, long timeout_millis);

    virtual std::tuple<int, std::string, int> GetUpdaterStatus();

  }; // end: class SWUpdater::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor
//-------------------------------------
ifm3d::SWUpdater::Impl::Impl(ifm3d::Device::Ptr cam,
                             const ifm3d::SWUpdater::FlashStatusCb& cb,
                             const std::string& swupdate_recovery_port)
  : cam_(cam),
    cb_(cb),
    client_(cam->IP().c_str(), std::stoi(swupdate_recovery_port))
{
  this->client_.set_connection_timeout(ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
  this->client_.set_read_timeout(ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);
  this->client_.set_write_timeout(ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);
}

//-------------------------------------
// "Public" interface
//-------------------------------------
void
ifm3d::SWUpdater::Impl::RebootToRecovery()
{
  this->cam_->Reboot(ifm3d::Device::boot_mode::RECOVERY);
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
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            curr - start);
          if (elapsed.count() > timeout_millis)
            {
              LOG_WARNING("Timed out waiting for recovery mode");
              return false;
            }
        }
    }
  return true;
}

void
ifm3d::SWUpdater::Impl::RebootToProductive()
{
  auto res = this->client_.Post(SWUPDATER_REBOOT_URL_SUFFIX,
                                "",
                                "application/x-www-form-urlencoded");
  ifm3d::check_http_result(res);
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
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            curr - start);
          if (elapsed.count() > timeout_millis)
            {
              // timeout
              LOG_WARNING("Timed out waiting for productive mode");
              return false;
            }
        }
    }
  return true;
}
bool
ifm3d::SWUpdater::Impl::FlashFirmware(const std::string& swu_file,
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
          throw ifm3d::Error(IFM3D_SWUPDATE_BAD_STATE);
        }
    }

  remaining_time = get_remaining_time();
  if (remaining_time <= 0)
    {
      return false;
    }

  this->UploadFirmware(swu_file, remaining_time);

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
  auto res = this->client_.Head("/");

  if (!res)
    {
      if (res.error() == httplib::Error::Connection ||
          res.error() == httplib::Error::ConnectionTimeout)
        {
          return false;
        }

      LOG_WARNING("{}", res.error());
      throw ifm3d::Error(IFM3D_RECOVERY_CONNECTION_ERROR);
    }

  return res->status == 200;
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
  catch (const ifm3d::Error& e)
    {
      // Rethrow unless the code is one that indicates the camera is not
      // currently reachable by XML-RPC (occurs during the reboot process)
      if (e.code() != IFM3D_XMLRPC_TIMEOUT &&
          e.code() != IFM3D_XMLRPC_OBJ_NOT_FOUND)
        {
          // throw;
        }
    }

  return false;
}

struct mime_ctx
{
  FILE* fp;
};

size_t
mime_read(char* buffer, size_t size, size_t nitems, void* arg)
{
  auto ctx = (mime_ctx*)arg;
  return fread(buffer, size, nitems, ctx->fp);
}

void
mime_free(void* arg)
{
  auto ctx = (mime_ctx*)arg;
  if (ctx->fp)
    {
      if (ctx->fp != stdin)
        {
          fclose(ctx->fp);
        }
      ctx->fp = nullptr;
    }
}

FILE*
fopen_read(const std::string& file)
{
  if (file == "-")
    {
#ifdef _WIN32
      // on windows we need to reopen stdin in binary mode
      _setmode(_fileno(stdin), O_BINARY);
#endif
      return stdin;
    }

  FILE* res = fopen(file.c_str(), "rb");
  if (!res)
    {
      throw ifm3d::Error(IFM3D_IO_ERROR);
    }

  return res;
}

void
fclose_read(std::FILE* file)
{
  if (file != stdin)
    {
      fclose(file);
    }
}

void
ifm3d::SWUpdater::Impl::UploadFirmware(const std::string& swu_file,
                                       long timeout_millis)
{
  std::FILE* input = fopen_read(swu_file);

  char buffer[4096];
  httplib::MultipartFormDataProviderItems items = {
    {SWUPDATER_MIME_PART_NAME,
     [&](size_t offset, httplib::DataSink& sink) -> bool {
       auto c = fread(buffer, 1, sizeof(buffer), input);

       if (c > 0)
         {
           return sink.write(buffer, c);
         }

       if (feof(input))
         {
           sink.done();
         }

       return ferror(input) == 0;
     },
     SWUPDATER_FILENAME,
     SWUPDATER_CONTENT_TYPE_HEADER}};

  auto res = this->client_.Post(SWUPDATER_UPLOAD_URL_SUFFIX, {}, {}, items);

  fclose_read(input);
  // Ignore Error::Write because the device will force close the connection
  // after the update finishes
  if (res.error() != httplib::Error::Write)
    {
      ifm3d::check_http_result(res);
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
      std::tie(status_id, std::ignore, std::ignore) = this->GetUpdaterStatus();
      return status_id == desired_status;
    }

  auto start = std::chrono::system_clock::now();
  do
    {
      if (timeout_millis > 0)
        {
          auto curr = std::chrono::system_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            curr - start);
          if (elapsed.count() > timeout_millis)
            {
              // timeout
              LOG_WARNING("Timed out waiting for updater status: {}",
                          desired_status);
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
          LOG_INFO("[{}][{}]: {}", status_id, status_error, status_message);
        }

      if (status_id == SWUPDATER_STATUS_FAILURE)
        {
          // Work around a false positive condition
          if (status_message != "ERROR parser/parse_config.c : parse_cfg")
            {
              LOG_ERROR("SWUpdate failed with status: {}", status_message);
              throw ifm3d::Error(IFM3D_UPDATE_ERROR);
            }
        }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  } while (status_id != desired_status);

  return true;
}

std::tuple<int, std::string, int>
ifm3d::SWUpdater::Impl::GetUpdaterStatus()
{
  auto res = this->client_.Get(SWUPDATER_STATUS_URL_SUFFIX);

  ifm3d::check_http_result(res);

  // Parse status
  auto json = ifm3d::json::parse(res->body.c_str());
  auto status_id = std::stoi(json["Status"].get<std::string>());
  auto status_error = std::stoi(json["Error"].get<std::string>());
  auto status_message = json["Msg"].get<std::string>();

  return std::make_tuple(status_id, status_message, status_error);
}

#endif // IFM3D_SWUPDATER_SWUPDATER_IMPL_H
