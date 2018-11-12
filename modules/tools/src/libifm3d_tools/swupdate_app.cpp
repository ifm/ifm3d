/*
* Copyright (C) 2018 Christian Ege
*
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

#include <ifm3d/tools/swupdate_app.h>
#include <cstdint>
#include <chrono>
#include <thread>
#include <iostream>
#include <istream>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <glog/logging.h>
#include <curl/curl.h>
#include <ifm3d/contrib/nlohmann/json.hpp>

#ifdef _WIN32
# include <io.h>
# include <fcntl.h>
#endif

static const std::string FWU_UPLOAD_URL = "/handle_post_request";
static const std::string FWU_REBOOT_URL = "/reboot_to_live";
static const std::string FWU_STATUS_URL = "/getstatus.json";
static const std::string FWU_CHECK_RECOVERY_URL = "/id.lp";
static const int FWU_RECOVERY_PORT = 8080;
static const std::string FWU_FILENAME_HEADER = "X_FILENAME: swupdate.swu";
static const std::string FWU_CONTENT_TYPE_HEADER =
  "Content-Type: application/octet-stream";

static const int FWU_STATUS_IDLE = 0;
static const int FWU_STATUS_START = 1;
static const int FWU_STATUS_RUN = 2;
static const int FWU_STATUS_SUCCESS = 3;
static const int FWU_STATUS_FAILURE = 4;

namespace po = boost::program_options;

ifm3d::SwupdateApp::SwupdateApp(int argc, const char **argv,
                                const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("file", po::value<std::string>()->default_value("-"),
     "Input file, defaults to `stdin' (good for reading off a pipeline)")
    ("check", po::bool_switch()->default_value(false),
     "check the current mode of device")
    ("r", po::bool_switch()->default_value(false),
     "reboot from recovery mode to productive mode");;

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

static std::string
createURL(std::string ip, int port, std::string path)
{
  return "http://" + ip + ":" + std::to_string(port) + path;
}

static size_t
SWupdateStatusWriteCallbackIgnore(char* ptr, size_t size, size_t nmemb,
                                  void* userdata)
{
  return size * nmemb;
}

static size_t
SWupdateStatusWriteCallback(char* ptr, size_t size, size_t nmemb,
                            void* userdata)
{
  std::string* body = static_cast<std::string*>(userdata);
  body->append(ptr, size * nmemb);
  return size * nmemb;
}

static size_t
SWupdateXferInfoCallback(void* clientp, size_t dltotal,
                         size_t dlnow, size_t uptotal, size_t upnow)
{
  if(uptotal > 0 && upnow >= uptotal)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

static size_t
SWupdateStatusReadCallback(void *dest, size_t size, size_t nmemb, void *userp)
{
  std::istream *is = (std::istream *)userp;

  if(is->good())
    {
      size_t max_read = size*nmemb;
      is->read(static_cast<char*>(dest), max_read);
      return is->gcount();
    }

  return 0;
}

int ifm3d::SwupdateApp::Run()
{
  if(this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto const file = vm_.count("file") ? true : false;
  auto const check = vm_.count("check") ? vm_["check"].as<bool>() : false;
  auto const recovery_reboot = vm_.count("r") ? vm_["r"].as<bool>() : false;

  if(check)
    {
      return checkRecoveryMode();
    }
  else if(recovery_reboot)
    {
      return rebootToProductiveMode();
    }
  else if(file)
    {
      checkRecovery();
      uploadFiletoDevice();
      std::cout << "Update successfull, rebooting device..." << std::endl;
      reboot();
    }
  return 0;
}

void ifm3d::SwupdateApp::checkRecovery()
{
  CURL* curl = curl_easy_init();
  CURLcode curlResult = CURLE_OK;

  curl_easy_setopt(curl, CURLOPT_URL,
                   createURL(this->cam_->IP(),
                             FWU_RECOVERY_PORT,
                             FWU_CHECK_RECOVERY_URL).c_str());

  curl_easy_setopt(curl, CURLOPT_NOBODY, true);

  curlResult = curl_easy_perform(curl);
  long statusCode;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &statusCode);

  if(curlResult != CURLE_OK || statusCode != 200)
    {
      throw ifm3d::error_t(IFM3D_RECOVERY_CONNECTION_ERROR);
    }
  else
    {
      std::cout << "Waiting for the SWUpdate process to become ready"
                << std::endl;
      waitForUpdateFinish(FWU_STATUS_IDLE);
    }
}

void
ifm3d::SwupdateApp::uploadData(std::shared_ptr<std::istream> data,
                               size_t filesize)
{
  CURL* curl = curl_easy_init();

  struct curl_slist* headerList = 0;
  headerList = curl_slist_append(headerList, FWU_CONTENT_TYPE_HEADER.c_str());
  headerList = curl_slist_append(headerList, FWU_FILENAME_HEADER.c_str());

  curl_easy_setopt(curl, CURLOPT_URL,
                   createURL(this->cam_->IP(),
                             FWU_RECOVERY_PORT, FWU_UPLOAD_URL).c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerList);
  curl_easy_setopt(curl, CURLOPT_READFUNCTION, SWupdateStatusReadCallback);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, filesize);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
                   SWupdateStatusWriteCallbackIgnore);
  curl_easy_setopt(curl, CURLOPT_READDATA, data.get());

  // Note: This is a workaround since the device will never actually close the
  //  connection
  curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, SWupdateXferInfoCallback);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0);

  CURLcode curlResult = curl_easy_perform(curl);

  curl_slist_free_all(headerList);
  curl_easy_cleanup(curl);

  // CURLE_ABORTED_BY_CALLBACK is the result of the workaround
  if(curlResult != CURLE_OK && curlResult != CURLE_ABORTED_BY_CALLBACK)
    {
      throw ifm3d::error_t(IFM3D_RECOVERY_CONNECTION_ERROR);
    }
}

void ifm3d::SwupdateApp::waitForUpdateFinish(int status)
{
  CURL* curl = curl_easy_init();

  std::string statusString;
  curl_easy_setopt(curl, CURLOPT_URL,
                   createURL(this->cam_->IP(),
                             FWU_RECOVERY_PORT, FWU_STATUS_URL).c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, SWupdateStatusWriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &statusString);

  bool updateFinished = false;
  while(!updateFinished)
    {
      statusString.clear();
      CURLcode curlResult = curl_easy_perform(curl);

      /* Check for connection error */
      if(curlResult != CURLE_OK)
        {
          throw ifm3d::error_t(IFM3D_RECOVERY_CONNECTION_ERROR);
        }

      /* Parse status */
      auto json = nlohmann::json::parse(statusString.c_str());
      int statusId = std::stoi(json["Status"].get<std::string>());
      int statusError = std::stoi(json["Error"].get<std::string>());
      std::string statusMessage = json["Msg"];

      /* Check for successful update */
      if(statusId == status)
        {
          updateFinished = true;
        }

      /* Check for update error */
      if(statusId == FWU_STATUS_FAILURE)
        {
          /* Don't stop on false positive */
          if(statusMessage != "ERROR parser/parse_config.c : parse_cfg")
            {
              std::cout << "Update failed with status: \""
                        << statusMessage << "\"" << std::endl;
              throw ifm3d::error_t(IFM3D_UPDATE_ERROR);
            }
        }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

  curl_easy_cleanup(curl);
}

void ifm3d::SwupdateApp::reboot()
{
  CURL* curl = curl_easy_init();

  curl_easy_setopt(curl, CURLOPT_URL,
                   createURL(this->cam_->IP(),
                             FWU_RECOVERY_PORT, FWU_REBOOT_URL).c_str());
  curl_easy_setopt(curl, CURLOPT_POST, true);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, 0);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
                   SWupdateStatusWriteCallbackIgnore);

  CURLcode curlResult = curl_easy_perform(curl);

  /* Check for connection error */
  if(curlResult != CURLE_OK)
    {
      throw ifm3d::error_t(IFM3D_IO_ERROR);
    }

  curl_easy_cleanup(curl);
}

int ifm3d::SwupdateApp::checkRecoveryMode()
{
  try
    {
      checkRecovery();
      // std::cout << "Device is in recovery mode" << std::endl;
      return 0;
    }
  catch(const ifm3d::error_t  &ex)
    {
      // checking for production mode if not able to connect to camera
      // it will throw connection error
      try
        {
          if(cam_->DeviceParameter("OperatingMode") != "")
            {
              std::cout << "Device is in Productive mode" << std::endl;
              return -1;
            }
          else if (ex.code() == IFM3D_RECOVERY_CONNECTION_ERROR)
            {
              std::cout << "Not able to connect to device" << std::endl;
              return -2;
            }
        }
      catch(...)
        {
          // if not able to connect to device
          std::cout << "Not able to connect to device" << std::endl;
          return -2;
        }
    }

  return 0;
}

int ifm3d::SwupdateApp::rebootToProductiveMode()
{
  try
    {
      checkRecovery();
      reboot();
      std::cout << "Device rebooted to Productive mode from Recovery mode "
                << std::endl;
      return 0;
    }
  catch(const ifm3d::error_t  &ex)
    {
      // checking for production mode if not able to connect to camera
      // it will throw connection error
      if(cam_->DeviceParameter("OperatingMode") != "")
        {
          std::cout << "Device is in Productive mode" << std::endl;
          return 0;
        }
      else if(ex.code() == IFM3D_RECOVERY_CONNECTION_ERROR)
        {
          std::cout << "Not able to connect to Device" << std::endl;
          return 0;
        }
    }

  return 0;
}

int ifm3d::SwupdateApp::uploadFiletoDevice()
{

  std::cout << "Loading firmware..." << std::endl;
  std::shared_ptr<std::istream> ifs;
  size_t file_size = 0;
  std::string infile = this->vm_["file"].as<std::string>();

  if(infile == "-")
    {
#ifdef _WIN32
      // on windows we need to reopen stdin in binary mode
      _setmode(_fileno(stdin), O_BINARY);
#endif

      std::size_t len;
      std::array<char, 1024> buf;

      std::stringstream* sstream = new std::stringstream;

      while((len = std::fread(buf.data(),
                              sizeof(buf[0]), buf.size(), stdin)) > 0)
        {
          if(std::ferror(stdin) && !std::feof(stdin))
            {
              std::cout << "Error reading from stdin" << std::endl;
              throw ifm3d::error_t(IFM3D_IO_ERROR);
            }

          sstream->write(buf.data(), len);

          file_size += len;
        }

      ifs.reset(sstream);
    }
  else
    {
      ifs.reset(new std::ifstream(infile, std::ios::in | std::ios::binary));
      if(!*ifs)
        {
          std::cout << "Could not open file: " << infile << std::endl;
          throw ifm3d::error_t(IFM3D_IO_ERROR);
        }

      ifs->unsetf(std::ios::skipws);
      ifs->seekg(0, std::ios::end);
      file_size = ifs->tellg();
      ifs->seekg(0, std::ios::beg);
    }

  std::cout << "Firmware loaded, uploading to device..." << std::endl;
  uploadData(ifs, file_size);

  std::cout << "Upload finished, waiting for the update process to finish..."
            << std::endl;
  waitForUpdateFinish(FWU_STATUS_SUCCESS);

  return 0;
}
