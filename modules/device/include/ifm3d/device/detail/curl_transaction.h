// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_CURL_TRANSACTION_H
#define IFM3D_DEVICE_CURL_TRANSACTION_H

#include <string>
#include <vector>
#include <fmt/format.h>
#include <curl/curl.h>
#include <ifm3d/device/err.h>
#include <ifm3d/device/module_device.h>

namespace ifm3d
{
  class CURLTransaction
  {
  public:
    CURLTransaction()
    {
      this->header_list_ = nullptr;
      this->mime_ = nullptr;
      this->curl_ = curl_easy_init();
      if (!this->curl_)
        {
          throw ifm3d::Error(IFM3D_CURL_ERROR);
        }
    }

    ~CURLTransaction()
    {
      if (mime_ != nullptr)
        {
          curl_mime_free(mime_);
        }
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
    template <typename F, typename... Args>
    void
    Call(F f, Args... args)
    {
      if ((void*)f == (void*)curl_easy_perform)
        {
          if (mime_ != nullptr)
            {
              Call(curl_easy_setopt, CURLOPT_MIMEPOST, mime_);
            }
        }

      CURLcode retcode = f(this->curl_, args...);
      if (retcode != CURLE_OK)
        {
          switch (retcode)
            {
            case CURLE_COULDNT_CONNECT:
              throw ifm3d::Error(IFM3D_RECOVERY_CONNECTION_ERROR);
            case CURLE_OPERATION_TIMEDOUT:
              throw ifm3d::Error(IFM3D_CURL_TIMEOUT);
            case CURLE_ABORTED_BY_CALLBACK:
              throw ifm3d::Error(IFM3D_CURL_ABORTED);
            default:
              throw ifm3d::Error(IFM3D_CURL_ERROR,
                                 curl_easy_strerror(retcode));
            }
        }
    }

    void
    AddHeader(const char* str)
    {
      this->header_list_ = curl_slist_append(this->header_list_, str);
      if (!this->header_list_)
        {
          throw ifm3d::Error(IFM3D_CURL_ERROR);
        }
    }

    void
    SetHeader()
    {
      this->Call(curl_easy_setopt, CURLOPT_HTTPHEADER, this->header_list_);
    }

    curl_mimepart*
    AddMimePart()
    {
      if (mime_ == nullptr)
        {
          mime_ = curl_mime_init(curl_);
        }

      return curl_mime_addpart(mime_);
    }

  private:
    CURL* curl_;
    curl_mime* mime_;
    struct curl_slist* header_list_;
  }; // end: class CURLTransaction

} // end: namespace ifm3d

#endif // IFM3D_DEVICE_CURL_TRANSACTION_H
