// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_FILE_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_FILE_H

#include <iostream>
#include <fstream>
#include <cstdio>

#include <fmt/color.h>

#include <ifm3d/common/logging/log_writer.h>

namespace ifm3d
{
  template <class Formatter>
  class LogWriterFile : public LogWriter
  {
  public:
    LogWriterFile(const std::string& file_name,
                  size_t max_size = 0,
                  int keep_files = 0)
      : max_size_(max_size),
        keep_files_(keep_files)
    {
      this->SetFileName(file_name);
    }

    void
    SetFileName(const std::string& file_name)
    {
      const std::lock_guard<std::mutex> lock(this->mutex_);

      this->CloseFile();

      size_t idx_ext = file_name.find_last_of(".");
      if (idx_ext != std::string::npos)
        {
          this->file_stem_ = file_name.substr(0, idx_ext);
          this->file_ext_ = file_name.substr(idx_ext);
        }
      else
        {
          this->file_stem_ = file_name;
          this->file_ext_ = "";
        }
    }

    void
    SetKeepFiles(int keep_files)
    {
      this->keep_files_ = keep_files;
    }

    void
    SetMaxSize(size_t max_size)
    {
      this->max_size_ = max_size;
    }

    void
    Write(const LogEntry& entry) override
    {
      const auto str = Formatter::format(entry);

      const std::lock_guard<std::mutex> lock(this->mutex_);

      if (!this->file_.is_open())
        {
          this->OpenFile();
        }

      if (this->keep_files_ > 0 && this->max_size_ > 0 &&
          this->file_size_ > this->max_size_)
        {
          this->RotateFiles();
        }

      this->file_.write(str.c_str(), str.size());
      this->file_size_ += str.size();

#if defined(_MSC_VER)
      this->file_.write("\r\n", 2);
      this->file_size_ += 2;
#else
      this->file_.write("\n", 1);
      this->file_size_ += 1;
#endif
    }

  private:
    void
    OpenFile()
    {
      if (!file_.is_open())
        {
          auto file_name = this->GenerateFileName(0);
          this->file_.open(file_name, std::ios::binary | std::ios::app);
          if (this->file_)
            {
              this->file_.seekp(0, std::ios::beg);
              auto start = this->file_.tellp();
              this->file_.seekp(0, std::ios::end);
              this->file_size_ = this->file_.tellp() - start;
            }
          else
            {
              this->file_size_ = 0;
            }
        }
    }

    void
    CloseFile()
    {
      if (this - file_.is_open())
        {
          this->file_.close();
        }
    }

    void
    RotateFiles()
    {
      this->CloseFile();

      std::remove(this->GenerateFileName(this->keep_files_ - 1).c_str());

      for (int i = this->keep_files_ - 2; i >= 0; --i)
        {
          auto cur = this->GenerateFileName(i);
          auto next = this->GenerateFileName(i + 1);

          std::rename(cur.c_str(), next.c_str());
        }

      this->OpenFile();
    }

    std::string
    GenerateFileName(size_t number)
    {
      return number > 0 ?
               fmt::format("{}.{}{}",
                           this->file_stem_,
                           number,
                           this->file_ext_) :
               fmt::format("{}{}", this->file_stem_, this->file_ext_);
    }

  protected:
    std::mutex mutex_;
    std::string file_stem_;
    std::string file_ext_;
    std::ofstream file_;
    size_t file_size_;
    size_t max_size_;
    size_t keep_files_;
    bool firstWrite_;
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_FILE_H
