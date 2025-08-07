// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_WRITER_FILE_H
#define IFM3D_COMMON_LOGGING_LOG_WRITER_FILE_H

#include <cstdio>
#include <fstream>
#include <iostream>
#include <mutex>

#include <fmt/color.h>

#include <ifm3d/common/logging/log_writer.h>

namespace ifm3d
{
  template <class FORMATTER>
  class LogWriterFile : public LogWriter
  {
  public:
    LogWriterFile(const std::string& file_name,
                  size_t max_size = 0,
                  int keep_files = 0)
      : _max_size(max_size),
        _keep_files(keep_files)
    {
      this->SetFileName(file_name);
    }

    void
    SetFileName(const std::string& file_name)
    {
      const std::lock_guard<std::mutex> lock(this->_mutex);

      this->close_file();

      size_t idx_ext = file_name.find_last_of('.');
      if (idx_ext != std::string::npos)
        {
          this->_file_stem = file_name.substr(0, idx_ext);
          this->_file_ext = file_name.substr(idx_ext);
        }
      else
        {
          this->_file_stem = file_name;
          this->_file_ext = "";
        }
    }

    void
    SetKeepFiles(int keep_files)
    {
      this->_keep_files = keep_files;
    }

    void
    SetMaxSize(size_t max_size)
    {
      this->_max_size = max_size;
    }

    void
    Write(const LogEntry& entry) override
    {
      const auto str = FORMATTER::Format(entry);

      const std::lock_guard<std::mutex> lock(this->_mutex);

      if (!this->_file.is_open())
        {
          this->open_file();
        }

      if (this->_keep_files > 0 && this->_max_size > 0 &&
          this->_file_size > this->_max_size)
        {
          this->rotate_files();
        }

      this->_file.write(str.c_str(), str.size());
      this->_file_size += str.size();

#if defined(_MSC_VER)
      this->_file.write("\r\n", 2);
      this->_file_size += 2;
#else
      this->_file.write("\n", 1);
      this->_file_size += 1;
#endif
    }

  private:
    void
    open_file()
    {
      if (!_file.is_open())
        {
          auto file_name = this->generate_file_name(0);
          this->_file.open(file_name, std::ios::binary | std::ios::app);
          if (this->_file)
            {
              this->_file.seekp(0, std::ios::beg);
              auto start = this->_file.tellp();
              this->_file.seekp(0, std::ios::end);
              this->_file_size = this->_file.tellp() - start;
            }
          else
            {
              this->_file_size = 0;
            }
        }
    }

    void
    close_file()
    {
      if (this->_file.is_open())
        {
          this->_file.close();
        }
    }

    void
    rotate_files()
    {
      this->close_file();

      std::remove(this->generate_file_name(this->_keep_files - 1).c_str());

      for (int i = this->_keep_files - 2; i >= 0; --i)
        {
          auto cur = this->generate_file_name(i);
          auto next = this->generate_file_name(i + 1);

          std::rename(cur.c_str(), next.c_str());
        }

      this->open_file();
    }

    std::string
    generate_file_name(size_t number)
    {
      return number > 0 ?
               fmt::format("{}.{}{}",
                           this->_file_stem,
                           number,
                           this->_file_ext) :
               fmt::format("{}{}", this->_file_stem, this->_file_ext);
    }

  protected:
    std::mutex _mutex;
    std::string _file_stem;
    std::string _file_ext;
    std::ofstream _file;
    size_t _file_size{};
    size_t _max_size;
    size_t _keep_files;
    bool _first_write{};
  };
}
#endif // IFM3D_COMMON_LOGGING_LOG_WRITER_FILE_H
