// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_LOGGING_LOG_H
#define IFM3D_COMMON_LOGGING_LOG_H

#include <cstring>

#include <ifm3d/common/logging/log_entry.h>
#include <ifm3d/common/logging/logger.h>
#include <ifm3d/common/logging/log_level.h>

#include <fmt/format.h>

#ifdef _MSC_VER
#  define LOG_GET_FUNC() __FUNCTION__
#else
#  define LOG_GET_FUNC() __PRETTY_FUNCTION__
#endif

#ifndef IFM3D_COMMON_LOGGING_STRIP_PREFIX
#  define IFM3D_COMMON_LOGGING_STRIP_PREFIX ""
#endif

namespace ifm3d
{
  constexpr const char*
  LOG_STRIP_PREFIX(const char* ptr, const char* prefix)
  {
    return ::std::strncmp(ptr, prefix, ::std::strlen(prefix)) == 0 ?
             (ptr + ::std::strlen(prefix)) :
             ptr;
  }
}

#define LOG_GET_FILE()                                                        \
  ::ifm3d::LOG_STRIP_PREFIX(__FILE__, IFM3D_COMMON_LOGGING_STRIP_PREFIX)
#define LOG_GET_LINE() __LINE__

#define LOG_IF_(condition)                                                    \
  if (!(condition))                                                           \
    {                                                                         \
      ;                                                                       \
    }                                                                         \
  else

#define LOG(log_level, msg, ...)                                              \
  LOG_IF_(::ifm3d::Logger::Get().ShouldLog(log_level))                        \
  {                                                                           \
    ::ifm3d::Logger::Get().Write(                                             \
      ::ifm3d::LogEntry(::fmt::format(msg, ##__VA_ARGS__),                    \
                        log_level,                                            \
                        LOG_GET_FILE(),                                       \
                        LOG_GET_FUNC(),                                       \
                        LOG_GET_LINE()));                                     \
  }

#define LOG_VERBOSE(fmt, ...)                                                 \
  LOG(::ifm3d::LogLevel::Verbose, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) LOG(::ifm3d::LogLevel::Debug, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) LOG(::ifm3d::LogLevel::Info, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...)                                                 \
  LOG(::ifm3d::LogLevel::Warning, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) LOG(::ifm3d::LogLevel::Error, fmt, ##__VA_ARGS__)
#define LOG_CRITICAL(fmt, ...)                                                \
  LOG(::ifm3d::LogLevel::Critical, fmt, ##__VA_ARGS__)

#endif // IFM3D_COMMON_LOGGING_LOG_H