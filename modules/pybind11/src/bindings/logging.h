/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_LOGGING
#define IFM3D_PYBIND_BINDING_LOGGING

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>

#include <fmt/format.h>
#include <fmt/chrono.h>

#include <ifm3d/common/logging/logging.h>

class PyLogWriter : public ifm3d::LogWriter
{
public:
  using LogWriter::LogWriter;

  void
  Write(const ifm3d::LogEntry& entry) override
  {
    PYBIND11_OVERRIDE_PURE_NAME(void, ifm3d::LogWriter, "write", Write, entry);
  };
};

void
bind_logging(pybind11::module_& m)
{
  // clang-format off

  py::enum_<ifm3d::LogLevel>(m, "LogLevel", "Enum: The log level.")
    .value("None", ifm3d::LogLevel::None)
    .value("Critical", ifm3d::LogLevel::Critical)
    .value("Error", ifm3d::LogLevel::Error)
    .value("Warning", ifm3d::LogLevel::Warning)
    .value("Info", ifm3d::LogLevel::Info)
    .value("Debug", ifm3d::LogLevel::Debug)
    .value("Verbose", ifm3d::LogLevel::Verbose); 

  py::class_<ifm3d::LogEntry> logEntry(
    m, 
    "LogEntry", 
    R"(
      Represent a single log entry.
    )"
  );

  logEntry.def_property_readonly(
    "file",
    &ifm3d::LogEntry::GetFile,
    R"(
      The name of the file from which the log entry originates
    )");

  logEntry.def_property_readonly(
    "file",
    &ifm3d::LogEntry::GetFunc,
    R"(
      The name of the function from which this log entry originates
    )");

  logEntry.def_property_readonly(
    "file",
    &ifm3d::LogEntry::GetLine,
    R"(
      The line number from which this log entry originates
    )");

  logEntry.def_property_readonly(
    "file",
    &ifm3d::LogEntry::GetLogLevel,
    R"(
      The log level of this log entry
    )");

  logEntry.def_property_readonly(
    "file",
    &ifm3d::LogEntry::GetMessage,
    R"(
      The message of this log entry
    )");

  logEntry.def_property_readonly(
    "file",
    &ifm3d::LogEntry::GetTime,
    R"(
      The time this log entry occured
    )");

  py::class_<ifm3d::LogWriter, PyLogWriter, std::shared_ptr<ifm3d::LogWriter>> logWriter(
    m, 
    "LogWriter",
    R"(
      Base class for creating custom log writers.
    )");

  logWriter.def(py::init<>());

  logWriter.def(
    "write",
    &ifm3d::LogWriter::Write,
    py::arg("entry"),
    R"(
      Called when the given entry should be written
    )");

  py::class_<ifm3d::LogFormatterText> logFormatterText(
    m, 
    "LogFormatterText", 
    R"(
      Formats a give LogEntry as a human readable text line
    )"
  );

  logFormatterText.def_static(
    "format",
    &ifm3d::LogFormatterText::format,
    py::arg("entry"),
    R"(
       Format the LogEntry as a human readable text line
    )");

  py::class_<ifm3d::LogFormatterJson> logFormatterJson(
    m, 
    "LogFormatterJson", 
    R"(
      Formats a give LogEntry as a json object
    )"
  );

  logFormatterJson.def_static(
    "format",
    &ifm3d::LogFormatterJson::format,
    py::arg("entry"),
    R"(
       Format the LogEntry as a json object
    )");

  py::class_<ifm3d::Logger> logger(
    m, 
    "Logger", 
    R"(
      Provides access for configuring the logging facilities of ifm3d
    )"
  );

  logger.def_static(
    "set_log_level",
    [](ifm3d::LogLevel level){ ifm3d::Logger::Get().SetLogLevel(level); },
    R"(
      Set the active log level, messages below this level will be discarded
    )");

  logger.def_static(
    "log_level",
    [](){ ifm3d::Logger::Get().GetLogLevel(); },
    R"(
      Get the active log level, messages below this level will be discarded
    )");

  logger.def_static(
    "set_writer",
    [](std::shared_ptr<ifm3d::LogWriter> writer) { ifm3d::Logger::Get().SetWriter(writer); },
    R"(
      Set the log writer.
    )");

  // clang-format on
}
#endif // IFM3D_PYBIND_BINDING_LOGGING