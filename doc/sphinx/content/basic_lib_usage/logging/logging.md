# ifm3d::Logger - Logger for the ifm3d library

The `ifm3d::Logger` is a powerful logging utility designed to facilitate effective logging within the ifm3d library and applications. It allows users to control log levels and define custom log writers (sinks) for handling log messages.

## Log Levels

The `ifm3d::Logger` library supports the following log levels:
:::::{tabs}
::::{group-tab} Python
:::python
class LogLevel():
    None
    Critical
    Error
    Info
    Warning
    Debug
    Verbose
:::
::::
::::{group-tab} C++
:::cpp
enum class LogLevel
{
  None = 0,
  Critical = 1,
  Error = 2,
  Warning = 3,
  Info = 4,
  Debug = 5,
  Verbose = 6
};
:::
::::
:::::

- None: No logging is done.
- Critical: Indicates a critical error that may cause the application to terminate.
- Error: Indicates an error that caused the application to fail to perform a specific function or operation.
- Warning: Indicates potential issues that do not prevent the application from functioning.
- Info: General information about the application's operation.
- Debug: Detailed information for debugging purposes.
- Verbose: Logs with internal values for debugging purposes.

By default, the log level is set to Warning, which means all log messages of Warning, Error, and Critical will be recorded. However, users can configure the log level to control which messages 
are logged.

:::::{tabs}
::::{group-tab} Python
:::python
from ifm3dpy.logging import Logger, LogLevel, LogWriter, LogFormatterText, LogEntry

# Define a custom LogWriter
class MyLogWriter(LogWriter):
    def write(self, entry: LogEntry) -> None:
        print("Logging from python: ", LogFormatterText.format(entry))

# Create the MyLogWriter instance
MY_LOGGER = MyLogWriter()

if __name__ == "__main__":
  # Set a log level
  Logger.set_writer(LogLevel.Error)

  # Assign the LogWriter instance
  Logger.set_writer(MY_LOGGER)

  # Do something that causes a log message to happen
  o3r = ifm3dpy.O3R()
  o3r.port("non-existing-port")

:::
::::
::::{group-tab} C++
Instantiating these objects is as follows:

## Accessing the Logger Instance

The `ifm3d::Logger` class follows the Singleton design pattern, which ensures that only one instance of the logger exists throughout the library. The `ifm3d::Logger::Get` method is responsible for creating the instance and returning a reference to the instance.


```CPP
auto& logger = ifm3d::Logger::Get();
```

## Setting the Log Level

Users can set the log level using the `SetLogLevel` method in the `Logger` class.

```CPP
auto& logger = ifm3d::Logger::Get();
logger.SetLogLevel(ifm3d::LogLevel::Verbose);
```

By setting the log level, only log messages with an equal or lower level will be recorded. For example, if the log level is set to Info, messages with levels Debug and Verbose will be ignored.

## Logging inside the ifm3d library

The ifm3d library uses the following macros for logging inside the library:

```cpp
LOG_DEBUG("This is a debug message.")
LOG_INFO("This is an info message.")
LOG_WARNING("This is a warning message.")
LOG_ERROR("This is an error message.")
LOG_CRITICAL("This is a critical message.")
LOG_VERBOSE("This is a verbose message.")
```

Logs will be printed according to the configured log level.

## Using the Logger in User Applications

The `ifm3d::Logger` can be used in applications that use ifm3d. The `ifm3d::Logger` uses the [fmt](https://github.com/fmtlib/fmt) library for formatting log entries. Users must link their applications with the fmt library to use the `ifm3d::Logger`. Logging in an application can be done by including the header `"ifm3d/common/logging/log.h"`, and logging can be performed using the macros provided by `ifm3d::Logger`.

```CPP
#include <iostream>
#include <chrono>
#include <thread>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/common/logging/log.h>

using namespace ifm3d::literals;

int main()
{
  // Get the logging instance
  auto& logger = ifm3d::Logger::Get();
  // Set the log level
  logger.SetLogLevel(ifm3d::LogLevel::Verbose);

  // Declare the device object (one object only, corresponding to the VPU)
  auto dev = std::make_shared<ifm3d::O3R>();
  LOG_INFO("Device creation done")

  // Declare the FrameGrabber
  // One FrameGrabber per camera head (define the port number).
  const auto FG_PCIC_PORT =
    dev->Get()["/ports/port2/data/pcicTCPPort"_json_pointer];
  auto fg = std::make_shared<ifm3d::FrameGrabber>(dev, FG_PCIC_PORT);

  LOG_DEBUG("Setting Schema")

  // Set Schema and start the grabber
  fg->Start({ ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE, ifm3d::buffer_id::XYZ, ifm3d::buffer_id::CONFIDENCE_IMAGE });

  // Use the framegrabber in streaming mode
  fg->OnNewFrame([&](ifm3d::Frame::Ptr frame)
  {
    auto distance_image = frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);
    LOG_VERBOSE("Width: {}, Height: {}", distance_image.width(), distance_image.height());
  });

  std::this_thread::sleep_for(std::chrono::seconds(10));
  fg->Stop();

  return 0;
}
```

## Log Writer (Sink)

A log writer is responsible for handling log messages. The `ifm3d::Logger` library provides a default log writer that writes log messages to the console. However, you can define your own custom log sinks to redirect log messages to different destinations, such as files, databases, or third-party loggers.

### Defining a Custom Log Writer

The `ifm3d::Logger` provides an abstract interface, `ifm3d::LogWriter`, which custom log writers can implement to work with `ifm3d::Logger`. The `ifm3d::Logger` provides the `SetLogWriter` method, which is used to set the custom logger with `ifm3d::Logger`.

The following example shows a custom logger implementation with the third-party library SPDLOG.

```cpp
class LogWriterSpdLog : public LogWriter
{
public:
  LogWriterSpdLog() {}

  void Write(const LogEntry& entry) override
  {
    spdlog::level::level_enum spdlog_level;
    switch (entry.GetLogLevel())
    {
      case LogLevel::Critical:
        spdlog_level = spdlog::level::critical;
        break;
      case LogLevel::Error:
        spdlog_level = spdlog::level::err;
        break;
      case LogLevel::Warning:
        spdlog_level = spdlog::level::warn;
        break;
      case LogLevel::Info:
        spdlog_level = spdlog::level::info;
        break;
      case LogLevel::Debug:
        spdlog_level = spdlog::level::debug;
        break;
      case LogLevel::Verbose:
        spdlog_level = spdlog::level::trace;
        break;
      default:
        spdlog_level = spdlog::level::off;
    }

    spdlog::log(
      spdlog::source_loc(entry.GetFile(), entry.GetLine(), entry.GetFunc()),
      spdlog_level,
      entry.GetMessage()
    );
  }
};
```
::::
:::::

