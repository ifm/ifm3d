# ifm3d::Logger - Logger for ifm3d library 
```ifm3d::Logger``` The Logger Library is a powerful logging utility designed to facilitate effective logging within ifm3d library and application.
It allows user to control log levels and define custom log writer (sink) for handling log messages.

## Log Levels
Log Levels
The ```ifm3d::Logger``` Library supports the following log levels:

```C++
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
```
* None: No logging is done 
* Critical: Indicates a critical error that may cause the application to terminate.
* Error: Indicates an error that caused the application to fail to perform a specific function or operation.
* Warning: Indicates potential issues that do not prevent the application from functioning.
* Info: General information about the application's operation.
* Debug: Detailed information for debugging purposes.
* Verbose : Logs with internal values for debugging purpose

By default, the log level is set to Warning, which means all log messages of Warning, Error,Critical will be recorded.
However, user can configure the log level to control which messages are logged.

## Accessing the Logger Instance

```ifm3d::Logger``` class follows the Singleton design pattern, which ensures that only one instance of the logger exists throughout the library.
The ```ifm3d::Logger::Get``` method is responsible for creating the instance and return the refernce of the instance.

```CPP
auto& logger = ifm3d::Logger::Get();
```

## Setting the Log Level

User can set the log level using the SetLogLevel method in Logger class.
```CPP
auto& logger = ifm3d::Logging::Get();
logger.SetLogLevel(ifm3d::LogLevel::Verbose)
```
By setting the log level, only log messages with an equal or lower level will be recorded. For example, if user set the log level to info, messages with levels Debug,Verbose will be ignored.

## Logging inside the ifm3d library
ifm3d use following marcos to do logging inside the library.

```
LOG_DEBUG("This is a debug message.")
LOG_INFO("This is an info message.")
LOG_WARNING("This is a warning message.")
LOG_ERROR("This is an error message.")
LOG_CRITICAL("This is a critical message.")
LOG_VERBOSE("This is a verbose message.")

```
logs will be printed according to the log level configured.

## Using Logger in user application

```ifm3d::Logger``` can be used in the application which uses ifm3d. ```ifm3d::Logger``` use [fmt](https://github.com/fmtlib/fmt) library fpr formatting lohg entries.
user must link the application with the fmt library to use the ```ifm3d::Logger```. Logging in application  can be done by including header 
```"ifm3d/common/logging/log.h"``` and logging can be done with macros provided by ```ifm3d::Logger```

```CPP

#include <iostream>
#include <chrono>
#include <thread>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/common/logging/log.h>

using namespace ifm3d::literals;

int
main()
{
  //Get the logging Instance 
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
  //Set Schema and start the grabber
  fg->Start({ ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,ifm3d::buffer_id::XYZ,ifm3d::buffer_id::CONFIDENCE_IMAGE });
  //////////////////////////
  // use framegrabber in streaming mode 
  //////////////////////////
  fg->OnNewFrame([&](ifm3d::Frame::Ptr frame)
    {
      auto distance_image = frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);
      LOG_VERBOSE(distance_image.width();
    });

  std::this_thread::sleep_for(std::chrono::seconds(10));
  fg->Stop();

  return 0;
}
```

## Log Writer (sink)
A log Writer is responsible for handling log messages. The ```ifm3d::Logger``` Library provides a default log writer that writes log messages to the console. 
However, you can define your own custom log sinks to redirect log messages to different destinations, such as files, databases, or to third party loggers.

### Defining a Custom Log Writer

```ifm3d::Logger``` provides a abstract interface ```ifm3d::LogWriter``` which custom log writter can implement to work with ```ifm3d::Logger```.
 ```ifm3d::Logger``` provide interface ```SetLogWriter(std::shared_ptr<ifm3d::LogWriter>)```. which is used to set the customLogger with ```ifm3d::Logger```.
 
 Following example shows a custom logger implementation with a third-party library SPDLOG.
 
 ```Cpp
 class LogWriterSpdLog : public LogWriter
  {
  public:
    LogWriterSpdLog() {}

    void
    Write(const LogEntry& entry) override
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
        entry.GetMessage());
    }
  }
 ```
