/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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

#include <ifm3d/camera/logging.h>
#include <cstdlib>
#include <mutex>
#include <glog/logging.h>
#include <ifm3d/camera/version.h>

const int IFM3D_TRACE = 5;
const int IFM3D_TRACE_DEEP = 10;
const int IFM3D_PROTO_DEBUG = 15;

namespace ifm3d
{
  /**
   * Wrapper used to ensure google logging is initialized once
   */
  class Logging
  {
  private:
    /**
     * Flag indicating the initialization state of the logging subsystem.
     */
    static std::once_flag init_;

    /**
     * Runs the one-time initialization code
     */
    static void _Init(int verbose=0)
    {
      FLAGS_logbuflevel = -1;
      FLAGS_v = verbose;
      google::InitGoogleLogging(IFM3D_LIBRARY_NAME);
      google::SetStderrLogging(google::GLOG_FATAL);
    }

  public:
    /**
     * Function used to initialize the library's logging subsystem. This
     * function is thread safe and idempotent. However, it must be called at
     * least once prior to writing log messages.
     *
     * @param[in] verbose glog's VLOG verbosity level
     */
    static void Init(int verbose=0)
    {
      std::call_once(ifm3d::Logging::init_, ifm3d::Logging::_Init, verbose);
    }

  }; // end: class Logging

} // end: namespace ifm3d

std::once_flag ifm3d::Logging::init_;


// Initializer sample for MSVC and GCC/Clang.
// 2010-2016 Joe Lowe. Released into the public domain.
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
    #define INITIALIZER(f) \
        static void f(void); \
        struct f##_t_ { f##_t_(void) { f(); } }; static f##_t_ f##_; \
        static void f(void)
#elif defined(_MSC_VER)
    #pragma section(".CRT$XCU",read)
    #define INITIALIZER2_(f,p) \
        static void f(void); \
        __declspec(allocate(".CRT$XCU")) void (*f##_)(void) = f; \
        __pragma(comment(linker,"/include:" p #f "_")) \
        static void f(void)
    #ifdef _WIN64
        #define INITIALIZER(f) INITIALIZER2_(f,"")
    #else
        #define INITIALIZER(f) INITIALIZER2_(f,"_")
    #endif
#else
    #define INITIALIZER(f) \
        static void f(void) __attribute__((constructor)); \
        static void f(void)
#endif


INITIALIZER(libifm3d_camera_ctor)
{
  int vlog = std::getenv("IFM3D_VLOG") == nullptr ?
    0 : std::atoi(std::getenv("IFM3D_VLOG"));

  ifm3d::Logging::Init(vlog);
}
