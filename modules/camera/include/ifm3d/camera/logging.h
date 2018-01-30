// -*- c++ -*-
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

#ifndef __IFM3D_CAMERA_LOGGING_H__
#define __IFM3D_CAMERA_LOGGING_H__

// Annoying stuff for windows -- makes sure clients can import these functions
#ifndef IFM3D_DLL_DECL
# if defined(_WIN32) && !defined(__CYGWIN__)
#   define IFM3D_DLL_DECL  __declspec(dllimport)
# else
#   define IFM3D_DLL_DECL
# endif
#endif

extern IFM3D_DLL_DECL const int IFM3D_TRACE;
extern IFM3D_DLL_DECL const int IFM3D_TRACE_DEEP;
extern IFM3D_DLL_DECL const int IFM3D_PROTO_DEBUG;

#endif // __IFM3D_CAMERA_LOGGING_H__
