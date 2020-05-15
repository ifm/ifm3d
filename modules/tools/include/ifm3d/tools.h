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

#ifndef __IFM3D_TOOLS_H__
#define __IFM3D_TOOLS_H__

#include <ifm3d/tools/app_types_app.h>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/tools/config_app.h>
#include <ifm3d/tools/cp_app.h>
#include <ifm3d/tools/dump_app.h>
#include <ifm3d/tools/export_app.h>
#include <ifm3d/tools/imager_types_app.h>
#include <ifm3d/tools/import_app.h>
#include <ifm3d/tools/ls_app.h>
#include <ifm3d/tools/passwd_app.h>
#include <ifm3d/tools/make_app.h>
#include <ifm3d/tools/reboot_app.h>
#include <ifm3d/tools/reset_app.h>
#include <ifm3d/tools/rm_app.h>
#include <ifm3d/tools/time_app.h>
#include <ifm3d/tools/trace_app.h>

#if defined(BUILD_MODULE_FRAMEGRABBER)
#include <ifm3d/tools/fg/schema_app.h>
#include <ifm3d/tools/fg/hz_app.h>
#include <ifm3d/tools/fg/jitter_app.h>
#endif

#if defined(BUILD_MODULE_SWUPDATER)
#include <ifm3d/tools/swupdater/swupdate_app.h>
#endif

#endif // __IFM3D_TOOLS_H__
