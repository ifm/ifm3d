// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_H
#define IFM3D_TOOLS_H

#include <ifm3d/tools/app_types_app.h>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/tools/config_app.h>
#include <ifm3d/tools/cp_app.h>
#include <ifm3d/tools/discover_app.h>
#include <ifm3d/tools/dump_app.h>
#include <ifm3d/tools/export_app.h>
#include <ifm3d/tools/imager_types_app.h>
#include <ifm3d/tools/import_app.h>
#include <ifm3d/tools/jsonschema_app.h>
#include <ifm3d/tools/ls_app.h>
#include <ifm3d/tools/passwd_app.h>
#include <ifm3d/tools/make_app.h>
#include <ifm3d/tools/mutable_args.h>
#include <ifm3d/tools/reboot_app.h>
#include <ifm3d/tools/reset_app.h>
#include <ifm3d/tools/rm_app.h>
#include <ifm3d/tools/time_app.h>
#include <ifm3d/tools/trace_app.h>

#if defined(BUILD_MODULE_FRAMEGRABBER)
#  include <ifm3d/tools/fg/schema_app.h>
#  include <ifm3d/tools/fg/hz_app.h>
#  include <ifm3d/tools/fg/jitter_app.h>
#endif

#if defined(BUILD_MODULE_SWUPDATER)
#  include <ifm3d/tools/swupdater/swupdate_app.h>
#endif

#endif // IFM3D_TOOLS_H
