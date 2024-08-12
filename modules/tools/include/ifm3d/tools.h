// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_H
#define IFM3D_TOOLS_H

#include <ifm3d/tools/legacy/app_types_app.h>
#include <ifm3d/tools/common/config_set_app.h>
#include <ifm3d/tools/legacy/cp_app.h>
#include <ifm3d/tools/ovp8xx/diagnostic_app.h>
#include <ifm3d/tools/common/discover_app.h>
#include <ifm3d/tools/common/dump_app.h>
#include <ifm3d/tools/legacy/export_app.h>
#include <ifm3d/tools/ovp8xx/get_diagnostic_app.h>
#include <ifm3d/tools/ovp8xx/get_filtered_app.h>
#include <ifm3d/tools/ovp8xx/get_filter_schema_app.h>
#include <ifm3d/tools/ovp8xx/get_init_app.h>
#include <ifm3d/tools/ovp8xx/get_service_report_app.h>
#include <ifm3d/tools/legacy/imager_types_app.h>
#include <ifm3d/tools/legacy/import_app.h>
#include <ifm3d/tools/ovp8xx/jsonschema_app.h>
#include <ifm3d/tools/legacy/ls_app.h>
#include <ifm3d/tools/legacy/passwd_app.h>
#include <ifm3d/tools/common/reboot_app.h>
#include <ifm3d/tools/ovp8xx/remove_app.h>
#include <ifm3d/tools/common/reset_app.h>
#include <ifm3d/tools/ovp8xx/reset_ovp8xx_app.h>
#include <ifm3d/tools/legacy/rm_app.h>
#include <ifm3d/tools/ovp8xx/save_init_app.h>
#include <ifm3d/tools/legacy/time_app.h>
#include <ifm3d/tools/common/trace_app.h>
#include <ifm3d/tools/legacy/o3d3xx_app.h>
#include <ifm3d/tools/legacy/o3x_app.h>
#include <ifm3d/tools/ovp8xx/ovp8xx_app.h>
#include <ifm3d/tools/ovp8xx/config_ovp8xx_app.h>

#if defined(BUILD_MODULE_FRAMEGRABBER)
#  include <ifm3d/tools/common/fg/hz_app.h>
#  include <ifm3d/tools/common/fg/jitter_app.h>
#  include <ifm3d/tools/common/fg/stat_app.h>
#endif

#if defined(BUILD_MODULE_SWUPDATER)
#  include <ifm3d/tools/common/swupdater/swupdate_app.h>
#  include <ifm3d/tools/common/swupdater/swupdate_deprecated_app.h>
#  include <ifm3d/tools/common/swupdater/flash_sw_app.h>
#  include <ifm3d/tools/common/swupdater/restart_app.h>
#endif

#endif // IFM3D_TOOLS_H
