/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_EXPORT_HPP
#define IFM3D_DEVICE_EXPORT_HPP

#if defined(IFM3D_DEVICE_STATIC_LIB)
#  define IFM3D_DEVICE_EXPORT
#else
#  if defined(_MSC_VER)
#    ifdef IFM3D_DEVICE_DLL_BUILD
#      define IFM3D_DEVICE_EXPORT __declspec(dllexport)
#    else
#      define IFM3D_DEVICE_EXPORT __declspec(dllimport)
#    endif
#  else
#    define IFM3D_DEVICE_EXPORT __attribute__((visibility("default")))
#  endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_DEVICE_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#  define IFM3D_DEVICE_DEPRECATED __declspec(deprecated)
#else
#  define IFM3D_DEVICE_DEPRECATED
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_DEVICE_LOCAL __attribute__((visibility("hidden")))
#else
#  define IFM3D_DEVICE_LOCAL
#endif

/** \defgroup Device Device Module */

#endif /* IFM3D_DEVICE_EXPORT_HPP */
