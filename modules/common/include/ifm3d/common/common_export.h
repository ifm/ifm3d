/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_COMMON_EXPORT_HPP
#define IFM3D_COMMON_EXPORT_HPP

#if defined(IFM3D_COMMON_STATIC_LIB)
#  define IFM3D_COMMON_EXPORT
#else
#  if defined(_MSC_VER)
#    ifdef IFM3D_COMMON_DLL_BUILD
#      define IFM3D_COMMON_EXPORT __declspec(dllexport)
#    else
#      define IFM3D_COMMON_EXPORT __declspec(dllimport)
#    endif
#  else
#    define IFM3D_COMMON_EXPORT __attribute__((visibility("default")))
#  endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_COMMON_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#  define IFM3D_COMMON_DEPRECATED __declspec(deprecated)
#else
#  define IFM3D_COMMON_DEPRECATED
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_COMMON_LOCAL __attribute__((visibility("hidden")))
#else
#  define IFM3D_COMMON_LOCAL
#endif

/** \defgroup Common Common Module */

#endif /* IFM3D_COMMON_EXPORT_HPP */
