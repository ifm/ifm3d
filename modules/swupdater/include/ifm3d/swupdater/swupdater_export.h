#ifndef IFM3D_SWUPDATER_EXPORT_HPP
#define IFM3D_SWUPDATER_EXPORT_HPP

#if defined(IFM3D_SWUPDATER_STATIC_LIB)
#  define IFM3D_SWUPDATER_EXPORT
#else
#  if defined(_MSC_VER)
#    ifdef IFM3D_SWUPDATER_DLL_BUILD
#      define IFM3D_SWUPDATER_EXPORT __declspec(dllexport)
#    else
#      define IFM3D_SWUPDATER_EXPORT __declspec(dllimport)
#    endif
#  else
#    define IFM3D_SWUPDATER_EXPORT __attribute__((visibility("default")))
#  endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_SWUPDATER_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#  define IFM3D_SWUPDATER_DEPRECATED __declspec(deprecated)
#else
#  define IFM3D_SWUPDATER_DEPRECATED
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_SWUPDATER_LOCAL __attribute__((visibility("hidden")))
#else
#  define IFM3D_SWUPDATER_LOCAL
#endif

/** \defgroup SWUpdater SWUpdater Module */

#endif /* IFM3D_SWUPDATER_EXPORT_HPP */
