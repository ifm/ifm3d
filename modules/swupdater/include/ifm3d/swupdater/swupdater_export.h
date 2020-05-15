#ifndef IFM3D_SWUPDATER_EXPORT_HPP
#define IFM3D_SWUPDATER_EXPORT_HPP

#if defined(IFM3D_CAMERA_STATIC_LIB) || !defined(_MSC_VER)
#  define IFM3D_SWUPDATER_EXPORT
#else
#  ifdef IFM3D_SWUPDATER_DLL_BUILD
#    define IFM3D_SWUPDATER_EXPORT __declspec(dllexport)
#  else
#     define IFM3D_SWUPDATER_EXPORT __declspec(dllimport)
#  endif
#endif

#endif /* IFM3D_SWUPDATER_EXPORT_HPP */
