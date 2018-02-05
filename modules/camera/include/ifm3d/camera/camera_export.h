#ifndef IFM3D_CAMERA_EXPORT_HPP
#define IFM3D_CAMERA_EXPORT_HPP

#ifdef IFM3D_CAMERA_STATIC_LIB
#  define COMMON_EXPORT
#else
#  ifdef IFM3D_CAMERA_DLL_BUILD
#    define IFM3D_CAMERA_EXPORT __declspec(dllexport)
#  else
#     define IFM3D_CAMERA_EXPORT __declspec(dllimport)
#  endif
#endif

#endif /* IFM3D_CAMERA_EXPORT_HPP */

