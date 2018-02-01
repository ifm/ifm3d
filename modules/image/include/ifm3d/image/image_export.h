#ifndef IFM3D_IMAGE_EXPORT_HPP
#define IFM3D_IMAGE_EXPORT_HPP

#ifdef IFM3D_IMAGE_STATIC_LIB
#  define COMMON_EXPORT
#else
#  ifdef IFM3D_IMAGE_DLL_BUILD
#    define IFM3D_IMAGE_EXPORT __declspec(dllexport)
#  else
#     define IFM3D_IMAGE_EXPORT __declspec(dllimport)
#  endif
#endif

#endif /* IFM3D_IMAGE_EXPORT_HPP */

