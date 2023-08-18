#ifndef IFM3D_FRAME_GRABBER_EXPORT_HPP
#define IFM3D_FRAME_GRABBER_EXPORT_HPP

#if defined(IFM3D_FRAME_GRABBER_STATIC_LIB)
#  define IFM3D_FRAME_GRABBER_EXPORT
#else
#  if defined(_MSC_VER)
#    ifdef IFM3D_FRAME_GRABBER_DLL_BUILD
#      define IFM3D_FRAME_GRABBER_EXPORT __declspec(dllexport)
#    else
#      define IFM3D_FRAME_GRABBER_EXPORT __declspec(dllimport)
#    endif
#  else
#    define IFM3D_FRAME_GRABBER_EXPORT __attribute__((visibility("default")))
#  endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_FRAME_GRABBER_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#  define IFM3D_FRAME_GRABBER_DEPRECATED __declspec(deprecated)
#else
#  define IFM3D_FRAME_GRABEER_DEPRECATED
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_FRAME_GRABBER_LOCAL __attribute__((visibility("hidden")))
#else
#  define IFM3D_FRAME_GRABBER_LOCAL
#endif

/** \defgroup FrameGrabber FrameGrabber Module */

#endif /* IFM3D_FRAME_GRABBER_EXPORT_HPP */
