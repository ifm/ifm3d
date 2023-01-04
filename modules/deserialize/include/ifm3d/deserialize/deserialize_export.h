#ifndef IFM3D_DESERIALIZE_EXPORT_HPP
#define IFM3D_DESERIALIZE_EXPORT_HPP

#if defined(IFM3D_DESERIALIZE_STATIC_LIB)
#  define IFM3D_DESERIALIZE_EXPORT
#else
#  if defined(_MSC_VER)
#    ifdef IFM3D_DESERIALIZE_DLL_BUILD
#      define IFM3D_DESERIALIZE_EXPORT __declspec(dllexport)
#    else
#      define IFM3D_DESERIALIZE_EXPORT __declspec(dllimport)
#    endif
#  else
#    define IFM3D_DESERIALIZE_EXPORT __attribute__((visibility("default")))
#  endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_DESERIALIZE_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#  define IFM3D_DESERIALIZE_DEPRECATED __declspec(deprecated)
#else
#  define IFM3D_DESERIALIZE_DEPRECATED
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define IFM3D_DESERIALIZE_LOCAL __attribute__((visibility("hidden")))
#else
#  define IFM3D_DESERIALIZE_LOCAL
#endif

#endif /* IFM3D_DESERIALIZE_EXPORT_HPP */
