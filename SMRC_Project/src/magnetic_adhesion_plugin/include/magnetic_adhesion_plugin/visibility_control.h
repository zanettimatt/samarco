#ifndef MAGNETIC_ADHESION_PLUGIN__VISIBILITY_CONTROL_H_
#define MAGNETIC_ADHESION_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MAGNETIC_ADHESION_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define MAGNETIC_ADHESION_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define MAGNETIC_ADHESION_PLUGIN_EXPORT __declspec(dllexport)
    #define MAGNETIC_ADHESION_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef MAGNETIC_ADHESION_PLUGIN_BUILDING_LIBRARY
    #define MAGNETIC_ADHESION_PLUGIN_PUBLIC MAGNETIC_ADHESION_PLUGIN_EXPORT
  #else
    #define MAGNETIC_ADHESION_PLUGIN_PUBLIC MAGNETIC_ADHESION_PLUGIN_IMPORT
  #endif
  #define MAGNETIC_ADHESION_PLUGIN_PUBLIC_TYPE MAGNETIC_ADHESION_PLUGIN_PUBLIC
  #define MAGNETIC_ADHESION_PLUGIN_LOCAL
#else
  #define MAGNETIC_ADHESION_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define MAGNETIC_ADHESION_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define MAGNETIC_ADHESION_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define MAGNETIC_ADHESION_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MAGNETIC_ADHESION_PLUGIN_PUBLIC
    #define MAGNETIC_ADHESION_PLUGIN_LOCAL
  #endif
  #define MAGNETIC_ADHESION_PLUGIN_PUBLIC_TYPE
#endif

#endif  // MAGNETIC_ADHESION_PLUGIN__VISIBILITY_CONTROL_H_
