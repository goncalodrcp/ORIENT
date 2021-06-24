#ifndef EXAMPLE_MODEL__VISIBILITY_CONTROL_H_
#define EXAMPLE_MODEL__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EXAMPLE_MODEL_EXPORT __attribute__ ((dllexport))
    #define EXAMPLE_MODEL_IMPORT __attribute__ ((dllimport))
  #else
    #define EXAMPLE_MODEL_EXPORT __declspec(dllexport)
    #define EXAMPLE_MODEL_IMPORT __declspec(dllimport)
  #endif
  #ifdef EXAMPLE_MODEL_BUILDING_LIBRARY
    #define EXAMPLE_MODEL_PUBLIC EXAMPLE_MODEL_EXPORT
  #else
    #define EXAMPLE_MODEL_PUBLIC EXAMPLE_MODEL_IMPORT
  #endif
  #define EXAMPLE_MODEL_PUBLIC_TYPE EXAMPLE_MODEL_PUBLIC
  #define EXAMPLE_MODEL_LOCAL
#else
  #define EXAMPLE_MODEL_EXPORT __attribute__ ((visibility("default")))
  #define EXAMPLE_MODEL_IMPORT
  #if __GNUC__ >= 4
    #define EXAMPLE_MODEL_PUBLIC __attribute__ ((visibility("default")))
    #define EXAMPLE_MODEL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EXAMPLE_MODEL_PUBLIC
    #define EXAMPLE_MODEL_LOCAL
  #endif
  #define EXAMPLE_MODEL_PUBLIC_TYPE
#endif
#endif  // EXAMPLE_MODEL__VISIBILITY_CONTROL_H_
// Generated 13-Apr-2021 16:36:32
 