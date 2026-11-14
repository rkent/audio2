#ifndef AUDIO2_STREAM__VISIBILITY_CONTROL_HPP_
#define AUDIO2_STREAM__VISIBILITY_CONTROL_HPP_

// Symbol export macros for the audio2_stream shared library, following the ROS 2
// visibility_control convention. Mark public classes and free functions with
// AUDIO2_STREAM_PUBLIC. AUDIO2_STREAM_BUILDING_LIBRARY is defined only while
// compiling the library itself (see CMakeLists.txt), which selects dllexport;
// code that uses the library gets dllimport.
//
// Windows exports only data and functions that are marked, so anything
// used outside the DLL needs AUDIO2_STREAM_PUBLIC. Inline functions and
// header-only code do not.

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AUDIO2_STREAM_EXPORT __attribute__ ((dllexport))
    #define AUDIO2_STREAM_IMPORT __attribute__ ((dllimport))
  #else
    #define AUDIO2_STREAM_EXPORT __declspec(dllexport)
    #define AUDIO2_STREAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef AUDIO2_STREAM_BUILDING_LIBRARY
    #define AUDIO2_STREAM_PUBLIC AUDIO2_STREAM_EXPORT
  #else
    #define AUDIO2_STREAM_PUBLIC AUDIO2_STREAM_IMPORT
  #endif
  #define AUDIO2_STREAM_PUBLIC_TYPE AUDIO2_STREAM_PUBLIC
  #define AUDIO2_STREAM_LOCAL
#else
  #define AUDIO2_STREAM_EXPORT __attribute__ ((visibility("default")))
  #define AUDIO2_STREAM_IMPORT
  #if __GNUC__ >= 4
    #define AUDIO2_STREAM_PUBLIC __attribute__ ((visibility("default")))
    #define AUDIO2_STREAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AUDIO2_STREAM_PUBLIC
    #define AUDIO2_STREAM_LOCAL
  #endif
  #define AUDIO2_STREAM_PUBLIC_TYPE
#endif

#endif  // AUDIO2_STREAM__VISIBILITY_CONTROL_HPP_
