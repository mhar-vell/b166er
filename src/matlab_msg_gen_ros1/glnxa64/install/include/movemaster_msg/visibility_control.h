#ifndef MOVEMASTER_MSG__VISIBILITY_CONTROL_H_
#define MOVEMASTER_MSG__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVEMASTER_MSG_EXPORT __attribute__ ((dllexport))
    #define MOVEMASTER_MSG_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVEMASTER_MSG_EXPORT __declspec(dllexport)
    #define MOVEMASTER_MSG_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVEMASTER_MSG_BUILDING_LIBRARY
    #define MOVEMASTER_MSG_PUBLIC MOVEMASTER_MSG_EXPORT
  #else
    #define MOVEMASTER_MSG_PUBLIC MOVEMASTER_MSG_IMPORT
  #endif
  #define MOVEMASTER_MSG_PUBLIC_TYPE MOVEMASTER_MSG_PUBLIC
  #define MOVEMASTER_MSG_LOCAL
#else
  #define MOVEMASTER_MSG_EXPORT __attribute__ ((visibility("default")))
  #define MOVEMASTER_MSG_IMPORT
  #if __GNUC__ >= 4
    #define MOVEMASTER_MSG_PUBLIC __attribute__ ((visibility("default")))
    #define MOVEMASTER_MSG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVEMASTER_MSG_PUBLIC
    #define MOVEMASTER_MSG_LOCAL
  #endif
  #define MOVEMASTER_MSG_PUBLIC_TYPE
#endif
#endif  // MOVEMASTER_MSG__VISIBILITY_CONTROL_H_
// Generated 02-Mar-2022 12:34:27
// Copyright 2019-2020 The MathWorks, Inc.
