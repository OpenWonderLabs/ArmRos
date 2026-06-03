#pragma once

// =======================
// Platform abstraction
// =======================

#if defined(_WIN32)
#define _WIN32_WINNT 0x0601
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#include <io.h>
#define access _access
#define F_OK 0
#define W_OK 2
inline void usleep(unsigned int micro_seconds) { Sleep(micro_seconds / 1000); }
inline void sleep(unsigned int seconds) { Sleep(seconds * 1000); }

#else
#include <unistd.h>
#include <sys/stat.h>
#endif

// =======================
// DLL export/import
// =======================

// C API
#if defined(_WIN32)
  #if defined(WOAN_STATIC)
    #define WOAN_C_API
  #elif defined(woanarm_c_EXPORTS)
    #define WOAN_C_API __declspec(dllexport)
  #else
    #define WOAN_C_API __declspec(dllimport)
  #endif
#else
  #define WOAN_C_API __attribute__((visibility("default")))
#endif

// C++ API
#if defined(_WIN32)
  #if defined(WOAN_STATIC)
    #define WOAN_CPP_API
  #elif defined(woanarm_core_EXPORTS)
    #define WOAN_CPP_API __declspec(dllexport)
  #else
    #define WOAN_CPP_API __declspec(dllimport)
  #endif
#else
  #define WOAN_CPP_API __attribute__((visibility("default")))
#endif
