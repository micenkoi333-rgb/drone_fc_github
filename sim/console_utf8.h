// console_utf8.h — щоб у консолі Windows українські літери і ° показувались правильно
#pragma once
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
inline void enableUtf8Console() { SetConsoleOutputCP(CP_UTF8); }
#else
inline void enableUtf8Console() {}
#endif
