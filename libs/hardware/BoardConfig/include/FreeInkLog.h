#pragma once

// LOG_ERR / LOG_INF / LOG_DBG for SDK code. Inside an app that provides
// <Logging.h> (CrossDink) they are the app's own macros: level filter, ms
// stamp, serial lock, RTC and PSRAM log rings. Standalone they fall back to a
// plain Serial line; host builds compile them out.
#if __has_include(<Logging.h>)
#include <Logging.h>
#elif defined(ARDUINO) && defined(ENABLE_SERIAL_LOG)
#include <Arduino.h>
#define FREEINK_LOG_LINE(level, origin, format, ...) \
  Serial.printf("[%lu] [" level "] [%s] " format "\n", millis(), origin, ##__VA_ARGS__)
#define LOG_ERR(origin, format, ...) FREEINK_LOG_LINE("ERR", origin, format, ##__VA_ARGS__)
#define LOG_INF(origin, format, ...) FREEINK_LOG_LINE("INF", origin, format, ##__VA_ARGS__)
#define LOG_DBG(origin, format, ...) FREEINK_LOG_LINE("DBG", origin, format, ##__VA_ARGS__)
#else
#define LOG_ERR(origin, format, ...)
#define LOG_INF(origin, format, ...)
#define LOG_DBG(origin, format, ...)
#endif
