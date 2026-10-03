#pragma once

// Logging for the OPDS module. Inside an app that provides <Logging.h> these
// are the app's LOG_* (level filter, ms stamp, log rings). Otherwise they are
// self-contained with the same signatures; on host builds (ENABLE_SERIAL_LOG
// undefined) the macros expand to nothing and pull in no Arduino headers.
#if __has_include(<Logging.h>)
#include <Logging.h>
#elif defined(ENABLE_SERIAL_LOG)
#include <Arduino.h>
#define LOG_ERR(origin, format, ...) Serial.printf("[%s] " format "\n", origin, ##__VA_ARGS__)
#define LOG_INF(origin, format, ...) Serial.printf("[%s] " format "\n", origin, ##__VA_ARGS__)
#define LOG_DBG(origin, format, ...) Serial.printf("[%s] " format "\n", origin, ##__VA_ARGS__)
#else
#define LOG_ERR(origin, format, ...)
#define LOG_INF(origin, format, ...)
#define LOG_DBG(origin, format, ...)
#endif
