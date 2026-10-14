#pragma once
// Host stub: evaluate args so callers see no unused parameters.
#define LOG_SINK_(...) ((void)sizeof(printf(__VA_ARGS__)))
#define LOG_ERR(tag, ...) ((void)(tag), LOG_SINK_(__VA_ARGS__))
#define LOG_INF(tag, ...) ((void)(tag), LOG_SINK_(__VA_ARGS__))
#define LOG_DBG(tag, ...) ((void)(tag), LOG_SINK_(__VA_ARGS__))
#include <cstdio>
