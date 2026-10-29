#pragma once
inline unsigned loggedErrors = 0;
#define LOG_ERR(...) (++loggedErrors)
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
