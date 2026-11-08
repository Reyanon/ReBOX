#ifndef RBX_PLATFORM_LOG_H
#define RBX_PLATFORM_LOG_H

// Built-in UDP logger (docs/RULES.md 5-1)
// - Port 18194. Receive on PC with `nc -lu 18194` or `python tools/udp_log.py`.
// - Target address set at build time via RBX_LOG_HOST. Broadcast if unset.
// - Without RBX_ENABLE_LOG (release builds) all calls are compiled out.

#define RBX_LOG_PORT 18194

#ifdef RBX_ENABLE_LOG

void rbx_log_init(void);
void rbx_log_shutdown(void);
void rbx_log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#define RBX_LOG(...) rbx_log_printf(__VA_ARGS__)

#else

#define rbx_log_init()     ((void)0)
#define rbx_log_shutdown() ((void)0)
#define RBX_LOG(...)       ((void)0)

#endif

#endif
