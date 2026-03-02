#ifdef __cplusplus
extern "C" {
#endif

void log_info(const char* format, ...);
void log_warn(const char* format, ...);
void log_error(const char* format, ...);
void log_fatal(const char* format, ...);

#ifdef __cplusplus
}
#endif

