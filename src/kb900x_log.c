/*
 * Copyright (c) Kandou-AI.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kb900x_log.h"
#include <stdarg.h>
#include <stdio.h>

static struct {
    // void *udata;  // For lock if multi thread
    // logLockFn lock;  // For lock if multi thread
    // void (*ptr)();  // For callback if needed
    FILE *fp;
    unsigned level;
    int quiet;
} KandouLoggerState = {.fp = NULL, // Default to NULL for the file pointer
                       .level = KB900X_LOG_ERR,
                       .quiet = 0};

char *type[KB900X_LOG_LEVEL_LENGTH] = {"DEBUG", "INFO", "WARN", "ERR", "FATAL"};

#ifdef LOGGING_COLORS
char *colors[KB900X_LOG_LEVEL_LENGTH] = {
    "\x1b[36m", // Cyan
    "\x1b[32m", // Green
    "\x1b[33m", // Yellow
    "\x1b[31m", // Red
    "\x1b[35m"  // Magenta
};
#endif

time_t current_time;
struct tm *m_time;

void get_time()
{
    time(&current_time);
    m_time = localtime(&current_time);
}

void kandou_log_msg(KB900X_LOG_LEVEL level, const char *file, int line, const char *fmt, ...)
{
    if (level < KandouLoggerState.level) {
        return;
    }
    // Get current time
    get_time();
    // log to stderr
    if (!KandouLoggerState.quiet) {
        va_list args;
        const int buffer_size_err = 16;
        char buf[buffer_size_err];
        buf[strftime(buf, buffer_size_err, "%H:%M:%S", m_time)] = '\0';
#ifdef LOGGING_COLORS
        fprintf(stderr, "[%s %s%-5s\x1b[0m \x1b[90m%s:%d\x1b[0m] ", buf, colors[level], type[level],
                file, line);
#else
        fprintf(stderr, "[%s %-5s %s:%d] ", buf, type[level], file, line);
#endif
        va_start(args, fmt);
        vfprintf(stderr, fmt, args); // NOLINT
        va_end(args);
        fprintf(stderr, "\n");
        fflush(stderr);
    }
    // log to file
    if (KandouLoggerState.fp) {
        va_list args;
        const int buffer_size_file = 32;
        char buf[buffer_size_file];
        buf[strftime(buf, buffer_size_file, "%Y-%m-%d %H:%M:%S", m_time)] = '\0';
        fprintf(KandouLoggerState.fp, "[%s %-5s %s:%d] ", buf, type[level], file, line);
        va_start(args, fmt);
        vfprintf(KandouLoggerState.fp, fmt, args); // NOLINT
        va_end(args);
        fprintf(KandouLoggerState.fp, "\n");
        fflush(KandouLoggerState.fp);
    }
}

void kandou_log_set_level(int level)
{
    KandouLoggerState.level = level;
}

void kandou_log_set_output_file(FILE *fp)
{
    KandouLoggerState.fp = fp;
}

void kandou_log_set_quiet(int quiet)
{
    KandouLoggerState.quiet = quiet;
}
