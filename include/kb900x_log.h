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

#ifndef __KB_LOG_H
#define __KB_LOG_H

#include <stdio.h>
#include <time.h>

#define KB900X_LOG_VERSION "0.1.0"

typedef enum {
    KB900X_LOG_DEBUG = 0,
    KB900X_LOG_INFO = 1,
    KB900X_LOG_WARN = 2,
    KB900X_LOG_ERR = 3,
    KB900X_LOG_FATAL = 4,
    KB900X_LOG_LEVEL_LENGTH = 5
} KB900X_LOG_LEVEL;

void kandou_log_msg(KB900X_LOG_LEVEL level, const char *file, int line, const char *fmt, ...);
void kandou_log_set_level(int level);
void kandou_log_set_output_file(FILE *fp);
void kandou_log_set_quiet(int enable);
// void kandou_log_set_udata(void *udata);
// void kandou_log_set_lock(logLockFn fn);
// void kandou_log_set_callback(void (*ptr)());

#define KANDOU_DEBUG(...) kandou_log_msg(KB900X_LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define KANDOU_INFO(...) kandou_log_msg(KB900X_LOG_INFO, __FILE__, __LINE__, __VA_ARGS__)
#define KANDOU_WARN(...) kandou_log_msg(KB900X_LOG_WARN, __FILE__, __LINE__, __VA_ARGS__)
#define KANDOU_ERR(...) kandou_log_msg(KB900X_LOG_ERR, __FILE__, __LINE__, __VA_ARGS__)
#define KANDOU_FATAL(...) kandou_log_msg(KB900X_LOG_FATAL, __FILE__, __LINE__, __VA_ARGS__)

#endif // __KB_LOG_H
