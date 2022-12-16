/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "debug.h"
#include <stdlib.h>
#include <stdarg.h>

static char log_buf[128];

/**
 * This function is print flash debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 *
 */
void log_debug(const char *file, const long line, const char *format, ...)
{

    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    log_print("[D](%s:%ld) ", file, line);
    /* must use vprintf to print */
    vsprintf(log_buf, format, args);
    log_print("%s", log_buf);
    printf("\n");
    va_end(args);


}
/**
 * This function is print flash routine info.
 *
 * @param format output format
 * @param ... args
 */
void log_info(const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    /* must use vprintf to print */
    vsprintf(log_buf, format, args);
    log_print("%s", log_buf);
    printf("\n");
    va_end(args);
}
/**
 * This function is print flash non-package info.
 *
 * @param format output format
 * @param ... args
 */
void log_print(const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    /* must use vprintf to print */
    vsprintf(log_buf, format, args);
    printf("%s", log_buf);
    va_end(args);
}