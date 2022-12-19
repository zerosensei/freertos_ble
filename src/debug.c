/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "debug.h"
#include <stdlib.h>
#include <stdarg.h>

static char log_buf[128];
SemaphoreHandle_t print_mutex = NULL;

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
    log_print("\n");
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
    
    xSemaphoreTake(print_mutex, portMAX_DELAY);
    /* args point to the first variable parameter */
    va_start(args, format);
    /* must use vprintf to print */
    vsprintf(log_buf, format, args);
    printf("%s", log_buf);
    va_end(args);
#if(DEBUG == Debug_UART1) // 使用其他串口输出打印信息需要修改这行代码
    while((R8_UART1_LSR & RB_LSR_TX_ALL_EMP) == 0)
    {
        __nop();
    }
#endif
    xSemaphoreGive(print_mutex);
}

void log_init(void)
{
    print_mutex = xSemaphoreCreateMutex();

    configASSERT(print_mutex);
}