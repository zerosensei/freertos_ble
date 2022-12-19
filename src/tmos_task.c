/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "FreeRTOS.h"
#include "task.h"
#include "soc.h"
#include "HAL.h"
#include "peripheral.h"

#define BLE_STK_SIZE   (configMINIMAL_STACK_SIZE * 2)
#define TMOS_TASK_PRIO  (configMAX_PRIORITIES - 1)

TaskHandle_t tmos_handle;
uint32_t tmos_task_trig = 0;

__attribute__((aligned(4))) uint8_t MEM_BUF[BLE_MEMHEAP_SIZE];

extern void SysTick_Handler(void);


__HIGH_CODE
void RTC_IRQHandler(void)
{
    extern uint8_t ble_task_flag;

    R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
    RTCTigFlag = 1;

#if (configUSE_TICKLESS_IDLE == 1)
    if (ble_task_flag) 
#endif
    {
        ble_task_flag = FALSE;

        xTaskResumeFromISR(tmos_handle);
        portYIELD_FROM_ISR(TRUE);
    }

    SysTick_Handler();
}

uint32_t tmos_idle(uint32_t time)
{
#if (defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    uint32_t time_sleep, time_curr;
    unsigned long irq_status;

    SYS_DisableAllIrq(&irq_status);
    time_curr = RTC_GetCycle32k();
    // 检测睡眠时间
    if (time < time_curr) {
        time_sleep = time + (RTC_TIMER_MAX_VALUE - time_curr);
    } else {
        time_sleep = time - time_curr;
    }

    // 若睡眠时间小于最小睡眠时间或大于最大睡眠时间，则不睡眠
    if ((time_sleep < SLEEP_RTC_MIN_TIME) ||
        (time_sleep > SLEEP_RTC_MAX_TIME)) {
        SYS_RecoverIrq(irq_status);
        return 2;
    }

    tmos_task_trig = time;
    SYS_RecoverIrq(irq_status);

    vTaskSuspend(tmos_handle);

#endif
    return 0;
}

void tmos_task(void *pvParameters)
{
    CH58X_BLEInit();
    HAL_Init();
    GAPRole_PeripheralInit();
    Peripheral_Init();

    while (1) {
        TMOS_SystemProcess();
    }
}

void tmos_task_init(void)
{
    xTaskCreate((TaskFunction_t)tmos_task,
            (const char *)"TMOS",
            (uint16_t)BLE_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)TMOS_TASK_PRIO,
            (TaskHandle_t *)&tmos_handle);
}