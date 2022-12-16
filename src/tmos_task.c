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

TaskHandle_t tmos_handle;
uint32_t ble_task_rtc_trig = 0;

__attribute__((aligned(4))) uint8_t MEM_BUF[BLE_MEMHEAP_SIZE];

extern void SysTick_Handler(void);


__HIGH_CODE
void RTC_IRQHandler(void)
{
    extern uint8_t ble_task_flag;
 
    R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
    RTCTigFlag = 1;

    if (ble_task_flag) 
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
    // ���˯��ʱ��
    if (time < time_curr) {
        time_sleep = time + (RTC_TIMER_MAX_VALUE - time_curr);
    } else {
        time_sleep = time - time_curr;
    }

    // ��˯��ʱ��С����С˯��ʱ���������˯��ʱ�䣬��˯��
    if ((time_sleep < SLEEP_RTC_MIN_TIME) ||
        (time_sleep > SLEEP_RTC_MAX_TIME)) {
        SYS_RecoverIrq(irq_status);
        return 2;
    }

    ble_task_rtc_trig = time;
    RTC_SetTignTime(time);
    SYS_RecoverIrq(irq_status);

    // suspend ble task
    if (!RTCTigFlag) {
        vTaskSuspend(tmos_handle);
    } else {
        return 3;
    }
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