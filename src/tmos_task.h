/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_TMOS_TASK_H
#define SRC_TMOS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_STK_SIZE   (configMINIMAL_STACK_SIZE * 5)
#define TMOS_TASK_PRIO  (configMAX_PRIORITIES - 1)

extern uint32_t ble_task_rtc_trig;
extern TaskHandle_t tmos_handle;

void tmos_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TMOS_TASK_H */