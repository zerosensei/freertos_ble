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

extern uint32_t tmos_task_trig;
extern TaskHandle_t tmos_handle;

void tmos_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TMOS_TASK_H */