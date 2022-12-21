/********************************** (C) COPYRIGHT *******************************
 * File Name          : RTC.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2016/04/12
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#ifndef __RTC_H
#define __RTC_H

#ifdef __cplusplus
extern "C" {
#endif


#define RTC_TIMER_MAX_VALUE    0xa8c00000

#ifdef CLK_OSC32K
#if (CLK_OSC32K==1)
#define FREQ_RTC    32000
#else
#define FREQ_RTC    32768
#endif
#endif /* CLK_OSC32K */

#define ROUNDED_DIV(a, b)  (((a) + ((b) / 2)) / (b))

#define MS_TO_RTC(ms)             ((uint32_t)ROUNDED_DIV(FREQ_RTC * (ms), 1000))
#define US_TO_RTC(us)             ((uint32_t)ROUNDED_DIV(FREQ_RTC * (us), 1000 * 1000))

#define RTC_TO_MS(ms)             ((uint32_t)ROUNDED_DIV((ms) * 1000, FREQ_RTC))
#define RTC_TO_US(us)             ((uint32_t)ROUNDED_DIV((us) * 1000 * 1000, FREQ_RTC))


/**
 * @brief   Initialize time Service.
 */
void HAL_TimeInit(void);

/**
 * @brief   配置RTC触发时间
 *
 * @param   time    - 触发时间.
 */
extern void RTC_SetTignTime(uint32_t time);

#ifdef __cplusplus
}
#endif

#endif
