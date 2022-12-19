/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "HAL.h"
#include "CH58x_common.h"


/* A fiddle factor to estimate the number of SysTick counts that would have
 * occurred while the SysTick counter is stopped during tickless idle
 * calculations. */
#ifndef portMISSED_COUNTS_FACTOR
#define portMISSED_COUNTS_FACTOR (45UL)
#endif

/*
 * The number of SysTick increments that make up one tick period.
 */
#if (configUSE_TICKLESS_IDLE == 1)
static uint32_t ulTimerCountsForOneTick = configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ;
#endif /* configUSE_TICKLESS_IDLE */



/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if (configUSE_TICKLESS_IDLE == 1)
static uint32_t ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);
#endif /* configUSE_TICKLESS_IDLE */

uint8_t ble_task_flag = FALSE;


/*-----------------------------------------------------------*/

#if (configUSE_TICKLESS_IDLE == 1)
void vPreSleepProcessing(uint32_t ulExpectedIdleTime)
{
}

void vPostSleepProcessing(unsigned long xExpectedIdleTime)
{
}


__HIGH_CODE
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint64_t ulReloadValue, ulCompleteTickPeriods;
    TickType_t xModifiableIdleTime;

    /* Stop the SysTick momentarily.  The time the SysTick is stopped for
     * is accounted for as best it can be, but using the tickless mode will
     * inevitably result in some tiny drift of the time maintained by the
     * kernel with respect to calendar time. */
    SysTick->CTLR &= ~SysTick_CTLR_STE;

    /* Calculate the reload value required to wait xExpectedIdleTime
     * tick periods.  -1 is used because this code will execute part way
     * through one of the tick periods. */
    ulReloadValue = SysTick->CNT + (ulTimerCountsForOneTick * (xExpectedIdleTime - 1UL));

    if (ulReloadValue > ulStoppedTimerCompensation) {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
     * method as that will mask interrupts that should exit sleep mode. */
    portDISABLE_INTERRUPTS();

    /* If a context switch is pending or a task is waiting for the scheduler
     * to be unsuspended then abandon the low power entry. */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
        /* Reset the reload register to the value required for normal tick
         * periods. */
        SysTick->CMP = ulTimerCountsForOneTick - 1UL;

        /* Restart SysTick. */
        SysTick->CTLR |= SysTick_CTLR_STE;

        /* Re-enable interrupts - see comments above the cpsid instruction()
         * above. */
		portENABLE_INTERRUPTS();
    } else {
        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
         * set its parameter to 0 to indicate that its implementation contains
         * its own wait for interrupt or wait for event instruction, and so wfi
         * should not be executed again.  However, the original expected idle
         * time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;

        configPRE_SLEEP_PROCESSING(xModifiableIdleTime);

        static uint32_t next_rtc_trig  = 0;
        static uint32_t curr_rtc_count = 0;

        next_rtc_trig = R32_RTC_TRIG ;
        next_rtc_trig = next_rtc_trig ? next_rtc_trig : RTC_TIMER_MAX_VALUE;
        curr_rtc_count = RTC_GetCycle32k();

        /* Determine the FreeRTOS xExpectedIdleTime and TMOS which comes first */
        uint32_t expected_rtc_trig = MIN(curr_rtc_count +
             MS_TO_RTC(xExpectedIdleTime * (configTICK_RATE_HZ / 1000)), next_rtc_trig);

        /* FreeRTOS is in idle, but TMOS is about to start work, 
         * determine whether the minimum sleep time of TMOS is met */
        if (expected_rtc_trig <= (curr_rtc_count + US_TO_RTC(200))) {
            /* Reset the reload register to the value required for normal tick
             * periods. */
            SysTick->CMP = ulTimerCountsForOneTick - 1UL;

            /* Restart SysTick. */
            SysTick->CTLR |= SysTick_CTLR_STE;

            /* Re-enable interrupts - see comments above the cpsid instruction()
            * above. */
            portENABLE_INTERRUPTS();

            /* The expected tring must be TMOS rtc Trig, should switch to TMOS task */
            extern TaskHandle_t tmos_handle;
            xTaskResumeFromISR(tmos_handle);

            return;
        }

        extern uint32_t ble_task_rtc_trig;

        /* This is TMOS trc trig, TMOS should resume when wakeup */
        if (expected_rtc_trig == ble_task_rtc_trig) {
            ble_task_flag = TRUE;
        }

        RTC_SetTignTime(expected_rtc_trig);

        /* Enable interrupts to wakeup */
        portENABLE_INTERRUPTS();

        if (xModifiableIdleTime > 0) {
            // LowPower_Idle();
            LowPower_Sleep(RB_PWR_RAM2K | RB_PWR_RAM30K | RB_PWR_EXTEND);
            HSECFG_Current(HSE_RCur_100);
        }

        configPOST_SLEEP_PROCESSING(xExpectedIdleTime);

        /* Disable interrupts again because the clock is about to be stopped
         * and interrupts that execute while the clock is stopped will increase
         * any slippage between the time maintained by the RTOS and calendar
         * time. */
        portDISABLE_INTERRUPTS();

        ulCompleteTickPeriods = RTC_TO_MS(RTC_GetCycle32k() - curr_rtc_count) * (1000 / configTICK_RATE_HZ);
        RTC_SetTignTime(MAX(next_rtc_trig, 
            curr_rtc_count + MS_TO_RTC(xExpectedIdleTime * (configTICK_RATE_HZ / 1000))));
            
        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
         * again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
         * value. */
        SysTick->CMP = 0UL;
        SysTick->CTLR = SysTick_CTLR_INIT |
                        SysTick_CTLR_STRE |
                        SysTick_CTLR_STCLK |
                        SysTick_CTLR_STIE |
                        SysTick_CTLR_STE;
        vTaskStepTick(ulCompleteTickPeriods);
        SysTick->CMP = ulTimerCountsForOneTick - 1UL;

        /* Exit with interrpts enabled. */
        portENABLE_INTERRUPTS();
    }
}

#endif /* configUSE_TICKLESS_IDLE */