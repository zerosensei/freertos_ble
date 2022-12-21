/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc.h"
#include "HAL.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "debug.h"
#include "tmos_task.h"

SemaphoreHandle_t led_bin = NULL;

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

static void prvSetupHardware(void)
{
#ifdef DEBUG
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
#endif

    HAL_TimeInit();
#if(defined HAL_SLEEP) && (HAL_SLEEP == TRUE)
    HAL_SleepInit();
#endif
    log_init();
}

static void test1_task(void *pvParameters)
{
    uint8_t buf[200];

    while (1) {
        LOG_INFO("task1\n");
        LOG_INFO("Name		  State  Priority  Stack   Number\n");
        vTaskList((char *)buf);
        LOG_INFO("%s", buf);
        LOG_INFO("free: %ld\n", xPortGetFreeHeapSize());
        vTaskDelay((TickType_t)1000 / portTICK_PERIOD_MS);
    }
}

static void button_task(void *pvParameters)
{
    GPIOPinRemap(ENABLE, RB_PIN_INTX);
    GPIOB_SetBits(GPIO_Pin_18); 
    GPIOB_ModeCfg(GPIO_Pin_18, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(GPIO_Pin_22, GPIO_ModeIN_PU);
    GPIOB_ITModeCfg(GPIO_Pin_22, GPIO_ITMode_FallEdge);
    PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, Long_Delay);
    PFIC_EnableIRQ(GPIO_B_IRQn);

    led_bin = xSemaphoreCreateBinary();
    configASSERT(led_bin);

    while (1) {
        xSemaphoreTake(led_bin, portMAX_DELAY);
        GPIOB_InverseBits(GPIO_Pin_18);
    }
}

__HIGH_CODE
void GPIOB_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = TRUE;

    if (GPIOB_ReadITFlagBit(GPIO_Pin_22)) {
        GPIOB_ClearITFlagBit(GPIO_Pin_22);
        xSemaphoreGiveFromISR(led_bin, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

int main()
{
    prvSetupHardware();
    tmos_task_init();
    
    xTaskCreate((TaskFunction_t)test1_task,
                (const char *)"test1",
                (uint16_t)(configMINIMAL_STACK_SIZE * 2),
                (void *)NULL,
                (UBaseType_t)(tskIDLE_PRIORITY + 1),
                NULL);

    xTaskCreate((TaskFunction_t)button_task,
                (const char *)"button",
                (uint16_t)(configMINIMAL_STACK_SIZE),
                (void *)NULL,
                (UBaseType_t)(tskIDLE_PRIORITY + 2),
                NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks	to be created.  See the memory management section on the
    FreeRTOS web site for more details. */
    for (;;)
        ;
}

void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */

    configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */

	/* Force an assert. */
    configASSERT_MSG(( volatile void * ) NULL, "task name: %s", pcTaskName);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
  volatile size_t xFreeHeapSpace;

	/* This is just a trivial example of an idle hook.  It is called on each
	cycle of the idle task.  It must *NOT* attempt to block.  In this case the
	idle task just queries the amount of FreeRTOS heap that remains.  See the
	memory management section on the http://www.FreeRTOS.org web site for memory
	management options.  If there is a lot of heap memory free then the
	configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
	RAM. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	/* Remove compiler warning about xFreeHeapSpace being set but never used. */
	( void ) xFreeHeapSpace;
}
/*-----------------------------------------------------------*/
void vPortSetupTimerInterrupt(void)
{
    uint8_t ret;

    /* No CLINT so use the SysTick to generate the tick interrupt. */
    ret = SysTick_Config(configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);
    LOG_ASSERT(ret == 0);
    PFIC_SetPriority(SysTick_IRQn, configKERNEL_INTERRUPT_PRIORITY - 1);
}

__HIGH_CODE
static inline void irq_handle(uint32_t mcause)
{
    uint32_t ulInterruptNumber;
    typedef void (*irq_handler_t)(void);
    extern const irq_handler_t isrTable[];

    ulInterruptNumber = mcause & 0x3FUL;

    /* Now call the real irq handler for ulInterruptNumber */
    isrTable[ulInterruptNumber]();
}

void SystemIrqHandler(uint32_t mcause)
{   
    irq_handle(mcause);
}

__HIGH_CODE
void SysTick_Handler(void)
{
    /* vPortSetupTimerInterrupt() uses SysTick to generate the tick .+. */
    if (xTaskIncrementTick() != 0) {
        vTaskSwitchContext();
    }
    SysTick->SR &= ~(1 << 0);
}

static char *cause_str(uint32_t cause)
{
    switch (cause) {
        case 0:
            return "Instruction address misaligned";
        case 1:
            return "Instruction Access fault";
        case 2:
            return "Illegal instruction";
        case 3:
            return "Breakpoint";
        case 4:
            return "Load address misaligned";
        case 5:
            return "Load access fault";
        case 6:
            return "Store/AMO address misaligned";
        case 7:
            return "Store/AMO access fault";
        case 8:
            return "Environment call from U-mode";
        case 9:
            return "Environment call from S-mode";
        case 11:
            return "Environment call from M-mode";
        case 12:
            return "Instruction page fault";
        case 13:
            return "Load page fault";
        case 15:
            return "Store/AMO page fault";
        default:
            return "unknown";
    }
}

__HIGH_CODE
void HardFault_Handler(void)
{
    LOG_INFO("hard fault:\n");

    uint32_t mcause;
    __asm__ volatile("csrr %0, mcause"
                     : "=r"(mcause));

    uint32_t mtval;
    __asm__ volatile("csrr %0, mtval"
                     : "=r"(mtval));

    uint32_t mepc;
    __asm__ volatile("csrr %0, mepc"
                     : "=r"(mepc));

    mcause &= 0x1f;
    LOG_INFO("mcause: %ld, %s\n", mcause, cause_str(mcause));
    LOG_INFO("mtval: %lx\n", mtval);
    LOG_INFO("mepc: %lx\n", mepc);

    while (1)
        ;
}