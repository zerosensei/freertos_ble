/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc.h"

typedef enum {
    CLK_FLASH_4 = 0X01,
    CLK_FLASH_5 = 0X05,
    CLK_FLASH_6 = 0X02,
    CLK_FLASH_7 = 0X06,
    CLK_FLASH_8 = 0X03,
    CLK_FLASH_9 = 0X07,

    AHB_READY_SHORT  = 0X00,
    AHB_READY_NORMAL = 0X40,
    AHB_READY_LONG   = 0X80,
    AHB_READY_LONGER = 0XC0,

    AHB_SAMPLE_NORMAL = 0X00,
    AHB_SAMPLE_DELAY  = 0X10,
    AHB_SAMPLE_BEFORE = 0X20,

    AHB_SCSWIDTH_3 = 0X00,
    AHB_SCSWIDTH_2 = 0X08,

} FLASH_CLKTypeDef;

extern uint32_t _highcode_lma;
extern uint32_t _highcode_vma_start;
extern uint32_t _highcode_vma_end;

extern uint32_t _data_lma;
extern uint32_t _data_vma;
extern uint32_t _edata;

extern uint32_t _sbss;
extern uint32_t _ebss;

extern void NMI_Handler(void);
extern void Ecall_U_Mode_Handler(void);
extern void SysTick_Handler(void);
extern void SW_Handler(void);
extern void TMR0_IRQHandler(void);
extern void GPIOA_IRQHandler(void);
extern void GPIOB_IRQHandler(void);
extern void SPI0_IRQHandler(void);
extern void BB_IRQHandler(void);
extern void LLE_IRQHandler(void);
extern void USB_IRQHandler(void);
extern void TMR1_IRQHandler(void);
extern void TMR2_IRQHandler(void);
extern void UART0_IRQHandler(void);
extern void UART1_IRQHandler(void);
extern void RTC_IRQHandler(void);
extern void ADC_IRQHandler(void);
extern void PWMX_IRQHandler(void);
extern void TMR3_IRQHandler(void);
extern void UART3_IRQHandler(void);
extern void Ecall_M_Mode_Handler(void);
extern void Ecall_U_Mode_Handler(void);
extern void Break_Point_Handler(void);
extern void WDOG_BAT_IRQHandler(void);

typedef void (*irq_handler_t)(void);

#define DEFINE_DEFAULT_IRQ_HANDLER(irq_handler)         \
    void irq_handler() __attribute__((weak, alias("DefaultIRQHandler")))

DEFINE_DEFAULT_IRQ_HANDLER(NMI_Handler);
DEFINE_DEFAULT_IRQ_HANDLER(HardFault_Handler);
DEFINE_DEFAULT_IRQ_HANDLER(Ecall_U_Mode_Handler);
DEFINE_DEFAULT_IRQ_HANDLER(SysTick_Handler);
DEFINE_DEFAULT_IRQ_HANDLER(SW_Handler);
DEFINE_DEFAULT_IRQ_HANDLER(TMR0_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(GPIOA_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(GPIOB_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(SPI0_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(BB_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(LLE_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(USB_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(USB2_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(TMR1_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(TMR2_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(UART0_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(UART1_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(RTC_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(ADC_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(I2C_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(PWMX_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(TMR3_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(UART2_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(UART3_IRQHandler);
DEFINE_DEFAULT_IRQ_HANDLER(Ecall_M_Mode_Handler);
DEFINE_DEFAULT_IRQ_HANDLER(Break_Point_Handler);
DEFINE_DEFAULT_IRQ_HANDLER(WDOG_BAT_IRQHandler);

static inline void DefaultIRQHandler(void)
{
    for (;;) {
    }
}

__attribute__((section("user_vectors"))) 
const irq_handler_t isrTable[] = {
    0,
    0,
    NMI_Handler,                 /* NMI Handler */
    HardFault_Handler,           /* Hard Fault Handler */
    (irq_handler_t) 0xF5F9BDA9,
    Ecall_M_Mode_Handler,        /* 5 */
    0,
    0,
    Ecall_U_Mode_Handler,		/* 8 */
    Break_Point_Handler,			/* 9 */
    0,
    0,
    SysTick_Handler,            /* SysTick Handler */
    0,
    SW_Handler,                 /* SW Handler */
    0,
    /* External Interrupts */
    TMR0_IRQHandler,            /* 0:  TMR0 */
    GPIOA_IRQHandler,          /* GPIOA */
    GPIOB_IRQHandler,           /* GPIOB */
    SPI0_IRQHandler,            /* SPI0 */
    BB_IRQHandler,              /* BLEB */
    LLE_IRQHandler,             /* BLEL */
    USB_IRQHandler,             /* USB */
    USB2_IRQHandler,			   /* USB2 */
    TMR1_IRQHandler,            /* TMR1 */
    TMR2_IRQHandler,            /* TMR2 */
    UART0_IRQHandler,           /* UART0 */
    UART1_IRQHandler,           /* UART1 */
    RTC_IRQHandler,             /* RTC */
    ADC_IRQHandler,             /* ADC */
    I2C_IRQHandler, 			/* I2C */
    PWMX_IRQHandler,            /* PWMX */
    TMR3_IRQHandler,            /* TMR3 */
    UART2_IRQHandler,           /* UART2 */
    UART3_IRQHandler,           /* UART3 */
    WDOG_BAT_IRQHandler,        /* WDOG_BAT */
};

extern uint32_t __VECTOR_TABLE[];

__attribute__((section(".highcode_copy")))
__attribute__((noinline)) 
void copy_section(uint32_t *p_load, uint32_t *p_vma, uint32_t *p_vma_end)
{
    while (p_vma <= p_vma_end) {
        *p_vma = *p_load;
        ++p_load;
        ++p_vma;
    }
}

__attribute__((section(".highcode_copy")))
__attribute__((noinline)) 
void zero_section(uint32_t *start, uint32_t *end)
{
    uint32_t *p_zero = start;

    while (p_zero <= end) {
        *p_zero = 0;
        ++p_zero;
    }
}

__attribute__((section(".highcode_copy")))
void SystemInitHook(void)
{
    uint32_t i;
    sys_safe_access_enable();
    R8_PLL_CONFIG &= ~(1 << 5);
    sys_safe_access_disable();
    if (!(R8_HFCK_PWR_CTRL & RB_CLK_PLL_PON)) {
        sys_safe_access_enable();
        R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON; // HSE power on
        for (i = 0; i < 2000; i++) {
            __nop();
            __nop();
        }
    }
    sys_safe_access_enable();
    R16_CLK_SYS_CFG = (1 << 6) | (CLK_SOURCE_PLL_60MHz & 0x1f);
    __nop();
    __nop();
    __nop();
    __nop();
    sys_safe_access_disable();

    sys_safe_access_enable();
    R8_FLASH_CFG       = CLK_FLASH_6;
    sys_safe_access_disable();

    sys_safe_access_enable();
    R8_PLL_CONFIG |= 1 << 7;
    sys_safe_access_disable();

    copy_section(&_highcode_lma, &_highcode_vma_start, &_highcode_vma_end);
    copy_section(&_data_lma, &_data_vma, &_edata);
    zero_section(&_sbss, &_ebss);
}

__attribute__((weak)) void SystemIrqHandler(uint32_t mcause)
{
    uint32_t intNum;

    if (mcause & 0x80000000) /* For external interrupt. */
    {
        intNum = mcause & 0x3FUL;

        __asm__ volatile("csrsi mstatus, 8"); /* Support nesting interrupt */

        /* Now call the real irq handler for intNum */
        isrTable[intNum]();

        __asm__ volatile("csrci mstatus, 8");
    }
}

void SystemInit(void)
{
    SystemInitHook();

    __asm__ volatile("csrw mtvec, %0" ::"r"((uint32_t)__VECTOR_TABLE)); /* MTVEC */
}
