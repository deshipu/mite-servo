#include "ch32v003fun.h"
#include <stdio.h>

// Number of ticks elapsed per millisecond (48,000 when using 48MHz Clock)
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)

volatile uint32_t systick_ticks;

/*
 * initialize TIM1 for PWM
 */
void t1pwm_init(void) {
    // Enable GPIOC, GPIOD and TIM1
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC |
	RCC_APB2Periph_TIM1;
    // PD2
    GPIOD->CFGLR &= ~(0xf << (4 * 2));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 2);
    // PC7
    //GPIOC->CFGLR &= ~(0xf << (4 * 7));
    //GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 7);
    // PC4 is T1CH4, 10MHz Output alt func, push-pull
    //GPIOC->CFGLR &= ~(0xf << (4 * 4));
    //GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 4);
    // Reset TIM1 to init all regs
    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    // CTLR1: default is up, events generated, edge align
    // SMCFGR: default clk input is CK_INT

    // Prescaler
    TIM1->PSC = 0x0002;
    // Auto Reload - sets period
    TIM1->ATRLR = 0xffff;
    // One-pulse mode
    TIM1->CTLR1 |= TIM_OPM;
    // direction
    TIM1->CTLR1 |= TIM_DIR;
    // Reload immediately
    TIM1->SWEVGR |= TIM_UG;
    // Enable CH1 output, positive pol
    TIM1->CCER |= TIM_CC1E; //| TIM_CC1P;
    // Enable CH4 output, positive pol
    TIM1->CCER |= TIM_CC4E | TIM_CC4P;
    // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;
    // CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;
    // Set the Capture Compare Register value to 50% initially
    TIM1->CH1CVR = 128;
    TIM1->CH4CVR = 128;
    // Enable TIM1 outputs
    TIM1->BDTR |= TIM_MOE;
    // Enable TIM1
    TIM1->CTLR1 |= TIM_CEN;
}

/*
 * set timer channel PW
 */
void t1pwm_setpw(uint8_t chl, uint16_t width) {
    switch (chl & 3) {
    case 0:
	TIM1->CH1CVR = width;
	break;
    case 1:
	TIM1->CH2CVR = width;
	break;
    case 2:
	TIM1->CH3CVR = width;
	break;
    case 3:
	TIM1->CH4CVR = width;
	break;
    }
}

/*
 * Initialises the SysTick to trigger an IRQ with auto-reload, using HCLK/1 as
 * its clock source
 */
void systick_init(void) {
    // Reset any pre-existing configuration
    SysTick->CTLR = 0x0000;
    // Set the compare register to trigger once per millisecond
    SysTick->CMP = SYSTICK_ONE_MILLISECOND - 1;
    // Reset the Count Register, and the global millis counter to 0
    SysTick->CNT = 0x00000000;
    systick_ticks = 0x00000000;
    // Set the SysTick Configuration
    // NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
    // busywait delay funtions used by ch32v003_fun.
    SysTick->CTLR |= SYSTICK_CTLR_STE |	// Enable Counter
	SYSTICK_CTLR_STIE |	// Enable Interrupts
	SYSTICK_CTLR_STCLK;	// Set Clock Source to HCLK/1
    // Enable the SysTick IRQ
    NVIC_EnableIRQ(SysTicK_IRQn);
}

/*
 * SysTick ISR - must be lightweight to prevent the CPU from bogging down.
 * Increments Compare Register and systick_millis when triggered (every 1ms)
 * NOTE: the `__attribute__((interrupt))` attribute is very important
 */
void SysTick_Handler(void) __attribute__ ((interrupt));
void SysTick_Handler(void) {
    // Increment the Compare Register for the next trigger
    // If more than this number of ticks elapse before the trigger is reset,
    // you may miss your next interrupt trigger
    // (Make sure the IQR is lightweight and CMP value is reasonable)
    SysTick->CMP += SYSTICK_ONE_MILLISECOND * 20;
    // Clear the trigger state for the next IRQ
    SysTick->SR = 0x00000000;

    systick_ticks++;
    TIM1->CTLR1 |= TIM_CEN;
}

int main() {
    SystemInit();
    funGpioInitAll();
    t1pwm_init();
    t1pwm_setpw(0, 0x1010);
    t1pwm_setpw(3, 64);

    //funPinMode(PC4, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    systick_init();
    while (1) {
	Delay_Ms(250);
    }
}
