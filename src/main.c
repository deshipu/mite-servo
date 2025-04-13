#include "ch32v003fun.h"
#include <stdio.h>

// Number of ticks elapsed per millisecond (48,000 when using 48MHz Clock)
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)

volatile uint32_t systick_ticks;

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
    SysTick->CMP += SYSTICK_ONE_MILLISECOND * 200;
    // Clear the trigger state for the next IRQ
    SysTick->SR = 0x00000000;

    systick_ticks++;
    if (systick_ticks & 1) {
        funDigitalWrite(PC4, FUN_HIGH);
    } else {
	funDigitalWrite(PC4, FUN_LOW);
    }
}

int main() {
    SystemInit();
    funGpioInitAll();
    funPinMode(PC4, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    systick_init();

    while (1) {
	Delay_Ms(250);
	Delay_Ms(250);
    }
}
