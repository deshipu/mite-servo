#include "ch32fun.h"
#include "../ch32fun/examples/i2c_slave/i2c_slave.h"
#include <stdio.h>

// Number of ticks elapsed per millisecond (48,000 when using 48MHz Clock)
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)
// I2C address to listen on
#define I2C_ADDRESS (0x09)

volatile uint32_t systick_ticks;
volatile uint8_t i2c_registers[16] = {0x00};

/*
 * initialize TIM1 for one-shot
 */
void timer1_init(void) {
    // Enable GPIOA, GPIOC, GPIOD and TIM1
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;
    // PD2
    GPIOD->CFGLR &= ~(0xf << (4 * 2));
    GPIOD->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF) << (4 * 2);
    // PA1
    GPIOA->CFGLR &= ~(0xf << (4 * 1));
    GPIOA->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF) << (4 * 1);
    // PC3
    GPIOC->CFGLR &= ~(0xf << (4 * 3));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 3);
    // PC4
    GPIOC->CFGLR &= ~(0xf << (4 * 4));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 4);

    // Reset TIM1 to init all regs
    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    // CTLR1: default is up, events generated, edge align
    // SMCFGR: default clk input is CK_INT

    // Prescaler
    TIM1->PSC = 0x0001;
    // Auto Reload - sets period
    TIM1->ATRLR = 0xffff;
    // One-pulse mode
    TIM1->CTLR1 |= TIM_OPM;
    // direction
    TIM1->CTLR1 |= TIM_DIR;
    // Reload immediately
    TIM1->SWEVGR |= TIM_UG;
    // Enable CH1 output, negative pol
    TIM1->CCER |= TIM_CC1E; // | TIM_CC1P;
    // Enable CH2 output, negative pol
    TIM1->CCER |= TIM_CC2E; // | TIM_CC2P;
    // Enable CH3 output, negative pol
    TIM1->CCER |= TIM_CC3E; // | TIM_CC3P;
    // Enable CH4 output, negative pol
    TIM1->CCER |= TIM_CC4E; // | TIM_CC4P;
    // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;
    // CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;
    // CH3 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1;
    // CH4 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;

    // Set the Capture Compare Register value to 0% initially
    TIM1->CH1CVR = 0;
    TIM1->CH2CVR = 0;
    TIM1->CH3CVR = 0;
    TIM1->CH4CVR = 0;

    // Enable TIM1 outputs
    TIM1->BDTR |= TIM_MOE;
    // Enable TIM1
    TIM1->CTLR1 |= TIM_CEN;
}

/*
 * initialize TIM2 for one-shot
 */
void timer2_init(void) {
    // Enable GPIOC, GPIOD and TIM2
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

    RCC->APB2PCENR |= RCC_APB2Periph_AFIO; // turn on remapping
    // Use alternate pins.
    AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_FULLREMAP;

    // PC1
    GPIOC->CFGLR &= ~(0xf << (4 * 1));
    GPIOC->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF) << (4 * 1);
    // PC7
    GPIOC->CFGLR &= ~(0xf << (4 * 7));
    GPIOC->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF) << (4 * 7);
    // PD6
    GPIOD->CFGLR &= ~(0xf << (4 * 6));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 6);
    // PD5
    GPIOD->CFGLR &= ~(0xf << (4 * 5));
    GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 5);

    // Reset TIM2 to init all regs
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    // CTLR1: default is up, events generated, edge align
    // SMCFGR: default clk input is CK_INT

    // Prescaler
    TIM2->PSC = 0x0001;
    // Auto Reload - sets period
    TIM2->ATRLR = 0xffff;
    // One-pulse mode
    TIM2->CTLR1 |= TIM_OPM;
    // direction
    TIM2->CTLR1 |= TIM_DIR;
    // Reload immediately
    TIM2->SWEVGR |= TIM_UG;
    // Enable CH1 output, negative pol
    TIM2->CCER |= TIM_CC1E; // | TIM_CC1P;
    // Enable CH2 output, negative pol
    TIM2->CCER |= TIM_CC2E; // | TIM_CC2P;
    // Enable CH3 output, negative pol
    TIM2->CCER |= TIM_CC3E; // | TIM_CC3P;
    // Enable CH4 output, negative pol
    TIM2->CCER |= TIM_CC4E; // | TIM_CC4P;
    // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM2->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;
    // CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;
    // CH3 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM2->CHCTLR2 |= TIM_OC3M_2 | TIM_OC3M_1;
    // CH4 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM2->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;

    // Set the Capture Compare Register value to 50% initially
    TIM2->CH1CVR = 0;
    TIM2->CH2CVR = 0;
    TIM2->CH3CVR = 0;
    TIM2->CH4CVR = 0;

    // initialize counter
    //TIM2->SWEVGR |= TIM_UG;

    // Enable TIM1 outputs
    TIM2->BDTR |= TIM_MOE;
    // Enable TIM1
    TIM2->CTLR1 |= TIM_CEN;
}

/*
 *  initialize i2c slave
 */
void i2c_init(void) {
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO; // turn on remapping
    funPinMode(PC6, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
    funPinMode(PD5, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL
    AFIO->PCFR1 |= AFIO_PCFR1_I2C1_HIGH_BIT_REMAP; // set high bit = 1  (I2C1REMAP1)
    AFIO->PCFR1 |= AFIO_PCFR1_I2C1_REMAP; // set low bit [ignored / don't care]    (I2C1_RM)
    SetupI2CSlave(I2C_ADDRESS, i2c_registers, sizeof(i2c_registers), NULL, NULL, false);
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
    NVIC_EnableIRQ(SysTick_IRQn);
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
    SysTick->CMP += SYSTICK_ONE_MILLISECOND * 20; // 50Hz
    // Clear the trigger state for the next IRQ
    SysTick->SR = 0x00000000;

    systick_ticks++;
//    if (systick_ticks & 1) {
//        AFIO->PCFR1 = AFIO_PCFR1_TIM1_REMAP_NOREMAP;
//    } else {
//        AFIO->PCFR1 = AFIO_PCFR1_TIM1_REMAP_FULLREMAP;
//    }
    TIM1->CH1CVR = i2c_registers[0] | (i2c_registers[1] << 8);
    TIM1->CH2CVR = i2c_registers[2] | (i2c_registers[3] << 8);
    TIM1->CH3CVR = i2c_registers[4] | (i2c_registers[5] << 8);
    TIM1->CH4CVR = i2c_registers[6] | (i2c_registers[7] << 8);
    TIM1->CTLR1 |= TIM_CEN;
    TIM2->CH1CVR = i2c_registers[8] | (i2c_registers[9] << 8);
    TIM2->CH2CVR = i2c_registers[10] | (i2c_registers[11] << 8);
    TIM2->CH3CVR = i2c_registers[12] | (i2c_registers[13] << 8);
    TIM2->CH4CVR = i2c_registers[14] | (i2c_registers[15] << 8);
    TIM2->CTLR1 |= TIM_CEN;
}

int main() {
    SystemInit();
    funGpioInitAll();
    timer1_init();
    timer2_init();
    i2c_init();

    t1pwm_setpw(0, 0x8cef);
    t1pwm_setpw(1, 0x3cef);
    t1pwm_setpw(2, 0x4cef);
    t1pwm_setpw(3, 6400);

    systick_init();
    while (1) {
	Delay_Ms(250);
    }
}
