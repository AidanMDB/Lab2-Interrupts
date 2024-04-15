/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Oct 17, 2022
  * @brief   ECE 362 Lab 6 Student template
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdint.h>

void initc();
void initb();
void togglexn(GPIO_TypeDef *port, int n);
void init_exti();
void set_col(int col);
void SysTick_Handler();
void init_systick();
void adjust_priorities();

extern void nano_wait(int);

volatile int current_col = 1;

int main(void) {
    // Uncomment when most things are working
    autotest();
    
    initb();
    initc();
    init_exti();
    init_systick();
    adjust_priorities();

    // Slowly blinking
    for(;;) {
        togglexn(GPIOC, 9);
        nano_wait(500000000);
    }
}

/**
 * @brief Init GPIO port C
 *        PC0-PC3 as input pins with the pull down resistor enabled
 *        PC4-PC9 as output pins
 * 
 */
void initc() {
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC -> MODER &= ~0x000fffff;
    GPIOC -> MODER |=  0x00055500;
    GPIOC -> PUPDR &= ~0x000000ff;
    GPIOC -> PUPDR |=  0x000000aa;
}

/**
 * @brief Init GPIO port B
 *        PB0, PB2, PB3, PB4 as input pins
 *          enable pull down resistor on PB2 and PB3
 *        PB8-PB11 as output pins
 * 
 */
void initb() {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB -> MODER &= ~0x00ff03f3;
    GPIOB -> MODER |=  0x00550000;
    GPIOB -> PUPDR &= ~0x000000f0;
    GPIOB -> PUPDR |=  0x000000a0;
}

/**
 * @brief Change the ODR value from 0 to 1 or 1 to 0 for a specified 
 *        pin of a port.
 * 
 * @param port : The passed in GPIO Port
 * @param n    : The pin number
 */
void togglexn(GPIO_TypeDef *port, int n) {
       port -> ODR ^= 1 << n; //XOR
}

//==========================================================
// Write the EXTI interrupt handler for pins 0 and 1 below.
// Copy the name from startup/startup_stm32.s, create a label
// of that name below, declare it to be global, and declare
// it to be a function.
// It acknowledge the pending bit for pin 0, and it should
// call togglexn(GPIOB, 8).
void EXTI0_1_IRQHandler() {
    EXTI -> PR = EXTI_PR_PR0;
    togglexn(GPIOB, 8);
}

//==========================================================
// Write the EXTI interrupt handler for pins 2-3 below.
// It should acknowledge the pending bit for pin2, and it
// should call togglexn(GPIOB, 9).
void EXTI2_3_IRQHandler() {
    EXTI -> PR = EXTI_PR_PR2;
    togglexn(GPIOB, 9);
}

//==========================================================
// Write the EXTI interrupt handler for pins 4-15 below.
// It should acknowledge the pending bit for pin4, and it
// should call togglxn(GPIOB, 10).
void EXTI4_15_IRQHandler() {
    EXTI -> PR = EXTI_PR_PR4;
    togglexn(GPIOB, 10);
}

/**
 * @brief Follow lab manual section 4.4 to initialize EXTI
 *        (1-2) Enable the SYSCFG subsystem, and select Port B for
 *            pins 0, 2, 3, and 4.
 *        (3) Configure the EXTI_RTSR register so that an EXTI
 *            interrupt is generated on the rising edge of
 *            pins 0, 2, 3, and 4.
 *        (4) Configure the EXTI_IMR register so that the EXTI
 *            interrupts are unmasked for pins 2, 3, and 4.
 *        (5) Enable the three interupts for EXTI pins 0-1, 2-3 and
 *            4-15. Don't enable any other interrupts.
 */
void init_exti() {
    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
    SYSCFG -> EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;

    EXTI -> RTSR |= EXTI_RTSR_TR0;
    EXTI -> RTSR |= EXTI_RTSR_TR2;
    EXTI -> RTSR |= EXTI_RTSR_TR3;
    EXTI -> RTSR |= EXTI_RTSR_TR4;

    EXTI -> IMR |= EXTI_IMR_MR0;
    EXTI -> IMR |= EXTI_IMR_MR2;
    EXTI -> IMR |= EXTI_IMR_MR3;
    EXTI -> IMR |= EXTI_IMR_MR4;

    NVIC -> ISER[0] |= 1 << EXTI0_1_IRQn;
    NVIC -> ISER[0] |= 1 << EXTI2_3_IRQn;
    NVIC -> ISER[0] |= 1 << EXTI4_15_IRQn;

}

/**
 * @brief For the keypad pins, 
 *        Set the specified column level to logic "high.
 *        Set the other three three columns to logic "low".
 * 
 * @param col 
 */
void set_col(int col) {
    // Set PC4-7 (i.e. all columns) output to be 0
        // Set the column `col` output to be 1
        //  if col = 1, PC7 will be set to 1 as
        //  it is connected to column 1 of the keypad
        //  Likewise, if col = 4, PC4 will be set to 1
    GPIOC -> ODR &= ~0x000000f0;
    GPIOC -> ODR |=  0x00000000;

    //BSR
    if (col == 1) {
        GPIOC -> ODR |= 0x0000080;
    }
    else if (col == 2) {
        GPIOC -> ODR |= 0x0000040;
    }
    else if (col == 3) {
        GPIOC -> ODR |= 0x0000020;
    }
    else if (col == 4) {
        GPIOC -> ODR |= 0x0000010;
    }
}

/**
 * @brief The ISR for the SysTick interrupt.
 * 
 */

void SysTick_Handler() {
    // 1. Read the row pins using GPIOC->IDR
    //    You can check the pins used for rows
    //    of keypad in lab 5 manual
    // 2. If the var `current_col` corresponds to
    //    the row value, toggle one of the leds connected
    //    to PB8-11.
    //    Basically the same we have done in lab 5
    // 3. Increment the `current_col` and wrap around
    //    to 1 if `current_col` > 4. So that next time
    //    we scan the next column
    // 4. Set the changed column pin designated by `current_col`
    //    to 1 and rest of the column pins to 0 to energized that
    //    particular column for next read of keypad.
    int row = (GPIOC -> IDR) & 0xf;

    if(current_col == 1 && row == 0b1000) {
        togglexn(GPIOB, 8);
    }
    else if(current_col == 2 && row == 0b0100) {
        togglexn(GPIOB, 9);
    }
    else if(current_col == 3 && row == 0b0010) {
        togglexn(GPIOB, 10);
    }
    else if(current_col == 4 && row == 0b0001) {
        togglexn(GPIOB, 11);
    }
    current_col += 1;
    if(current_col > 4) {
        current_col = 1;
    }
    set_col(current_col);
}

/**
 * @brief Enable the SysTick interrupt to occur every 1/16 seconds.
 * 
 */
void init_systick() {
    SysTick -> LOAD = 6000000/16 - 1;
    SysTick -> CTRL |= ( SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
    SysTick -> CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
}

/**
 * @brief Bonus question
 *        Set the priority for EXTI pins 2-3 interrupt to 192.
 *        Set the priority for EXTI pins 4-15 interrupt to 128.
 *        Do not adjust the priority for any other interrupts.
 * 
 */
void adjust_priorities() {

}
