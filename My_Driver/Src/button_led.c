/*
 * button_led.c
 *
 *  Created on: Apr 11, 2024
 *      Author: Acer
 */


#include <stdint.h>
#include "main.h"
#include "button_led.h"

#define GPIOD_BASE_ADDR 0x40020C00
#define GPIOA_BASE_ADDR 0x40020000

/**
 * @brief init leds
 * @param None
 * @retval None
 */
void leds_init()
{
    //  enable clock GPIOD
    __HAL_RCC_GPIOD_CLK_ENABLE();
    //  set PD12, 13, 14, 15 in output mode (push-pull)
    uint32_t *GPIOD_MODDER = (uint32_t *)(GPIOD_BASE_ADDR + 0x00);
    uint32_t *GPIOD_OTYPER = (uint32_t *)(GPIOD_BASE_ADDR + 0x04);
    // SET PD12, 13, 14, 15 in Output mode
    *GPIOD_MODDER |= (0b01 << 24); // PD12
    *GPIOD_MODDER |= (0b01 << 26); // PD13
    *GPIOD_MODDER |= (0b01 << 28); // PD14
    *GPIOD_MODDER |= (0b01 << 30); // PD15
    //*GPIOD_MODDER |= (0b01 << 24) | (0b01 << 26) | (0b01 << 28) | (0b01 << 30);
    // Set output type of PD12, 13, 14, 15 are push-pull
    *GPIOD_OTYPER &= ~(0b1111 << 12);
}

/**
 * @brief Control the led
 *
 * @param
 *      led: the led is controled
 *      state: led state to control
 * @retval none
 */

void led_crtl(led_t led, uint8_t state)
{
    // write state into output data register

    uint32_t *GPIOD_ODR = (uint32_t *)(0x40020C14);
    if (state == 1)

    {

        // set pin in led to 1
        *GPIOD_ODR |= (1 << led);
    }

    else

    {

        // clear pin in led to 0
        *GPIOD_ODR &= ~(1 << led);
    }
}

/**
 * @brief init button
 * @param None
 * @return None
 */
void button_init()
{
    //  enable clock GPIOA
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // set PA0 in INPUT mode
    uint32_t *MODER = (uint32_t *)(GPIOA_BASE_ADDR + 0x00);
    *MODER &= ~(0b11 << 0);
    // ne need pull-up/down. set in floating
    uint32_t *PUPDR = (uint32_t *)(GPIOA_BASE_ADDR + 0x0C);
    *PUPDR &= ~(0b11 << 0);
}

/**
 * @brief Read button state
 * @param None
 * @retval
 *       0: not pressed
 *       1: pressed
 */
char buttonReadState()
{

    // Read bit 0 in input data register of GPIOA (0x40020010)

    uint32_t *IDR = (uint32_t *)(GPIOA_BASE_ADDR + 0x10);

    char button_state = (*IDR >> 0) & 1;

    return button_state;
}
