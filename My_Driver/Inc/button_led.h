#ifndef _BUTTON_LED_H
#define _BUTTON_LED_H

typedef enum
{
    LED_GREEN = 12,
    LED_ORANGE,
    LED_RED,
    LED_BLUE
} led_t;
/**
 * @brief init leds
 * @param None
 * @retval None
 */
void leds_init();
/**
 * @brief Control the led
 *
 * @param
 *      led: the led is controled
 *      state: led state to control
 * @retval none
 */

void led_crtl(led_t led, uint8_t state);
/**
 * @brief init button
 * @param None
 * @return None
 */
void button_init();
/**
 * @brief Read button state
 * @param None
 * @retval
 *       0: not pressed
 *       1: pressed
 */
char buttonReadState();

#endif
