#ifndef LED_RGB_H
#define LED_RGB_H

#include "driver/ledc.h"

/**
 * @brief Configuration structure for RGB LED control
 *
 * This structure contains the parameters required to configure the LEDC (LED Controller)
 * timer and channels for controlling an RGB LED.
 */
typedef struct {
    ledc_mode_t speed_mode;               /*!< PWM speed mode (high-speed or low-speed) */
    ledc_timer_bit_t duty_resolution;     /*!< Resolution of the PWM duty cycle */
    ledc_timer_t timer_num;               /*!< Timer number used for PWM generation */
    uint32_t freq_hz;                     /*!< Frequency of the PWM signal */

    int R_gpio_num;                       /*!< GPIO number for the red channel */
    ledc_channel_t R_channel;             /*!< LEDC channel for the red LED */

    int G_gpio_num;                       /*!< GPIO number for the green channel */
    ledc_channel_t G_channel;             /*!< LEDC channel for the green LED */

    int B_gpio_num;                       /*!< GPIO number for the blue channel */
    ledc_channel_t B_channel;             /*!< LEDC channel for the blue LED */
} led_RGB;

/**
 * @brief Initialize the boot button
 *
 * This function initializes the boot button as an input.
 */
void boot_button_init();

/**
 * @brief Initialize and configure the RGB LED
 *
 * This function configures the LEDC timer and channels based on the provided
 * RGB LED configuration structure.
 *
 * @param led Pointer to the `led_RGB` structure containing configuration details.
 */
void RGB_led_init(led_RGB* led);

/**
 * @brief Set the color of the RGB LED
 *
 * This function sets the brightness levels of the red, green, and blue LEDs to
 * achieve the desired color.
 *
 * @param led Pointer to the `led_RGB` structure containing configuration details.
 * @param red Brightness level for the red LED (0-255).
 * @param green Brightness level for the green LED (0-255).
 * @param blue Brightness level for the blue LED (0-255).
 */
void RGB_led_set_color(led_RGB *led, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Indicates if the LED is turned on
 *
 * This volatile variable can be used to check or set the current state of the LED.
 */
extern volatile bool led_on;

#endif // LED_RGB_H