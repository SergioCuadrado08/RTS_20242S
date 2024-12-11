#ifndef LED_RGB_H
#define LED_RGB_H

#include "driver/ledc.h"
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// Definición de la estructura
typedef struct {
    ledc_mode_t speed_mode;               /*!< Modo de velocidad del PWM */
    ledc_timer_bit_t duty_resolution;     /*!< Resolución del duty cycle */
    ledc_timer_t timer_num;               /*!< Timer asignado */
    uint32_t freq_hz;                     /*!< Frecuencia del PWM */
    
    int R_gpio_num;                       /*!< GPIO para el canal rojo */
    ledc_channel_t R_channel;             /*!< Canal del LEDC para el rojo */
    
    int G_gpio_num;                       /*!< GPIO para el canal verde */
    ledc_channel_t G_channel;             /*!< Canal del LEDC para el verde */
    
    int B_gpio_num;                       /*!< GPIO para el canal azul */
    ledc_channel_t B_channel;             /*!< Canal del LEDC para el azul */
} led_RGB; 

// Prototipos de funciones
extern volatile bool led_on;
void boot_button_init();
void RGB_led_init(led_RGB *led);  
void RGB_led_set_color(led_RGB *led, uint8_t red, uint8_t green, uint8_t blue); 

#endif // LED_RGB_H