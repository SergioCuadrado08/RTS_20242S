#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include"freertos/FreeRTOS.h"
#include "LED_RGB.h"

extern volatile bool led_on;

void app_main() {
    led_RGB led_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .R_gpio_num = 15,
        .R_channel = LEDC_CHANNEL_0,
        .G_gpio_num = 2,
        .G_channel = LEDC_CHANNEL_1,
        .B_gpio_num = 4,
        .B_channel = LEDC_CHANNEL_2
    };

    RGB_led_init(&led_config);

    boot_button_init();
    while (1) {
        if (led_on) {
            // Cambiar colores del LED RGB
            RGB_led_set_color(&led_config, 255, 0, 0);  // Rojo
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            RGB_led_set_color(&led_config, 0, 255, 0);  // Verde
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            RGB_led_set_color(&led_config, 0, 0, 255);  // Azul
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
            // Apagar el LED RGB
            RGB_led_set_color(&led_config, 0, 0, 0);  // Negro (apagado)
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}
