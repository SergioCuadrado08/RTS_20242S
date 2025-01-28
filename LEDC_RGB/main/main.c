#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include"freertos/FreeRTOS.h"
#include "LED_RGB.h"
#include "esp_adc_cal.h"

extern volatile bool led_on;

#define POT_CHANNEL ADC1_CHANNEL_6  // GPIO34 para el potenciómetro en ESP32
#define ADC_WIDTH ADC_WIDTH_BIT_12  // Resolución de 12 bits (0 - 4095)
#define ADC_ATTEN ADC_ATTEN_DB_11   // Atenuación para rango de voltaje 0-3.3V

int lectura_adc = 0;
uint8_t brillo = 0; 

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
    adc1_config_width(ADC_WIDTH);                          // Configurar resolución
    adc1_config_channel_atten(POT_CHANNEL, ADC_ATTEN);     // Configurar atenuación
    
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
