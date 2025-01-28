#include "LED_RGB.h"



void RGB_led_init(led_RGB *led) {
    // Configurar el timer del PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = led->speed_mode,
        .duty_resolution = led->duty_resolution,
        .timer_num       = led->timer_num,
        .freq_hz         = led->freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configurar el canal rojo
    ledc_channel_config_t ledc_channel = {
        .speed_mode      = led->speed_mode,
        .channel         = led->R_channel,
        .timer_sel       = led->timer_num,
        .gpio_num        = led->R_gpio_num,
        .duty            = 0,
        .hpoint          = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Configurar el canal verde
    ledc_channel.channel = led->G_channel;
    ledc_channel.gpio_num = led->G_gpio_num;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Configurar el canal azul
    ledc_channel.channel = led->B_channel;
    ledc_channel.gpio_num = led->B_gpio_num;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void RGB_led_set_color(led_RGB *led, uint8_t red, uint8_t green, uint8_t blue) {
    // Establecer el duty para el rojo
    ESP_ERROR_CHECK(ledc_set_duty(led->speed_mode, led->R_channel, red));
    ESP_ERROR_CHECK(ledc_update_duty(led->speed_mode, led->R_channel));

    // Establecer el duty para el verde
    ESP_ERROR_CHECK(ledc_set_duty(led->speed_mode, led->G_channel, green));
    ESP_ERROR_CHECK(ledc_update_duty(led->speed_mode, led->G_channel));

    // Establecer el duty para el azul
    ESP_ERROR_CHECK(ledc_set_duty(led->speed_mode, led->B_channel, blue));
    ESP_ERROR_CHECK(ledc_update_duty(led->speed_mode, led->B_channel));
}
