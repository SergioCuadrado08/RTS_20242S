

#include <stdio.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/soc_caps.h"
#include "LED_RGB.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EXAMPLE_ADC1_CHAN0          ADC1_CHANNEL_0
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12


#define SPEED_MODE              LEDC_LOW_SPEED_MODE            
#define DUTY_RESOLUTION         LEDC_TIMER_8_BIT  
#define TIMER_NUM               LEDC_TIMER_0  
#define FREQ_HZ                 (4000)
#define R_GPIO_NUM              (15)
#define R_CHANNEL               0
#define G_GPIO_NUM              (2)
#define G_CHANNEL               1
#define B_GPIO_NUM              (16)
#define B_CHANNEL               2
#define GPIO_OUTPUT_PIN_SEL_IN  (1ULL<<GPIO_NUM_0)  

volatile int button_state = 0;

QueueHandle_t queue_colors_conf;

typedef enum {
    red_conf,
    green_conf,
    blue_conf,
    rgb_conf,
}color_conf;


led_RGB LED_RGB =   {
    .speed_mode=        SPEED_MODE,                
    .duty_resolution=   DUTY_RESOLUTION ,     
    .timer_num=         TIMER_NUM,
    .freq_hz=           FREQ_HZ,                      
         
    .R_gpio_num=        R_GPIO_NUM,  
    .R_channel=         R_CHANNEL,         
    
    .G_gpio_num=        G_GPIO_NUM,  
    .G_channel=         G_CHANNEL,

    .B_gpio_num=        B_GPIO_NUM,  
    .B_channel=         B_CHANNEL
};

adc_oneshot_unit_handle_t adc_conf(void){
    adc_oneshot_unit_handle_t handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &handle));


    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, EXAMPLE_ADC1_CHAN0, &config));
    return handle;
}



void button_read (){
   
    color_conf color_conf = rgb_conf;
    while (1)
    {
        if (gpio_get_level(GPIO_NUM_0) == 0){
            xQueueReceive(queue_colors_conf, &color_conf, pdMS_TO_TICKS(100));
            if (color_conf == rgb_conf){

                color_conf = red_conf;
                xQueueSend(queue_colors_conf, &color_conf, pdMS_TO_TICKS(0));
            }
            else if (color_conf == red_conf){

                color_conf = green_conf;
                xQueueSend(queue_colors_conf, &color_conf, pdMS_TO_TICKS(0));
            }
            else if (color_conf == green_conf){

                color_conf = blue_conf;
                xQueueSend(queue_colors_conf, &color_conf, pdMS_TO_TICKS(0));
            }

            else{
                color_conf = rgb_conf;
                xQueueSend(queue_colors_conf, &color_conf, pdMS_TO_TICKS(0));
            }

            while (gpio_get_level(GPIO_NUM_0)==0){
            vTaskDelay(10); 
            }

        }
    vTaskDelay(10);
    }
    

}
void IRAM_ATTR button_isr (void *arg){
    button_state = !button_state;
}

void gpio_config_f (void){
    gpio_config_t io_config;

        io_config.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_IN;

        io_config.mode = GPIO_MODE_INPUT;

        io_config.pull_up_en = GPIO_PULLUP_ENABLE;

        io_config.pull_down_en = GPIO_PULLUP_DISABLE;

        io_config.intr_type = GPIO_INTR_NEGEDGE;

        gpio_config(&io_config);

        gpio_install_isr_service(0);
        gpio_isr_handler_add(GPIO_NUM_0, button_isr, NULL);
    }




void config_color (){
    adc_oneshot_unit_handle_t adc1_handle = adc_conf();
    color_conf color_config;
    int adc_value[1][1];

    float RED     = 0;
    float GREEN   = 0;
    float BLUE    = 0;
    
    
while (1)
    {

        adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_value[0][0]);
  
        xQueueReceive(queue_colors_conf,&color_config,pdMS_TO_TICKS(100));
        
       
        if (color_config==red_conf)
            {
                RED= (adc_value[0][0]*100)/4095; 
                RGB_led_set_color(&LED_RGB,RED,0,0);
                vTaskDelay(10 );
                printf("RED %f\n",RED);
                
            }
        else if (color_config==green_conf)
            {
                
                GREEN= (adc_value[0][0]*100)/4095; 
                RGB_led_set_color(&LED_RGB,0,GREEN,0); 
                vTaskDelay(10 );

            }
            
        else if(color_config==blue_conf)
            {
                
                BLUE= (adc_value[0][0]*100)/4095;
                RGB_led_set_color(&LED_RGB,0,0,BLUE); 
                vTaskDelay(10);

            }

        else{
            RGB_led_set_color(&LED_RGB,RED,GREEN,BLUE); 
            vTaskDelay(100 );

        }
        printf("FLAG %d\n",button_state);

    }
}

void app_main(void) {

    RGB_led_init(&LED_RGB);

    gpio_config_f();

    queue_colors_conf = xQueueCreate( 10 , sizeof(uint32_t));  
   
    xTaskCreatePinnedToCore( &button_read , "read buttom" , 2048 , NULL, 5, NULL , 0 );
    xTaskCreatePinnedToCore( &config_color , "change led state" , 2048 , NULL , 5 , NULL , 0 );
   
}

   
