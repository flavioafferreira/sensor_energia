
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>


#include <zephyr/types.h>
#include <zephyr/kernel.h>
//#include <zephyr/timing/timing.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
//new
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#define MY_DEVICE DT_PATH(soc, i2c_40003000, ssd1306_3c)


//Special Routines
#include "includes/special.h"

//Circular Buffer Structure
#include "includes/variables.h"


#include <time.h>
//#include <date_time.h>
#include <zephyr/fs/nvs.h>

_Circular_Buffer C_Buffer[CIRCULAR_BUFFER_ELEMENTS];


//display
static struct device *dev;
static uint16_t rows;
static uint8_t ppt=0;
static uint8_t font_width;
static uint8_t font_height;
#define SELECTED_FONT_INDEX  4  // perhaps make this a config parameter



float BetaTermistor(void) {
  //USE ONLY ONCE TO CALCULATE BETA
  float beta;
  float T1=(-6 + 273.15); // -6 celsius
  float T2=(56 + 273.15); // 56 celsius
  float RT1=54200/1000; //54.2k
  float RT2=2480/1000;  //2.48k
  beta = (log(RT1 / RT2)) / ((1 / T1) - (1 / T2));  // cálculo de beta.
  printf("Beta=%f\n",beta);
  return beta;
 
}
  
float ntc_temperature(uint16_t conversao,uint8_t sensor_number){
  // ELECTRIC WIRE DIAGRAM
  //  +3V --- RESISTOR_SERIE_NTC ----AD--- NTC --- GND


  //sources:  https://blog.eletrogate.com/termistor-ntc-para-controle-de-temperatura/
  //          https://elcereza.com/termistor/
  float voltageUc = conversao*(ADC_VOLTAGE_REF/(ADC_RESOLUTION-1));
  //printf("voltageUC=%f\n",voltageUc);

  float resistor=0;
  switch (sensor_number){
    case NTC_1: resistor=RESISTOR_SERIE_NTC1;break;
    case NTC_2: resistor=RESISTOR_SERIE_NTC2;break;
    case NTC_3: resistor=RESISTOR_SERIE_NTC3;break;
  }

  float Rt =  (voltageUc*resistor)/(VOLTAGE_ALIM-voltageUc);
  //printf("Rt=%f\n",Rt);
  float T = 1 /( 1 / TERMISTOR_KELVIN_25 + log(Rt / TERMISTOR_RES_25) / TERMISTOR_BETA ); 
  //printf("T=%f\n",T);
  float Tc = T - 273.15; 
  //printf("Tc=%f\n",Tc);
  return Tc;
}

float current_sensor(uint16_t conversao){

   float voltageUc = conversao * (ADC_VOLTAGE_REF/(ADC_RESOLUTION-1));

   
   float current= (voltageUc-ZERO_VOLTAGE)*(1000/SENSOR_SENSIBILITY);   //200mV=1A
   if (current<=LOW_LIMIT_CURRENT)current=0;
   
  return current;
}




void bungiorno_oled(uint32_t interval){
    uint8_t x_offset = 0;
    uint8_t y_offset;

    cfb_framebuffer_set_font(dev, 2);
    cfb_print(dev, "OASE", x_offset, y_offset);
    cfb_framebuffer_finalize(dev);
    cfb_framebuffer_clear(dev, false);
    cfb_framebuffer_set_font(dev, SELECTED_FONT_INDEX);
    k_sleep(K_MSEC(interval));


}

void display_temperature_oled(float *data)
{
    uint8_t x_offset = 0;
    uint8_t y_offset;

	char To_Send[40];
    snprintf(To_Send,sizeof(To_Send), "%02.1fC", *data);

    cfb_framebuffer_set_font(dev, 1);
    cfb_print(dev, To_Send, x_offset, y_offset);
    cfb_framebuffer_finalize(dev);
    cfb_framebuffer_clear(dev, false);
    cfb_framebuffer_set_font(dev, SELECTED_FONT_INDEX);

}



void display_play(uint8_t message_display[30],uint32_t interval,uint8_t qty)
{
    uint8_t x_offset = 0;
    uint8_t y_offset;
    uint8_t resp=0;
    uint8_t counter=0;
 
    while (counter<qty) {

        for (int i=0; i < rows; i++) {

            y_offset = i * ppt;

            switch (i) {
                case 0:
                    cfb_print(dev, message_display, x_offset, y_offset);
                    break;
                case 1:
                    cfb_print(dev, " auguro",    x_offset, y_offset);
                    break;
                case 2:
                    cfb_print(dev, "  buona ",  x_offset, y_offset);
                    break;
                case 3:
                    cfb_print(dev, "   fortuna ",    x_offset, y_offset);
                    break;
                default:
                    break;
            }

            cfb_framebuffer_finalize(dev);
            
            k_sleep(K_MSEC(interval));
            cfb_print(dev, "                     ", x_offset, y_offset);

        }

        resp=cfb_framebuffer_clear(dev, false);
        counter++;
    }


}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void display_init(void)
{
    uint8_t resp=0;

    dev = device_get_binding(DT_LABEL(MY_DEVICE));
  

    if (dev == NULL) {
        printf("OLED-Device not found\n");
        return;
    }


    if (display_set_pixel_format(dev, PIXEL_FORMAT_MONO10) != 0) {
      printf("OLED-Failed to set required pixel format\n");
        return;
    }

    printf("Binding to %s\n", DT_LABEL(MY_DEVICE));

    if (cfb_framebuffer_init(dev)) {
        printf("OLED-Framebuffer initialization failed!\n");
        return;
    }

    resp=cfb_framebuffer_clear(dev, true);

    display_blanking_off(dev);

    rows = cfb_get_display_parameter(dev, CFB_DISPLAY_ROWS);
    ppt  = cfb_get_display_parameter(dev, CFB_DISPLAY_PPT);

    int num_fonts = cfb_get_numof_fonts(dev);

    for (int idx = 0; idx < num_fonts; idx++) {

        cfb_get_font_size(dev, idx, &font_width, &font_height);

        printf("OLED-Index[%d] font dimensions %2dx%d\n",idx, font_width, font_height);
    }

    cfb_framebuffer_set_font(dev, SELECTED_FONT_INDEX);

    printf("OLED-Selected font: index[%d]\n", SELECTED_FONT_INDEX);

    cfb_framebuffer_invert(dev);  // white on black

    

    printf("OLED-x_res %d, y_res %d, ppt %d, rows %d, cols %d  resp=%d\n",
            cfb_get_display_parameter(dev, CFB_DISPLAY_WIDTH),
            cfb_get_display_parameter(dev, CFB_DISPLAY_HEIGH),
            ppt,
            rows,
            cfb_get_display_parameter(dev, CFB_DISPLAY_COLS),resp);


           
}