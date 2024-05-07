
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


//Lorawan
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/random/rand32.h>


//Circular Buffer
uint32_t C_Buffer_Free_Position=0;
uint32_t C_Buffer_Current_Position=0;
uint32_t C_Buffer_Alarm_Free_Position=0;
uint32_t C_Buffer_Alarm_Current_Position=0;

_Circular_Buffer C_Buffer[CIRCULAR_BUFFER_ELEMENTS];


//MUTEX
K_MUTEX_DEFINE(c_buffer_busy);

extern uint32_t data_sent_cnt;


//
extern Sensor_Status_ sensor_status;


//SEMAPHORE
extern uint8_t lorawan_reconnect;

//SETUP
_Setup Initial_Setup;

//display
static struct device *dev;
static uint16_t rows;
static uint8_t ppt=0;
static uint8_t font_width;
static uint8_t font_height;
#define SELECTED_FONT_INDEX  4  // perhaps make this a config parameter

//LED
extern uint16_t led_period;


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



///LORAWAN


//lorawan
// https://www.youtube.com/watch?v=M5VGos3YTpI&t=150s
// https://playcode.io/javascript
//https://www.exploringbinary.com/displaying-the-raw-fields-of-a-floating-point-number/



void lorawan_tx_data(void){

  char data_test[] =  { 0X00 , 0X00 , 0X00 , 0X00 , //LATITUDE
                        0X00 , 0X00 , 0X00 , 0X00 , //LONGITUDE
					    0X00 , 0X00 , 0X00 , 0X00 , //TIMESTAMP
					    0X00 , 0X00 , 0X00 , 0X00 , //ANALOG
                        0X00 ,                      //DIGITAL
                        0X00 ,                      //DIGITAL
					    0X00 , 0X00 ,               //NTC0
                        0X00 , 0X00 ,               //NTC1 
                        0X00 , 0X00                 //NTC2 
                      };
  int ret=0,nt=0,k=0;
  uint64_t j=0;

  k_mutex_lock(&c_buffer_busy, K_FOREVER);

  uint32_t pos=C_Buffer_Current_Position;
  float a=C_Buffer[pos].gnss_module.latitude;  //4 bytes 0..3
  float b=C_Buffer[pos].gnss_module.longitude; //4 bytes 4..7
  float c=C_Buffer[pos].gnss_module.timestamp; //4 bytes 8 
  float d=C_Buffer[pos].analog.value;          //4 bytes 12..17
  uint8_t e=C_Buffer[pos].digital[0].value;      //1 byte 16
  uint8_t f=C_Buffer[pos].digital[1].value;      //1 byte 17
  uint16_t g=C_Buffer[pos].ntc[0].value;          //2 bytes 18..19
  uint16_t h=C_Buffer[pos].ntc[1].value;          //2 bytes 20..21
  uint16_t i=C_Buffer[pos].ntc[2].value;          //2 bytes 22..23
                                             //total 30 bytes
  k_mutex_unlock(&c_buffer_busy);
  unsigned char *ptr_lati         = (unsigned char *) &a; //TAKE THE ADDRESS OF VARIABLES
  unsigned char *ptr_long         = (unsigned char *) &b;
  unsigned char *ptr_timestamp    = (unsigned char *) &c;
  unsigned char *ptr_analog       = (unsigned char *) &d;
  unsigned char *ptr_digi0        = (unsigned char *) &e;
  unsigned char *ptr_digi1        = (unsigned char *) &f;
  unsigned char *ptr_ntc0         = (unsigned char *) &g;
  unsigned char *ptr_ntc1         = (unsigned char *) &h;
  unsigned char *ptr_ntc2         = (unsigned char *) &i;

  
  for (int i = 0; i < sizeof(float); i++) {
     data_test[i]    =*(ptr_lati      + i);
     data_test[i+4]  =*(ptr_long      + i);
     data_test[i+8]  =*(ptr_timestamp + i);
     data_test[i+12] =*(ptr_analog    + i);
  }

     //data_test[16] =*(ptr_digi0);
     data_test[16] = sensor_status.number[SENSOR_DIG_4]; //ALARM COUNTER
     data_test[17] =*(ptr_digi1);


     data_test[18]    =*(ptr_ntc0 + 0); //first LSB and after MSB - little endian
     data_test[19]    =*(ptr_ntc0 + 1); //first LSB and after MSB - little endian
     data_test[20]    =*(ptr_ntc1 + 0); //first LSB and after MSB - little endian
     data_test[21]    =*(ptr_ntc1 + 1); //first LSB and after MSB - little endian
     data_test[22]    =*(ptr_ntc2 + 0); //first LSB and after MSB - little endian
     data_test[23]    =*(ptr_ntc2 + 1); //first LSB and after MSB - little endian



 color(12);
 printk("HELIUM PAYLOAD: ");
 for (int h = 0; h < sizeof(data_test); h++) {
     printk("%02X ",data_test[h]);
  }
  color(10);
  printk("\nSending payload...\n");
  color(255);
  data_sent_cnt++;

  ret = lorawan_send(2, data_test, sizeof(data_test),LORAWAN_MSG_UNCONFIRMED);

		if (ret < 0) {
			printk("lorawan_send confirm failed -trying again : %d\n\n", ret);

      while(ret<0 && nt<=RETRY){ 
       ret = lorawan_send(2, data_test, sizeof(data_test),LORAWAN_MSG_UNCONFIRMED);
       nt++;
       //lorawan_reconnect_cnt++;
       //if(lorawan_reconnect_cnt==LIMIT_RECONNECT_CNT){lorawan_reconnect_cnt=0;lorawan_reconnect=1;}
       if (ret==0){
        printk("Payload Data sent %d\n",data_sent_cnt);
        
        //lorawan_reconnect_cnt=0;
        }else{printk("Data send failed-trying again ret=%d \n ",ret);
              k_sleep(DELAY_RTY);
            }
      }
      nt=0;
			//return;
		}else{  color(10);
		        printk("Payload Data sent %d\n\n",data_sent_cnt);
            color(255);
            //lorawan_reconnect_cnt=0;
		     }
    if(data_sent_cnt>=DATA_SENT_JOIN_AGAIN){lorawan_reconnect=1;led_period=LED_BLINK_SLOW;}

  

}

void setup_initialize(void){

  uint8_t i;
  uint8_t dev[8] = LORAWAN_DEV_EUI_HELIUM;
  uint8_t join[8] = LORAWAN_JOIN_EUI_HELIUM;
  uint8_t key[16] = LORAWAN_APP_KEY_HELIUM;
  for(i=0;i<=7;i++){Initial_Setup.dev[i] = dev[i];} 
  for(i=0;i<=7;i++){Initial_Setup.join[i] = join[i];} 
  for(i=0;i<=15;i++){Initial_Setup.key[i] = key[i];} 
  for(i=0;i<=15;i++){Initial_Setup.nwk_key[i] = 0;} 
  Initial_Setup.joined=OFF;
  Initial_Setup.dev_nonce=0;
  
  Initial_Setup.led_blink_time=RUN_LED_BLINK_INTERVAL;
  Initial_Setup.interval_uplink=LORAWAN_INTERVAL_NORMAL;
  Initial_Setup.output_port=0;
  Initial_Setup.turn_angle[0]=0;
  Initial_Setup.turn_angle[1]=0;
  Initial_Setup.turn_angle[2]=0;
  Initial_Setup.turn_angle[3]=0;
  Initial_Setup.turn_speed[0]=0;
  Initial_Setup.turn_speed[1]=0;
  Initial_Setup.turn_speed[2]=0;
  Initial_Setup.turn_speed[3]=0;
  
  
}

void print_setup(void){
  
	  printk("Led Blink Time      : %d ms\n",Initial_Setup.led_blink_time);
	  printk("Interval UpLink Time: %d minutes\n",Initial_Setup.interval_uplink);
    printk("DEV     : ");
    for(int i=0;i<=7;i++){printk("%02X ",Initial_Setup.dev[i]);}
    printk("\n");
    printk("JOIN    : ");
    for(int i=0;i<=7;i++){printk("%02X ",Initial_Setup.join[i]);}
    printk("\n");
    printk("KEY     : ");
    for(int i=0;i<=15;i++){printk("%02X ",Initial_Setup.key[i]);}
    printk("\n");
    printk("NWK_KEY : ");
    for(int i=0;i<=15;i++){printk("%02X ",Initial_Setup.nwk_key[i]);}
    printk("\n");
    printk("DEV_NOUNCE: %08X\n",Initial_Setup.dev_nonce);
    if(Initial_Setup.joined==1){printk("JOIN = ON");}else{printk("JOIN = OFF");}
    printk("\n");


}

void color(uint8_t color) {
    switch (color) {
        case 0: printk("\033[0m");        // Preto
                break;
        case 1: printk("\033[31m");       // Vermelho
                break;
        case 2: printk("\033[32m");       // Verde
                break;
        case 3: printk("\033[33m");       // Amarelo
                break;
        case 4: printk("\033[34m");       // Azul
                break;
        case 5: printk("\033[35m");       // Magenta
                break;
        case 6: printk("\033[36m");       // Ciano
                break;
        case 7: printk("\033[37m");       // Branco
                break;
        case 8: printk("\033[90m");       // Cinza claro
                break;
        case 9: printk("\033[91m");       // Vermelho claro
                break;
        case 10: printk("\033[92m");      // Verde claro
                break;
        case 11: printk("\033[93m");      // Amarelo claro
                break;
        case 12: printk("\033[94m");      // Azul claro
                break;
        case 13: printk("\033[95m");      // Magenta claro
                break;
        case 14: printk("\033[96m");      // Ciano claro
                break;
        case 15: printk("\033[97m");      // Branco claro
                break;
        case 255: printk("\033[0m");       // Padrão (branco)
                break;
    }
}


Data_Return cmd_interpreter(uint8_t *data,uint8_t len){
  
  static Data_Return buf;
  buf.len=0;
  
  //printk("testing command \n");
  color(4);
  	switch(data[0]){
			case CMD_RESET_ALARM_FLAG: //RESET ALARM SIGNAL
			   color(1);
         sensor_status.number[SENSOR_DIG_4]=0;
         Initial_Setup.interval_uplink=LORAWAN_INTERVAL_NORMAL;
			   printk("ALARM FLAG RESET 4\n");
         buf.len=sprintf(buf.data, "ALARM FLAG RESET");
         
		     
			break;
			case CMD_LED4_ON: // TURN ON LED 4
			   color(1);
			   //gpio_pin_set_dt(LED4, ON);
			   printk("TURNED ON LED 4\n");
         buf.len=sprintf(buf.data, "TURNED ON LED 4");
         
		     
			break;
			
			case CMD_LED4_OFF: //TURN OFF LED 4
			   color(1);
			   //gpio_pin_set_dt(LED4, OFF);
			   printk("TURNED OFF LED 4\n");
         buf.len=sprintf(buf.data, "TURNED OFF LED 4");
         
			break;


      case CMD_RESET: //R
			    color(2);
			    setup_initialize();
				  //flash_write_setup();
				  print_setup();
				  printk("Setup Reset\n");
          buf.len=sprintf(buf.data, "SETUP RESET");
              
			break;

      case CMD_ALARM_OFF:
           color(1);
           sensor_status.active[SENSOR_DIG_4]=0;
           sensor_status.busy[SENSOR_DIG_4]=ON;
           printk("ALARM OFF\n");
           buf.len=sprintf(buf.data, "ALARM OFF");
      break;

      case CMD_ALARM_ON:
           color(1);
           sensor_status.active[SENSOR_DIG_4]=1;
           sensor_status.busy[SENSOR_DIG_4]=OFF;
           printk("ALARM ON\n");
           buf.len=sprintf(buf.data, "ALARM ON");
      break;


      case CMD_READ: //P
			    color(3);
			    //flash_read_setup();
			    print_setup();
          buf.len=sprintf(buf.data, "COMMAND READ"); 
			break;

      case CMD_WRITE: //Q
			     color(3);
			     
			     print_setup();
           buf.len=sprintf(buf.data, "COMMAND WRITE");             
			break;
			
      case CMD_TEST: //Q
			     color(3);
			     //test_command();
           //k_sem_give(&lorawan_tx);
			break;


		}
       color(0);
          
	  return buf;
    
}

