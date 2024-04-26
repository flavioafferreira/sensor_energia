#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>


/*
 THESE INSTRUCTIONS APPLIES TO "WaterSensorV4 Montaggio Prototype Corrected New v2"
1 - TO CONFIGURE THE CAPACTIVE SENSOR SHOULD DISCONNECT THE POWER RESISTOR AND CONNECT TO THE VCC OF PROGRAMMER
2 - PUT THE SCL AND SDA AS INPUT TO ALLOW HIGH IMPEDANCE OR REMOVE THE 0OHM RESISTOR 


LED0     P1.09  WRITE TEST OK

ANALOG INPUTS
CURRENT  P0.02 ANALOG - SHOULD DO INPUT AD TEST AIN-0
NTC      P0.29 ANALOG - WRITE TEST OK           AIN-5

SERIAL
UART TX  P0.17
UART RX  P0.20

DIGITAL INPUT
IN0_P    P0.11 CONNECTED TO I2C2CL - SHOULD BE INPUT in0

J2 CONNECTOR
1- SPI-MISO P0.28 OK - WRITE TEST OK
2- SPI-MOSI P0.30 OK - WRITE TEST OK
3- SPI-CLK  P0.31 OK - WRITE TEST OK
4- SPI-CS   P0.04 OK - WRITE TEST OK
5- GND
6- ISC-SCL  P0.09 (in0-p)  - WRITE TEST OK 
7- I2C-SDA  P0.10 (in1-p)  - WRITE TEST OK
8- MOTOR    P0.15  - WRITE TEST OK dig0
9- CAP-RST  P0.03  - WRITE TEST OK dig1
10-BUTTON-0 P0.05  - WRITE TEST OK dig2

PROGRAMMER PORT
1-VDD
2-SWIO
3-SWCLK
4-RESET
5-GND
6-UART TX P0.17
7-UART RX P0.20
*/




#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000
#define CON_STATUS_LED DK_LED2


#define INITIAL_MOTOR_ANALYSE 0

#define DEBUG_CORRENT OFF
#define DEBUG 1
#define DEBUG_BLE 1
#define DEBUG_DISPLAY 0


#define CLEAR_SCREEN 0

#define LED_SLEEP_TIME_MS   300
#define ON 1
#define OFF 0

#define MOTOR_ON 1
#define MOTOR_OFF 0

#define SENSOR_CAPACITIVE_ON  0
#define SENSOR_CAPACITIVE_OFF 1

//i2c 

#define CAP_INTERVAL 100 //INTERVAL ms BETWEEN READS 
#define CAP_PRINT_VALUES_INTERVAL 1000 //INTERVAL ms BETWEEN PRINTS
#define CAP_PRODUCT_NUMBER 0X00
#define CAP_SOFTWARE_NUMBER 0X01
#define CAP_SYSTEM_FLAGS 0X10
#define CAP_STATUS 0X31
#define CAP_COUNT_HIGH 0X42
#define CAP_COUNT_LOW  0X43
#define CAP_LTA_HIGH 0X83
#define CAP_LTA_LOW 0X84
#define CAP_ALL 0XFF

#define CAP_LIMIT 150

#define CAP_READ_QTY 50
#define CAP_ELEMENT_TEST 10
#define CAP_LIMIT_TEST  600 // WITHOUT USE THE CALIBRATION TEMPERATURE TABLE
#define SECURITY_MARGIN  60





//commands
#define CMD_STOP  0x30
#define CMD_START 0x31
#define CMD_SEND_A 0x32
#define CMD_SEND_B 0x33
#define CMD_SEND_C 0x34

#define PUMP_ON_TIME   30 //SECONDS MINIMUM TIME AFTER TURN ON/OFF
#define DELTA_TEMP  0.4

#define PUMP_ON_TIME_STARTUP   5 //SECONDS
#define STARTUP_WAIT_TIME_MEASURE   5 //SECONDS BEFORE MEASURE TO STABILIZE
#define STARTUP_LIMIT_TIME  1 // LIMIT TIME OF MINUTES WORKING AFTER DETECT WATER ON STARTUP

//sensor_name   sensor_number

#define NTC_1         0
#define NTC_2         1
#define NTC_3         2

#define CIRCULAR_BUFFER_ELEMENTS 1440

#define NUMBER_OF_NTC_SENSORS 2
#define NUMBER_OF_ANALOG_SENSORS 2


#define ADC_INTERVAL            1000 //ms
#define ADC_RESOLUTION          16383 
#define ADC_VOLTAGE_REF         3.6 //volts  = internal voltage
#define VOLTAGE_ALIM            3.0 //volts  //use a regulator or diode with 1k in series in order to keep the voltage stable
#define RESISTOR_SERIE_NTC1     47000 //ohms THERMISTOR SHOULD BE GROUNDED ONE SIDE AND IN SERIES WITH TERMISTOR_SERIE_RESISTOR CONNECTED TO VDD
#define RESISTOR_SERIE_NTC2     9960 //ohms
#define RESISTOR_SERIE_NTC3     9960 //ohms

// NTC VISHAY NTCLE100E3103HB0A 10K
#define TERMISTOR_KELVIN_25   298.15 // KELVIN TEMPERATURE 25C
#define TERMISTOR_RES_25       10000 // RESISTANCE AT 25C
#define TERMISTOR_BETA          3977 // BETA

#define I2C_DEVICE_ID       01 
#define I2C_DEVICE_ADDRESS  00 

//current

#define CURRENT_LEVEL 5.20 // VALUE THRESHOLD TO DECIDE OFF THE PUMP


//CURRENT SENSOR TMCS1101A3U-Q1
#define SENSOR_SENSIBILITY 200 // 200 mv/A
#define ZERO_VOLTAGE 0.320 // 0.320 volts
#define LOW_LIMIT_CURRENT 0.1 // 0.1A



//DIGITAL INPUT TIMERS
#define DIGITAL0_INPUT_DELAY          500 // 500ms -- max 2 cycles/sec
#define DIGITAL1_INPUT_DELAY          500 // 500ms -- max 2 cycles/sec

typedef struct _Analog_ { 
    int32_t timestamp; 
    int32_t value; 
} Analog;


typedef struct _Ntc_ { 
    int32_t timestamp; 
    int16_t value; 
} Ntc;


typedef struct _Circular_Buffer_ { 
    uint16_t indice;
    Ntc      ntc[NUMBER_OF_NTC_SENSORS];
    Analog   analog[NUMBER_OF_ANALOG_SENSORS];

} _Circular_Buffer;

typedef struct _values_temp_ { 
    float value[NUMBER_OF_ANALOG_SENSORS]; 
    float difference;
} Values_Temp;


typedef struct _cap_sensor_ {
    int32_t status; 
    int32_t CS;
    int32_t CS_h;
    int32_t CS_l;
    int32_t LTA; 
    int32_t LTA_h;
    int32_t LTA_l;
    int32_t dif;
} cap_sensor;

typedef struct _motor_sensor_ {
    float current; 
    float voltage;
    float temperature;
    float current_mean;
    uint8_t initial_condition;
} motor_sensor_st;


typedef struct _power_sensor_ {
    float current_sensor_a[360];
    float current_sensor_b[360]; 
} power_sensor_st;



typedef struct _time_stamp_ {
    signed int d;
    signed int h; 
    signed int m;
    signed int s;

} time_stamp;


