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

//PRESENCE SENSOR - 
#define INIT_TIME            10 //SECONDS TO START AFTER RESET
#define SLIP_TIME_REACTIVATE 100 //SECONDS TO RECEIVE THE SECOND PULSE
#define NUMBER_OF_SENSORS      6  //

//SENSOR NUMBER
#define SENSOR_DIG_0 0
#define SENSOR_DIG_1 1
#define SENSOR_DIG_2 2
#define SENSOR_DIG_3 3
#define SENSOR_DIG_4 4



//commands
#define CMD_STOP  0x30
#define CMD_START 0x31
#define CMD_SEND_A 0x32
#define CMD_SEND_B 0x33
#define CMD_SEND_C 0x34



//CALLBACK CMDS 0
#define CMD_READ                0X52 //R
#define CMD_WRITE               0X57 //W
#define CMD_RESET               0X30 //0
#define CMD_DEV_EUI             0X31 //1
#define CMD_JOIN_EUI            0X32 //2
#define CMD_APP_EUI             0X33 //3
#define CMD_RESET_ALARM_FLAG    0X34 //4
#define CMD_LED4_ON             0X35 //5
#define CMD_LED4_OFF            0X36 //6
#define CMD_ALARM_OFF           0X37 //7
#define CMD_ALARM_ON            0X38 //8
#define CMD_TEST                0X39 //9

#define PUMP_ON_TIME   30 //SECONDS MINIMUM TIME AFTER TURN ON/OFF
#define DELTA_TEMP  0.4

#define PUMP_ON_TIME_STARTUP   5 //SECONDS
#define STARTUP_WAIT_TIME_MEASURE   5 //SECONDS BEFORE MEASURE TO STABILIZE
#define STARTUP_LIMIT_TIME  1 // LIMIT TIME OF MINUTES WORKING AFTER DETECT WATER ON STARTUP

//sensor_name   sensor_number

#define NTC_1         0
#define NTC_2         1
#define NTC_3         2

#define CIRCULAR_BUFFER_ELEMENTS 10

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


//LORAWAN
#define DELAY K_MSEC(10000)
#define DELAY_RTY K_MSEC(3000)
#define RETRY 5
#define LIMIT_RECONNECT_CNT    100  // IF THIS LIMIT REACHED, FORCE A RE-JOIN
#define LORAWAN_INTERVAL_NORMAL 10  //INTERVAL BETWEEN UPLOADS IN MINUTES             - MINIMUM 3 MINUTES
#define LORAWAN_INTERVAL_ALARM   3  //INTERVAL BETWEEN UPLOADS IN MINUTES AFTER ALARM - MINIMUM 3 MINUTES

#define DOWNLINK_BUFF_SIZE 51
#define DATA_SENT_JOIN_AGAIN 40 //AFTER HOW MANY INTERACTS MAKES A RE-JOIN
#define LORAWAN_DEV_EUI_HELIUM  {0x60, 0x81, 0xF9, 0x07, 0x40, 0x35, 0x0D, 0x69} //msb
#define LORAWAN_JOIN_EUI_HELIUM {0x60, 0x81, 0xF9, 0x82, 0xBD, 0x7F, 0x80, 0xD5} //msb
#define LORAWAN_APP_KEY_HELIUM  {0xE0, 0x07, 0x38, 0x87, 0xAF, 0x4F, 0x16, 0x6E, 0x8E, 0x52, 0xD3, 0x27, 0x0F, 0x2E, 0x64, 0x6F}

//#define LORAWAN_DEV_EUI_HELIUM  {0x60, 0x81, 0xF9, 0x07, 0x40, 0x35, 0x0D, 0x6A} //msb
//#define LORAWAN_JOIN_EUI_HELIUM {0x60, 0x81, 0xF9, 0x82, 0xBD, 0x7F, 0x80, 0xD6} //msb
//#define LORAWAN_APP_KEY_HELIUM  {0xE0, 0x07, 0x38, 0x87, 0xAF, 0x4F, 0x16, 0x6E, 0x8E, 0x52, 0xD3, 0x27, 0x0F, 0x2E, 0x64, 0x70}

//DIGITAL INPUT TIMERS
#define DIGITAL0_INPUT_DELAY          500 // 500ms -- max 2 cycles/sec
#define DIGITAL1_INPUT_DELAY          500 // 500ms -- max 2 cycles/sec

//LED
#define LED_BLINK_FAST       30  //ms
#define LED_BLINK_SLOW       501 //ms


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


typedef enum _MessageType { 
    MessageType_EVENTS = 0, 
    MessageType_STATISTICS = 1, 
    MessageType_HISTORY = 2 
} MessageType;

typedef enum _PhysicalDimension { 
    PhysicalDimension_TIME = 0, 
    PhysicalDimension_LENGTH = 1, 
    PhysicalDimension_WEIGHT = 2, 
    PhysicalDimension_TEMPERATURE = 3, 
    PhysicalDimension_SPEED = 4, 
    PhysicalDimension_AREA = 5, 
    PhysicalDimension_VOLUME = 6, 
    PhysicalDimension_ROTATION_SPEED = 7, 
    PhysicalDimension_VOLTAGE = 8, 
    PhysicalDimension_CURRENT = 9, 
    PhysicalDimension_FREQUENCY = 10, 
    PhysicalDimension_FORCE = 11 
} PhysicalDimension;


//STRUCTURE FOR HISTORY

typedef struct _InputValue_ { 
    int32_t timestamp; 
    float value; 
} InputValue_st;

typedef struct _InputData_ { 
    uint32_t input_id; /* Source ID table */
    bool enable; 
    bool has_label;
    char label[20]; 
    bool has_phy_dimension;
    PhysicalDimension phy_dimension; 
    InputValue_st values; // 800 fields 
} InputData_st;

typedef struct _Position_ { 
    int32_t timestamp; 
    float latitude; 
    float longitude; 
} Position_st;

typedef struct _History_ { 
    int32_t timestamp; 
    Position_st positions; // 800 fields
    InputValue_st device_internal_temperatures; // 800 fields 
    InputData_st input_data[6]; /* one for each input */
} History_st;


typedef struct _UplinkMessage_ { 
    MessageType type; 
    uint8_t which_Data;
    union {
        History_st history;
        //Statistics_st statistics;
        //Events_st events;
    } Data_st; 
} UplinkMessage_st;

// The structure for Circular Buffer is minor than structure for create
// and send the Protobuf. The Protobuf requires different size of fields
// Because of this, was created two different structures. 



typedef struct _Gnss_ { 
    int32_t timestamp; 
    float latitude; 
    float longitude;
    uint8_t  gps_fixed; 
    struct tm t;
} Gnss;

typedef struct _Analog_ { 
    int32_t timestamp; 
    int32_t value; 
} Analog;

typedef struct _Ntc_ { 
    int32_t timestamp; 
    int16_t value; 
} Ntc;

typedef struct _Digital_ { 
    int32_t timestamp; 
    int32_t value; 
} Digital;

//Circular Buffer Structure

typedef struct _Circular_Buffer_ { 
    uint16_t indice;
    Gnss     gnss_module;
    Analog   analog;
    Ntc      ntc[3];
    Digital  digital[2];
} _Circular_Buffer;



//Circular Buffer for Alarm System

typedef struct _Alarm_ { 
    int32_t timestamp; 
    int8_t  id;
    int8_t  event_code;
    int32_t value;     
} _Alarm;

//Alarm C_Buffer_Alarm[ALARM_EVENT_QTY_MAX];
//next position 






//LORAWAN STRUCTURES

//LORAWAN DOWNLINK FIFO
//Port 2, Pending 1, RSSI -128dB, SNR -9dBm
struct _Downlink_Fifo { 
    void *fifo_reserved;
    uint8_t port; 
    int16_t rssi;
    int8_t snr;
    uint8_t data[DOWNLINK_BUFF_SIZE];
    uint8_t len;     
};

struct _Downlink_ { 
    uint8_t port; 
    int16_t rssi;
    int8_t snr;
    uint8_t data[DOWNLINK_BUFF_SIZE];
    uint8_t len;     
};

typedef struct _Setup_{ 
    uint16_t led_blink_time;   //milliseconds  
    uint16_t interval_uplink;  //minutes       2
    uint8_t  output_port;      //ON-OFF        b7..b0 1    
    uint16_t turn_angle[4];    //degrees    -180 <--> +180 
    uint8_t  turn_speed[4];    //rpm = degrees/second   
    uint8_t  dev [8];
    uint8_t  join[8];
    uint8_t  key [16];
    uint8_t  nwk_key[16];
    uint32_t dev_nonce;
    uint8_t  joined;      //ON-OFF
}_Setup;



typedef struct _Sensor_Status_{
    uint8_t number[NUMBER_OF_SENSORS];
    uint8_t active[NUMBER_OF_SENSORS];
    uint8_t busy[NUMBER_OF_SENSORS];
}Sensor_Status_;


