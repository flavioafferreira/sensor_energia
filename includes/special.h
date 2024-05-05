
float ntc_temperature(uint16_t conversao,uint8_t sensor_number);
void display_play(uint8_t message_display[30],uint32_t interval,uint8_t qty);
void display_temperature_oled(float *data);
void bungiorno_oled(uint32_t interval);
float current_sensor(uint16_t conversao);

#define MaxBuf 164

#define MAX_COMMANDS 10
#define MAX_COMMAND_LENGTH 2
#define MAX_PASSWORD_LENGTH 6
#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE



typedef struct _Data_Return_{
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
}Data_Return;

typedef struct {
    char command[MAX_COMMAND_LENGTH];
    void (*function)(uint16_t);
} Command;

Data_Return cmd_interpreter(uint8_t *data,uint8_t len);
void color(uint8_t color);






