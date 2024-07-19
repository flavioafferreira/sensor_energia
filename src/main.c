/*
    toolchains v2.6.0
    sdk v2.4.2
    module SX1276
*/

#include "uart_async_adapter.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>
#include <stdio.h>
#include <zephyr/logging/log.h>

#include <soc.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include <complex.h>

#include <arm_math.h>

//#include <nrfx_example.h>
//#include <saadc_examples_common.h>
#include <nrfx_saadc.h>

#include "dsp/fast_math_functions.h"
#include "dsp/complex_math_functions.h"
#include "arm_const_structs.h"
#include "dsp/statistics_functions.h"
#include "dsp/statistics_functions_f16.h"
#include "dsp/filtering_functions.h"
#include "dsp/fast_math_functions.h"

#include "dsp/transform_functions.h"

#include "includes/variables.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);



//NVS
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

/* 1000 msec = 1 sec */
#define SLEEP_TIME      100
/* maximum reboot counts, make high enough to trigger sector change (buffer */
/* rotation). */
#define MAX_REBOOT 400

#define ADDRESS_ID 1
#define KEY_ID 2
#define RBT_CNT_ID 3
#define STRING_ID 4
#define LONG_ID 5

int rc = 0, cnt = 0, cnt_his = 0;
char buf[16];
uint8_t key[8], longarray[128];
uint32_t reboot_counter = 0U, reboot_counter_his;
struct flash_pages_info info;


//INIT_LORAWAN
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/random/rand32.h>

//LORAWAN 
uint8_t lorawan_reconnect=0;
uint32_t data_sent_cnt=0;
uint32_t dev_nonce;

//LED AND INPUT
uint16_t led_period=LED_BLINK_FAST;
uint16_t led_period_off=LED_OFF_MULTIPLIER;

uint8_t led_status=1;

//DIGITAL OUTPUT --> LED
#define LED_0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led_0 = GPIO_DT_SPEC_GET_OR(LED_0_NODE, gpios, {0});
#define LED_OUT_0_ADR &led_0
#define LED_OUT_0      led_0


// LORAPWR pin12
#define DIG_0_NODE DT_ALIAS(dg0)
static const struct gpio_dt_spec digital_dig0 = GPIO_DT_SPEC_GET_OR(DIG_0_NODE, gpios, {0});
#define DIG_OUT_0_ADR &digital_dig0
#define DIG_OUT_0      digital_dig0

// GPS-RST pin16
#define DIG_1_NODE DT_ALIAS(dg1)
static const struct gpio_dt_spec digital_dig1 = GPIO_DT_SPEC_GET_OR(DIG_1_NODE, gpios, {0});
#define DIG_OUT_1_ADR &digital_dig1
#define DIG_OUT_1      digital_dig1

// GPS-PWR  pin12
#define DIG_2_NODE DT_ALIAS(dg2)
static const struct gpio_dt_spec digital_dig2 = GPIO_DT_SPEC_GET_OR(DIG_2_NODE, gpios, {0});
#define DIG_OUT_2_ADR &digital_dig2
#define DIG_OUT_2      digital_dig2

//HEADER  H5 PIN 1 P0.30-AIN6-AI6
#define DIG_3_NODE DT_ALIAS(dg3)
static const struct gpio_dt_spec digital_dig3 = GPIO_DT_SPEC_GET_OR(DIG_3_NODE, gpios, {0});
#define DIG_OUT_3_ADR &digital_dig3
#define DIG_OUT_3      digital_dig3


//DIGITAL INPUT - BUTTON 0
#define DIG_IN_0_NODE DT_ALIAS(in0)
static const struct gpio_dt_spec digital_dig_in0 = GPIO_DT_SPEC_GET_OR(DIG_IN_0_NODE, gpios, {0});
static struct gpio_callback digital_cb_data_dig_in0;
#define DIG_0_ADR &digital_dig_in0
#define DIG_0      digital_dig_in0
#define DIG_0_CB  &digital_cb_data_dig_in0


//GPS
Gnss position;


//ALARM
Sensor_Status_ sensor_status;

// SEMAPHORES FOR THREADS
static K_SEM_DEFINE(gps_init,0,1);


// UART PORT DEFINITION ON app.overlay

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct k_work_delayable uart_work;

struct uart_data_t *buf_extra;
uint32_t buff_extra_index=0;
uint32_t buff_marker=0;



static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);


static K_FIFO_DEFINE(command_rx);
static K_FIFO_DEFINE(command_tx);

#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX 50000

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif


//UART END
void blink(uint8_t times);


uint8_t lora_cycle_minute=0;
static K_SEM_DEFINE(lorawan_tx, 0, 1); //uplink
static K_SEM_DEFINE(lorawan_rx, 0, 1); //downlink
static K_SEM_DEFINE(lorawan_init, 0, 1); //downlink
static K_SEM_DEFINE(timer_init,0,1);
extern _Setup Initial_Setup;

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay), "No default LoRa radio specified in DT");
#define DEFAULT_RADIO DT_LABEL(DEFAULT_RADIO_NODE)

#define DELAY K_MSEC(10000)
#define MAX_DATA_LEN 10
char data_test[] =  { 0X00 , 0X01 ,
                      0X00 , 0X02 , 
					  0X00 , 0X00 , 0X00 , 0X01 ,
					  0X00 , 0X00 , 0X00 , 0X01 };


const struct device *lora_dev;
struct lorawan_join_config join_cfg;

uint8_t dev_eui[] = LORAWAN_DEV_EUI_HELIUM;
uint8_t join_eui[] = LORAWAN_JOIN_EUI_HELIUM;
uint8_t app_key[] = LORAWAN_APP_KEY_HELIUM;

//LORAWAN DOWNLINK FIFO

static K_FIFO_DEFINE(my_fifo_downlink);
struct _Downlink_Fifo downlink_cmd;
struct _Downlink_ downlink_cmd_new;

// DOWNLINK CHOOSE FIRST AND PORT2

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

	ARG_UNUSED(dev);
	struct uart_data_t *buf;
	uint8_t i = 0;
    static bool disable_req;

	switch (evt->type)
	{

    case UART_RX_BUF_REQUEST:
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}
		break;


	case UART_RX_RDY:
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

      	if (disable_req) {
			return;
		}  
        /*
        if (buf->data[buf->len - 1] == 0x24) {   //0x24=$
                buf->data[buf->len-1] = 0x00;  //replace the character $ by 0x00
                uart_rx_disable(uart);
        }
        */
        if (buf->data[buf->len - 1] == 0x0a) {   //0x24=$
                buf->data[buf->len-2] = 0x2c;  //replace the character 0x0d by ,
                buf->data[buf->len-1] = 0x00;  //replace the character 0x0a by 0x00
            
                uart_rx_disable(uart);
        }



        blink(3);
		break;

	case UART_RX_BUF_RELEASED:

		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data);

		if (buf->len > 0) {
            k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

        break;

	case UART_RX_DISABLED:
        disable_req = false;
		// blink(LED4,4);
		buf = k_malloc(sizeof(*buf)); 
		if (buf){
			buf->len = 0;
		 }else{
			  k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			  return;
		    }

		buf->len = 0;
		uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
		break;
	}

}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;
	buf = k_malloc(sizeof(*buf)); // SIZE IS 92 BYTES
	if (buf)
	{
		buf->len = 0;
	}
	else
	{
		LOG_WRN("Not able to allocate UART receive buffer - GPS\n");
        
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
    
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		//printk("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		//printk("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

    
	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
    
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

//UART_FUNCTIONS END

static void dl_callback(uint8_t port, bool data_pending,int16_t rssi, int8_t snr, uint8_t len, const uint8_t *data)	{
    
 
    //printk("Port %d, Pending %d, RSSI %ddB, SNR %ddBm \n", port, data_pending, rssi, snr);
    uint8_t i=0;
    if (data) {
        //printk(data, len, "Payload: \n");
	    downlink_cmd_new.port=port;
        downlink_cmd_new.rssi=rssi;
		downlink_cmd_new.snr=snr;
        downlink_cmd_new.len = len;
        while (i < len) {
            downlink_cmd_new.data[i] = data[i];
            i++;
        }
		k_sem_give(&lorawan_rx);//downlink
    }

    
}

struct lorawan_downlink_cb downlink_cb = {
	   .port = LW_RECV_PORT_ANY,
	   .cb = dl_callback
    };

static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	color(10);
	LOG_INF("New Datarate: DR_%d, Max Payload %d \n", dr, max_size);
	color(255);
}

//END_LORAWAN

void configure_digital_outputs(void)
{
	gpio_pin_configure_dt(&led_0, GPIO_OUTPUT);
	//LOG_INF("Set up Digital Output at %s pin %d\n", LED_OUT_0.port->name, LED_OUT_0.pin);
    gpio_pin_set_dt(&led_0, OFF);

	gpio_pin_configure_dt(&digital_dig0, GPIO_OUTPUT);
	//LOG_INF("Set up Digital Output at %s pin %d\n", DIG_OUT_0.port->name, DIG_OUT_0.pin);
    gpio_pin_set_dt(&digital_dig0, OFF); //reset is a off pulse, then is inverted

	gpio_pin_configure_dt(&digital_dig1, GPIO_OUTPUT);
	//LOG_INF("Set up Digital Output at %s pin %d\n", DIG_OUT_1.port->name, DIG_OUT_1.pin);
    gpio_pin_set_dt(&digital_dig1, OFF);

    gpio_pin_configure_dt(&digital_dig2, GPIO_OUTPUT);
	//LOG_INF("Set up Digital Output at %s pin %d\n", DIG_OUT_2.port->name, DIG_OUT_2.pin);
    gpio_pin_set_dt(&digital_dig2, OFF);

    gpio_pin_configure_dt(&digital_dig3, GPIO_OUTPUT);
	//LOG_INF("Set up Digital Output at %s pin %d\n", DIG_OUT_3.port->name, DIG_OUT_3.pin);
    gpio_pin_set_dt(&digital_dig3, OFF);


}

void blink(uint8_t times){
    uint8_t i=0;
    while (i<times){
        gpio_pin_set_dt(&digital_dig3, ON);
        gpio_pin_set_dt(&digital_dig3, OFF);
        i++;
    }

}

void led_on_off(uint8_t status)
{
   gpio_pin_set_dt(&led_0, status);
	
}


#define FFT_SIZE 2048 //was 2048
/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
static float32_t testInput_f32_10khz[FFT_SIZE];
static float32_t testOutput[FFT_SIZE];


/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = FFT_SIZE;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;


/*
#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3
#include <nrfx_log.h>
*/
/**
 * @defgroup nrfx_saadc_advanced_non_blocking_internal_timer_example Advanced non-blocking sampling with internal timer SAADC example
 * @{
 * @ingroup nrfx_saadc_examples
 *
 * @brief Example showing advanced functionality of nrfx_saadc driver operating in the non-blocking continuous sampling mode with internal timer.
 *
 * @details Application initializes nrfx_saadc driver and starts operating in the non-blocking mode.
 *          In the example, @ref m_single_channel is configured and the SAADC driver is set to the advanced mode
 *          with @p nrfx_saadc_adv_config_t::start_on_end enabled. With that flag @p NRF_SAADC_TASK_START task
 *          is triggered on the @p NRF_SAADC_EVENT_END event automatically by the SAADC driver.
 *
 *          Calibration in a non-blocking manner is triggered by @p nrfx_saadc_offset_calibrate. Then, upon receiving @p NRFX_SAADC_EVT_CALIBRATEDONE
 *          event in @ref saadc_handler() sampling is initiated by calling @p nrfx_saadc_mode_trigger() function.
 *          Consecutive sample tasks are triggered by the internal timer at the sample rate specified in  @ref SAADC_SAMPLE_FREQUENCY symbol.
 *
 *          The example features GPPI channel configured to test the functionality of SAADC. The endpoints are setup up in a way
 *          that @p NRF_SAADC_EVENT_RESULTDONE event is connected with the GPIOTE task that toggles the @ref OUT_GPIO_PIN pin.
 *
 *          Please note that the internal timer can only be used in the non-blocking mode with only a single input channel enabled.
 */

/** @brief Symbol specifying analog input to be observed by SAADC channel 0. */
#define CH0_AIN ANALOG_INPUT_TO_SAADC_AIN(ANALOG_INPUT_A0)

/** @brief Symbol specifying GPIO pin used to test the functionality of SAADC. */
#define OUT_GPIO_PIN LOOPBACK_PIN_1B

/** @brief Acquisition time [us] for source resistance <= 10 kOhm (see SAADC electrical specification). */
#define ACQ_TIME_10K 3UL

/** @brief Conversion time [us] (see SAADC electrical specification). */
#define CONV_TIME 2UL

/** @brief Internal timer frequency [Hz] is derived from PCLK16M (see SAMPLERATE register in SAADC). */
#define INTERNAL_TIMER_FREQ 16000000UL

/** @brief SAADC sample frequency for the continuous sampling. */
#define SAADC_SAMPLE_FREQUENCY 48000UL  // was 36000

/** @brief Internal timer capture and compare value. */
#define INTERNAL_TIMER_CC (INTERNAL_TIMER_FREQ / SAADC_SAMPLE_FREQUENCY)

/**
 * @brief Symbol specifying the number of sample buffers ( @ref m_sample_buffers ).
 * Two buffers are required for performing double-buffered conversions.
 */
#define BUFFER_COUNT 2UL

/** @brief Symbol specifying the size of singular sample buffer ( @ref m_sample_buffers ). */
#define BUFFER_SIZE 3840UL //was 2880

/** @brief Symbol specifying the number of SAADC samplings to trigger. */
#define SAMPLING_ITERATIONS 1UL

/** @brief SAADC channel configuration structure for single channel use. */


static const nrfx_saadc_channel_t m_single_channel_0 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput0,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};
static const nrfx_saadc_channel_t m_single_channel_1 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput1,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};
static const nrfx_saadc_channel_t m_single_channel_2 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput2,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};
static const nrfx_saadc_channel_t m_single_channel_3 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput3,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};
static const nrfx_saadc_channel_t m_single_channel_4 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput4,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};
static const nrfx_saadc_channel_t m_single_channel_5 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput5,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};
static const nrfx_saadc_channel_t m_single_channel_6 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput6,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};
static const nrfx_saadc_channel_t m_single_channel_7 ={                                                       
    .channel_config =                                   
    {                                                   
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      
        .gain       = NRF_SAADC_GAIN1_6,                
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,     
        .acq_time   = NRF_SAADC_ACQTIME_3US,            
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      
        .burst      = NRF_SAADC_BURST_DISABLED,         
    },                                                  
    .pin_p          = SAADC_CH_PSELP_PSELP_AnalogInput7,        
    .pin_n          = NRF_SAADC_INPUT_DISABLED,         
    .channel_index  = 0,                           
};

/** @brief Samples buffer to store values from a single channel ( @ref m_single_channel). */
static nrf_saadc_value_t m_sample_buffers[BUFFER_COUNT][BUFFER_SIZE];

/** @brief Flag indicating that sampling on every specified channel is finished and buffer ( @ref m_sample_buffers ) is filled with samples. */
static bool m_saadc_done;

/** For continuous sampling the sample rate SAADC_SAMPLE_FREQUENCY should fulfill the following criteria (see SAADC Continuous sampling). */
NRFX_STATIC_ASSERT(SAADC_SAMPLE_FREQUENCY <= (1000000UL / (ACQ_TIME_10K + CONV_TIME)));

/** Possible range value of a CC field in the SAMPLERATE register (SAADC) is 80-2047. */
NRFX_STATIC_ASSERT((INTERNAL_TIMER_CC >= 80UL ) && (INTERNAL_TIMER_CC <= 2047UL));


uint16_t sample[BUFFER_SIZE];
static K_SEM_DEFINE(adc_values_ready, 0, 1);
static K_SEM_DEFINE(req_adc_values, 0, 1);


/*FILTER FIR*/
 #define TEST_LENGTH_SAMPLES  FFT_SIZE
 #define SNR_THRESHOLD_F32    140.0f
 #define BLOCK_SIZE            32
 #define NUM_TAPS              29
 
 float32_t refOutput[TEST_LENGTH_SAMPLES];
  
/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
  static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */
  const float32_t firCoeffs32[NUM_TAPS] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
  };
/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */
  uint32_t blockSize = BLOCK_SIZE;
  uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;
  float32_t  snr;
/*FILTER FIR END*/




#define DEBUG_PRINT 0

/**
 * @brief Function for handling SAADC driver events.
 *
 * @param[in] p_event Pointer to an SAADC driver event.
 */
static void saadc_handler(nrfx_saadc_evt_t const * p_event){
    nrfx_err_t status;
    (void)status;

    static uint16_t buffer_index = 1;
    static uint16_t buf_req_evt_counter;
    uint16_t samples_number;

    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_CALIBRATEDONE:
            if (DEBUG_PRINT)printf("SAADC event: CALIBRATEDONE\n");

            status = nrfx_saadc_mode_trigger();
            NRFX_ASSERT(status == NRFX_SUCCESS);
            break;

        case NRFX_SAADC_EVT_READY:
            if (DEBUG_PRINT)printf("SAADC event: READY\n");
            break;

        case NRFX_SAADC_EVT_BUF_REQ:
            if (DEBUG_PRINT)printf("SAADC event: BUF_REQ\n");
            
            if (++buf_req_evt_counter < SAMPLING_ITERATIONS)
            {
                // Next available buffer must be set on the NRFX_SAADC_EVT_BUF_REQ event to achieve the continuous conversion. 
                status = nrfx_saadc_buffer_set(m_sample_buffers[buffer_index++], BUFFER_SIZE);
                NRFX_ASSERT(status == NRFX_SUCCESS);
                buffer_index = buffer_index % BUFFER_COUNT;
            }
            break;
            
            
                
                status = nrfx_saadc_buffer_set(m_sample_buffers[buffer_index++], BUFFER_SIZE);
                NRFX_ASSERT(status == NRFX_SUCCESS);
                buffer_index = buffer_index % BUFFER_COUNT;
            
            break;
            


        case NRFX_SAADC_EVT_DONE:
            if (DEBUG_PRINT)printf("SAADC event: DONE\n");
            if (DEBUG_PRINT)printf("Sample buffer address == %p\n", p_event->data.done.p_buffer);

            samples_number = p_event->data.done.size;
            if(k_sem_count_get(&req_adc_values)){
               k_sem_take(&req_adc_values,K_NO_WAIT);
              
             for (uint16_t i = 0; i < samples_number; i++)
               {
                //printf("[Sample %u] value == %d\n", i, p_event->data.done.p_buffer[i]);
                //printf("%d\n",p_event->data.done.p_buffer[i]);
                sample[i]=p_event->data.done.p_buffer[i];
                //printf("[Sample %u] value == %d\n", i, sample[i]);
               }

              k_sem_give(&adc_values_ready);
            }
            
              
             
            break;

        case NRFX_SAADC_EVT_FINISHED:
              if (DEBUG_PRINT)printf("SAADC event: FINISHED\n");
              
              
              status = nrfx_saadc_buffer_set(m_sample_buffers[buffer_index++], BUFFER_SIZE);
              NRFX_ASSERT(status == NRFX_SUCCESS);
              buffer_index = buffer_index % BUFFER_COUNT;
              
              // Inicia a próxima conversão
              status = nrfx_saadc_mode_trigger();
              NRFX_ASSERT(status == NRFX_SUCCESS);

            break;

        default:
            break;
    }
}

/**
 * @brief Function for application main entry.
 *
 * @return Nothing.
 */


float arm_snr_f32_x(float *pRef, float *pTest, uint32_t buffSize)
{
  float EnergySignal = 0.0, EnergyError = 0.0;
  uint32_t i;
  float SNR;
  int temp;
  int *test;

  for (i = 0; i < buffSize; i++)
    {
 	  /* Checking for a NAN value in pRef array */
	  test =   (int *)(&pRef[i]);
      temp =  *test;

	  if(temp == 0x7FC00000)
	  {
	  		return(0);
	  }

	  /* Checking for a NAN value in pTest array */
	  test =   (int *)(&pTest[i]);
      temp =  *test;

	  if(temp == 0x7FC00000)
	  {
	  		return(0);
	  }
      EnergySignal += pRef[i] * pRef[i];
      EnergyError += (pRef[i] - pTest[i]) * (pRef[i] - pTest[i]); 
    }

	/* Checking for a NAN value in EnergyError */
	test =   (int *)(&EnergyError);
    temp =  *test;

    if(temp == 0x7FC00000)
    {
  		return(0);
    }
	

  SNR = 10 * log10 (EnergySignal / EnergyError);

  return (SNR);

}

int filter(void){
  //https://github.com/ARM-software/CMSIS_4/tree/master/CMSIS/DSP_Lib/Examples/arm_fir_example/ARM

  uint32_t i;
  arm_fir_instance_f32 S;
  arm_status status;
  float32_t  *inputF32, *outputF32;
  
  inputF32 = &testInput_f32_10khz[0];
  outputF32 = &testOutput[0];
  
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
 
  for(i=0; i < numBlocks; i++)
  {
    arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
  }
  
  snr = arm_snr_f32(&refOutput[0], &testOutput[0], TEST_LENGTH_SAMPLES);
  
  if (snr < SNR_THRESHOLD_F32)
  {
    status = ARM_MATH_TEST_FAILURE;
  }
  else
  {
    status = ARM_MATH_SUCCESS;
  }
  
 return status;
}

void fft() {
    // Preencha input_buffer com os dados do sinal que você deseja analisar
  arm_status status;

   /*
   18khz = 720 amostras
   taxa fft = 1024
   bin= 17.5hz/bin
   */
float32_t inputArray[10] = {1.5, 2.0, 5.3, 3.1, 6.7, 4.2, 9.8, 1.0, 7.2, 8.5};

  status = ARM_MATH_SUCCESS;
  /* Process the data through the CFFT/CIFFT module */
  arm_cfft_f32(&arm_cfft_sR_f32_len1024, &testInput_f32_10khz, ifftFlag, doBitReverse);
  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  arm_cmplx_mag_f32(&testInput_f32_10khz, &testOutput, fftSize);
 
}

void memory_init(void){
    	/* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at NVS_PARTITION_OFFSET
	 */
	fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		//LOG_INF("Flash device %s is not ready\n", fs.flash_device->name);
		return 0;
	}
	fs.offset = NVS_PARTITION_OFFSET;
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		//LOG_INF("Unable to get page info\n");
		return 0;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	rc = nvs_mount(&fs);
	if (rc) {
		//LOG_INF("Flash Init failed\n");
		return 0;
	}

}

void reboot_counter_read(void){

  rc = nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
 	if (rc > 0) { /* item was found, show it */
 		 //LOG_INF("Id: %d, Reboot_counter: %d\n",RBT_CNT_ID, reboot_counter);
	 } else   {/* item was not found, add it */
		 //LOG_INF("No Reboot counter found, adding it at id %d\n",RBT_CNT_ID);
		(void)nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	}
  dev_nonce=reboot_counter;
  reboot_counter++;
  (void)nvs_write(&fs, RBT_CNT_ID, &reboot_counter,sizeof(reboot_counter));  

}

#ifdef NORMAL_STARTUP
int main(){
  uint8_t mensagem[]="ABCDEF";  
  memory_init();
  reboot_counter_read();
  configure_digital_outputs();
  k_fifo_init(&fifo_uart_rx_data);

  uart_init();
  uint32_t counter=0;
  led_on_off(1);
  //LOG_INF("Start - turn on the UART \n");
  
  k_sem_give(&gps_init);
  k_msleep(2000);
  led_on_off(0);


  //LOG_INF("Start - turn on SX1276 \n");
  setup_initialize();
  
  k_sem_give(&lorawan_init);  //START HELIUM JOIN
  k_sem_give(&timer_init);

  
   while (1)
    {
        k_msleep(5000);
        LOG_INF("Working...%ld   \n",counter);
        //printf("%s\n",&mensagem);
        counter++;
        //NRFX_EXAMPLE_LOG_PROCESS();
    }
}

#else
int main(){
  configure_digital_outputs();
  uint8_t status=0;
  led_on_off(1);
  LOG_INF("Test Start \n");
  k_msleep(2000);
  led_on_off(0);

   while (1)
    {
        status=0;
        gpio_pin_set_dt(&digital_dig0, status);
        gpio_pin_set_dt(&digital_dig1, status);
        gpio_pin_set_dt(&digital_dig2, status);
        led_on_off(status);
        k_msleep(200);
        status=255;
        gpio_pin_set_dt(&digital_dig0, status);
        gpio_pin_set_dt(&digital_dig1, status);
        gpio_pin_set_dt(&digital_dig2, status);
        led_on_off(status);
        k_msleep(200);
        


    }


}

#endif

void conversion_change_channel(uint8_t new_channel){
    nrfx_err_t status;
    (void)status;
    nrfx_saadc_uninit();
    //printf("NRFX STOPPED\n");
    
    init_conversion(0);
}

void init_conversion(uint8_t channel){
    nrfx_err_t status;
    (void)status;
  

    //NRFX_EXAMPLE_LOG_INIT();
    //printf("Starting nrfx_saadc advanced non-blocking sampling with internal timer example\n");
    //NRFX_EXAMPLE_LOG_PROCESS();

    status = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    NRFX_ASSERT(status == NRFX_SUCCESS);


    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0);


    if(channel==0)status = nrfx_saadc_channel_config(&m_single_channel_0);
    if(channel==1)status = nrfx_saadc_channel_config(&m_single_channel_1);
    if(channel==2)status = nrfx_saadc_channel_config(&m_single_channel_2);
    if(channel==3)status = nrfx_saadc_channel_config(&m_single_channel_3);
    if(channel==4)status = nrfx_saadc_channel_config(&m_single_channel_4);
    if(channel==5)status = nrfx_saadc_channel_config(&m_single_channel_5);
    if(channel==6)status = nrfx_saadc_channel_config(&m_single_channel_6);
    if(channel==7)status = nrfx_saadc_channel_config(&m_single_channel_7);


    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    adv_config.internal_timer_cc = INTERNAL_TIMER_CC;
    adv_config.start_on_end = true;

    uint32_t channel_mask = nrfx_saadc_channels_configured_get();
    status = nrfx_saadc_advanced_mode_set(channel_mask,
                                          NRF_SAADC_RESOLUTION_14BIT,
                                          &adv_config,
                                          saadc_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    status = nrfx_saadc_buffer_set(m_sample_buffers[0], BUFFER_SIZE);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    //pin_on_event_toggle_setup(OUT_GPIO_PIN,nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_RESULTDONE));

    m_saadc_done = true;
    status = nrfx_saadc_offset_calibrate(saadc_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);
      
    
}

void imprimir_grafico(float *dados, int tamanho_dados, uint16_t tempo_configuravel,float valor_maximo) {
    int linhas = 25; // Número de linhas no eixo Y
    int colunas = 130; // Número de colunas no eixo X
    

    int i, j;

    for (i = linhas - 1; i >= 0; i--) {
        float linha_atual = i * valor_maximo / linhas;
        printf("|");
        for (j = 0; j < colunas; j++) {
            int indice_dados = (j * tamanho_dados) / colunas;
            if (dados[indice_dados] >= linha_atual) {
                printf("*");
            } else {
                printf(" ");
            }
            k_usleep(tempo_configuravel);
        }
        printf("\n");
    }

    // Imprimir eixo X com valores
    printf("+");
    for (i = 0; i < colunas; i++) {
        if (i % (colunas / 10) == 0) {
            printf("+");
        } else {
            printf("-");
        }
        k_usleep(tempo_configuravel);
    }
    printf("+\n");

    // Imprimir rótulos do eixo X com valores
    printf(" ");
    for (i = 0; i < colunas; i++) {
        if (i % (colunas / 10) == 0) {
            int valor = (i * tamanho_dados) / colunas; // Calcula o valor correspondente
            printf("%d", valor);
        } else {
            printf(" ");
        }
        k_usleep(tempo_configuravel);
    }
    printf("\n");
}

void adc_thread(void)
{
    nrfx_err_t status;
    (void)status;
    float32_t maxValue=0;
    float frequency=0;

    printf("ADC Thread Init\n");
    init_conversion(0);
    k_sem_give(&req_adc_values);
    while(1){
           
           k_sem_take(&adc_values_ready,K_FOREVER);
            
            for (uint16_t i = 0; i < FFT_SIZE; i++){
                testInput_f32_10khz[i]=((0.00021973*sample[i])*1000)-1485;
                //printf("%03.1f ",i,testInput_f32_10khz[i]);
                
                //k_msleep(1);
             }
            
             k_msleep(20);
             
             //printf("filter result:%d\n",filter());
           
             fft();
              printf("Result FFT\n");

             for (uint16_t i = 0; i < 128; i++){
                //printf("%3.1f \n",i,testOutput[i]);
                //k_msleep(1);
             }
             printf(" \n");
             testOutput[0]=0;

             arm_max_f32(&testOutput, 128, &maxValue, &testIndex);
             imprimir_grafico(testOutput, 64,00,maxValue);

             frequency=23.33*testIndex;
             printf("Max Value=%5.0f bin=%u freq_bin=%3.0fHz\n",maxValue,testIndex,frequency);

            k_msleep(1000);
            conversion_change_channel(0);
             printf("WAIT 3 SECONDS \n");
            k_msleep(3000);
             printf("START AD READING \n");
            k_sem_give(&req_adc_values);

            
            
    }
}

/*

void gnss_write_thread_old(void){
    uint8_t debug = ON;
	uint8_t value;
	uint32_t i = 0, j = 1, k = 0, h = 0, g = 0, index = 0, bfcnt = 0;
	uint64_t time = k_uptime_get();
	uint8_t state = 0, pkt_init = 0;
	static uint8_t buffer[BUFF_SIZE];

    //http://aprs.gids.nl/nmea/     sentences descriptions
    //const char nmea_id[10] = "$GPGGA"; //capture this sentence
    const char nmea_id[10] = "$GPRMC"; //capture this sentence
	

    static char *field[20];
    char *ret;
    char *token;
    char marker[2]="\n";
    //marker[0]=0x0d;

	uint8_t part[2];

	while (i < BUFF_SIZE)buffer[i++] = 0x20;//space
	i = 0;

	struct uart_data_t *buf2a;
	buf2a = k_malloc(sizeof(*buf2a));
	//

	k_sem_take(&gps_init,K_FOREVER); 

	for (;;)
	{
		
		buf2a = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
		k_fifo_init(&fifo_uart_rx_data);

		if (buf2a->len > 0)
		{
			k = (buf2a->len);

			i = 0;
			index = 0;
			//blink(LED4,2);
           
			while (i < k && pkt_init == 0)
			{
				// printf("%02X ",buf2a->data[i]);
				switch (buf2a->data[i])
				{
				case 0x24: //$
					if (state == 0)state = 1;
					break;
				case 0x47: // G
					if (state == 1)state = 2;

					break;
				case 0x50: // P
					if (state == 2)state = 3;

					break;
				case 0x52: // R
					if (state == 3)state = 4;

					break;
				case 0x4D: // M
					if (state == 4)state = 5;

					break;
				case 0x43: // C
					if (state == 5){
						state = 6;
					    index = i - 5;
					}
					break;
				}
				i++;
			}

			if (state == 6 && pkt_init == 0)
			{
				//printf("BEGIN:\n");
				while (index < k)
				{
					//printf("%c", buf2a->data[index]);
					if (buf2a->data[index]!=0x0D) {
						buffer[bfcnt] = buf2a->data[index];
						bfcnt++;
					}
					index++;
				}
				pkt_init=1;
			}
		}

		buf2a = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
		if (buf2a->len > 0)
		{
			if ((pkt_init >= 1) && (bfcnt < BUFF_SIZE)  )
			{
				index = 0;
				while ((index < k)  && (bfcnt < BUFF_SIZE))
				{
					if (buf2a->data[index]!=0x0D) {
						buffer[bfcnt] = buf2a->data[index];
						bfcnt++;
					}
					index++;
				}
				pkt_init++;
			}

			if (bfcnt >= BUFF_SIZE - 1)
			{
                index = 0;
                while(index < bfcnt ){
					//printf("%c",buffer[index]);
				   	index++;
				}
   				ret = strstr(buffer, nmea_id);
   				//printf("The substring is: %s\n", ret);
   				token = strtok(ret, marker);
   				//printf("%s\n", token );
   				i=parse_comma_delimited_str(token, field, 20);
                //debug_print_fields(i,field);
				
				if (i==12){
				  //printf("\nGPS Fixed  :%s\r\n",field[2]); //(0=invalid; 1=GPS fix; 2=Diff. GPS fix)
				  
				  position.gps_fixed=*field[2]-0x40; //char A=0x41 - 0x40 = 1
				  //printf("inteiro %d\n",position.gps_fixed);
				  if (position.gps_fixed==1){  
                   LOG_INF("GPS Fixed  :Yes\n");
				   //printf("Time       :%s\r\n",field[1]);
				   //printf("Date       :%s\r\n",field[9]);
                   //printf("Latitude  N:%s\r\n",field[3]);
                   //printf("Longitude E:%s\r\n",field[5]);
				   position.latitude=atof(field[3]);
				   position.longitude=atof(field[5]);
				   fill_date(field[1],field[9]);
				  }else LOG_INF("Not Fixed yet\n");
			    }
				index = 0;
				pkt_init = 0;
				bfcnt = 0;
				state = 0;
				while (index < BUFF_SIZE)buffer[index++] = 0x20;//space
				index=0;
			}
			// printf("j:%d\n",j);
			j++;
		}
	}
}
*/

void gnss_write_thread(void){
    uint8_t debug = ON;
	uint8_t value;
	uint32_t i = 0, j = 1, k = 0, h = 0, g = 0, index = 0, bfcnt = 0;
	uint64_t time = k_uptime_get();
	uint8_t state = 0, pkt_init = 0;
	static uint8_t buffer[BUFF_SIZE];

    //http://aprs.gids.nl/nmea/     sentences descriptions
    //const char nmea_id[10] = "$GPGGA"; //capture this sentence
    const char nmea_id[10] = "$GPRMC"; //capture this sentence
	

    static char *field[20];
    char *ret;
    char marker[2]="\n";
    //marker[0]=0x0d;

	uint8_t part[2];

    // variaveis

 
   //char str[] = "$GNRMC,193326.076,V,3150.7822,N,11711.9278,E,1.00,2.00,040724,CA1,CA2,N,V*21";
   char str[] = "$GNRMC,193326.076,V,,,,,0.00,0.00,040724,,,N,V*21";


   #define FIELD_SIZE 50
   #define FIELD_SIZE_SMALL 5
    char message_id[FIELD_SIZE];
    char utc_time[FIELD_SIZE];
    char status[FIELD_SIZE];
    float latitude;
    char ns_indicator[FIELD_SIZE];
    float longitude;
    char ew_indicator[FIELD_SIZE];
    float speed_over_ground;
    float course_over_ground;
    char date[FIELD_SIZE];
    char magnetic_variation[FIELD_SIZE];
    char mode[FIELD_SIZE];
    char fix[FIELD_SIZE];
    char checksum[FIELD_SIZE];
    
    // Ponteiro para os tokens
    char *token;
    token = k_malloc(100);
    // Delimitador
    const char delimiter[2] = ",";
	struct uart_data_t *buf2a;
	buf2a = k_malloc(sizeof(*buf2a));

	k_sem_take(&gps_init,K_FOREVER);

    struct uart_data_t *initial_buf = k_malloc(sizeof(struct uart_data_t));
    if (!initial_buf) {
        printk("Erro ao alocar memória para buffer inicial\n");
        return;
    }
    initial_buf->len = 0;

    printf("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\n");

    while (1) {
        struct uart_data_t *buf2a = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
        if (buf2a) {
            blink(4);
            if (buf2a->len > 0) {
                //printf("length: %u %s\n", buf2a->len, buf2a->data);

                //printf("%.*s\n",buf2a->len,buf2a->data);
                
                blink(5);
            }
            k_free(buf2a);
        }
    }






    /*
	while(1){

	   buf2a = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
       blink(4);
       
 
	   state = 0;
       if (buf2a->len > 0){
          printk("%s", buf2a->data);
       }

      
		if (buf2a->len > 0)
		{
			k = (buf2a->len);

			i = 0;
			index = 0;
			//blink(LED4,2);
           
			while (i < k && pkt_init == 0)
			{
				// printf("%02X ",buf2a->data[i]);
				switch (buf2a->data[i])
				{
				case 0x24: //$
					state = 1;
					break;
				case 0x47: // G
					if (state == 1)state = 2;
					break;
				case 0x4E: // N
					if (state == 2)state = 3;

					break;
				case 0x52: // R
					if (state == 3)state = 4;

					break;
				case 0x4D: // M
					if (state == 4)state = 5;

					break;
				case 0x43: // C
					if (state == 5){
						state = 6;
					    index = i - 5;
					}
					break;
				}
				i++;
                
			}

			if (state == 6 && pkt_init == 0){
                printk("%s", buf2a->data);
                state = 0;
                strcpy(token,' ');
                token = strtok(buf2a->data, delimiter);
                strcpy(message_id, token);
                token = strtok(NULL, delimiter);
                strcpy(utc_time, token);
                token = strtok(NULL, delimiter);
                strcpy(status, token);
                token = strtok(NULL, delimiter);
                latitude = atof(token);
                token = strtok(NULL, delimiter);
                strcpy(ns_indicator, token);
                token = strtok(NULL, delimiter);
                longitude = atof(token);
                token = strtok(NULL, delimiter);
                strcpy(ew_indicator, token);
                token = strtok(NULL, delimiter);
                speed_over_ground = atof(token);
                token = strtok(NULL, delimiter);
                course_over_ground = atof(token);
                token = strtok(NULL, delimiter);
                strcpy(date, token);
                token = strtok(NULL, delimiter);
                strcpy(magnetic_variation, token);
                token = strtok(NULL, delimiter);
                strcpy(mode, token);
                token = strtok(NULL, delimiter);
                strcpy(fix, token);
                token = strtok(NULL, delimiter);
                strcpy(checksum, token);

                
                
                printk("Message ID: %s\n", message_id);
                printk("UTC time: %s\n", utc_time);
                printk("Status: %s\n", status);
                printk("Latitude: %f\n", latitude);
                printk("N/S indicator: %s\n", ns_indicator);
                printk("Longitude: %f\n", longitude);
                printk("E/W indicator: %s\n", ew_indicator);
                printk("Speed over ground: %f\n", speed_over_ground);
                printk("Course over ground: %f\n", course_over_ground);
                printk("Date: %s\n", date);
                printk("Magnetic Variation: %s\n", magnetic_variation);
                printk("Mode: %s\n", mode);
                printk("Fix: %s\n", fix);
                
                printk("Checksum: %s\n", checksum);
                
                k_sleep(K_MSEC(5000));
                


			}
		}
        
			

	}
    */
}

void downlink_thread(void){
    uint8_t cmd=0;
	while(1){
	  k_sem_take(&lorawan_rx,K_FOREVER);
      color(4);
      //NAO PODE USAR LOG_INF AQUI
	  printk("CMD-Received\n");
	  printk("Len: %d\n",downlink_cmd_new.len);
	  printk("Port %d, RSSI %ddB, SNR %ddBm \n", downlink_cmd_new.port, downlink_cmd_new.rssi, downlink_cmd_new.snr);
	  printk(downlink_cmd_new.data, downlink_cmd_new.len, "Payload: \n");

	  printk("%X:%X:%X\n",downlink_cmd_new.data[0],downlink_cmd_new.data[1],downlink_cmd_new.data[2]);
      static uint8_t *data=downlink_cmd_new.data;
	  

	  (void)cmd_interpreter(data,downlink_cmd_new.len);
      color(0);
	}
    
}

void shoot_minute_save_thread(void)
{

	// each one minute this thread will shot.
	uint64_t actual_time = k_uptime_get() / 1000;
	signed int h, m, s, last_minute;
	h = (actual_time / 3600);
	m = (actual_time - (3600 * h)) / 60;
	s = (actual_time - (3600 * h) - (m * 60));
	last_minute = m;

	// time_print ();
    k_sem_take(&timer_init,K_FOREVER); //wait init

	while (1)
	{
		actual_time = k_uptime_get() / 1000;
		h = (actual_time / 3600);
		m = (actual_time - (3600 * h)) / 60;
		s = (actual_time - (3600 * h) - (m * 60));

		if (m == (last_minute + 1))
		{
			last_minute = m;
			if (m == 59)
			{
				last_minute = -1;
			}
			if (h == 24)
			{
				h = 0;
			} // only up to 23:59:59h
			  // START RUN THE MINUTE ROUTINE
			
            			
			
			if (lora_cycle_minute>=Initial_Setup.interval_uplink){
				k_sem_give(&lorawan_tx);
				lora_cycle_minute=0;
			}
			lora_cycle_minute++;
			LOG_INF("Minute Cycle thread \n");
		}
		k_sleep(K_MSEC(100));
	}
}

void shoot_led_thread(void)
{
    while(1){

      if (led_status==OFF){
          led_status=ON;
          led_on_off(led_status);
          k_msleep(led_period);
      }else{
          led_status=OFF;
          led_on_off(led_status);
          k_msleep(led_period*led_period_off);
        } 
     }
} 

void lorawan_thread(void)
{
/*
https://community.st.com/t5/stm32-mcus-wireless/lorawan-device-join-process-can-succeed-only-follow-counter-dev/td-p/51393
https://www.thethingsnetwork.org/forum/t/lorawan-1-1-devnonce-must-be-stored-in-a-non-volatile-memory-on-end-device/48995

*/


	//THIS THREAD MUST HAVE MAXIMUM PRIORITY(-9) IN ORDER TO RECEIVE DOWNLINK CALL BACK
    uint64_t i=0,j=0;
	int ret;
    uint32_t random;
    
    lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora0));

	//LoRaMacTestSetDutyCycleOn(0);//disable dutyCycle for test

    k_sem_take(&lorawan_init, K_FOREVER);  // WAIT FOR INIT
	color(10);
    //printk("LoraWan Thread Started\n\n");
    color(255);
	if (!device_is_ready(lora_dev)) {
		//printk("%s: device not ready.\n\n", lora_dev->name);
		return;
	}
    lorawan_set_region(LORAWAN_REGION_EU868);
	
	lorawan_register_downlink_callback(&downlink_cb);
	lorawan_register_dr_changed_callback(lorwan_datarate_changed);
	lorawan_set_conf_msg_tries(20); //was 10
    
    //random = sys_rand32_get()+1;
    //dev_nonce = random & 0x0000FFFF;
    join_cfg.otaa.dev_nonce = dev_nonce+10;    

    while(1){
     ret=-1;

   	 while ( ret < 0 ) {
    	    color(10);
   	        LOG_INF("Joining network over OTAA\n\n");
			color(255);
            k_sleep(K_MSEC(3000));//was 1000
            lorawan_start();
			k_sleep(K_MSEC(2000));//was 500ms
		    lorawan_enable_adr( true );

            lorawan_set_datarate(LORAWAN_DR_4);
   
			join_cfg.mode = LORAWAN_CLASS_A; //was A
			join_cfg.dev_eui = dev_eui;
			join_cfg.otaa.join_eui = join_eui;
			join_cfg.otaa.app_key = app_key;
			join_cfg.otaa.nwk_key = app_key;
        
   		
		    ret = lorawan_join(&join_cfg);

            
			 if (ret<0){
                 led_period=LED_BLINK_FAST;
                 led_period_off=LED_OFF_MULTIPLIER;
				 color(10);
				 LOG_INF("Failed..Waiting some seconds to try join again ret=%d\n\n",ret);
                 join_cfg.otaa.dev_nonce=join_cfg.otaa.dev_nonce+INITIAL_DEV_NOUNCE;
				 color(255);
			     k_sleep(K_MSEC(60000));
	         }
			
    
      } 
	  color(10);
	  LOG_INF("Joined OTAA\n\n");
      led_period=LED_BLINK_SLOW;
      led_period_off=LED_OFF_MULTIPLIER_JOINED;
	  color(255);
	  Initial_Setup.joined=ON;
      for(int i=0;i<=15;i++){Initial_Setup.nwk_key[i]=join_cfg.otaa.nwk_key[i];}
      Initial_Setup.dev_nonce=join_cfg.otaa.dev_nonce;
	  print_setup();

	  	  
	  lorawan_reconnect=0;

      while (!lorawan_reconnect) {
		  k_sem_take(&lorawan_tx, K_FOREVER);
		  lorawan_tx_data();
	    }
    }
}


#ifdef NORMAL_STARTUP
K_THREAD_DEFINE(shoot_led_thread_id, 1024, shoot_led_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(shoot_minute_save_thread_id, 1024, shoot_minute_save_thread, NULL, NULL, NULL, 4, 0, 0);
//K_THREAD_DEFINE(adc_thread_id, 1024, adc_thread, NULL, NULL,NULL, 7, 0, 0);
K_THREAD_DEFINE(lorawan_thread_id, 4096, lorawan_thread, NULL, NULL, NULL, -9, 0, 0);
K_THREAD_DEFINE(downlink_thread_id, 4096, downlink_thread, NULL, NULL, NULL, 8, 0, 0);
K_THREAD_DEFINE(gnss_write_thread_id, 8000, gnss_write_thread, NULL, NULL, NULL, 5, 0, 0);

#endif
