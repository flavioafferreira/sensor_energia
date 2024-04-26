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

//PORT 1.06 UART TX
//PORT 1.04 UART RX
//PORT 0.05 LED

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
static void saadc_handler(nrfx_saadc_evt_t const * p_event)
{
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

int main(){


  printk("Start - turn on the UART \n");
  

   while (1)
    {
        k_msleep(1000);
        //printk("Working...\n");
        //NRFX_EXAMPLE_LOG_PROCESS();
    }

}


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

K_THREAD_DEFINE(adc_thread_id, 1024, adc_thread, NULL, NULL,NULL, 7, 0, 0);