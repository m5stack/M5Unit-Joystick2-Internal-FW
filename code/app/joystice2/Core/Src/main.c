/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "flash.h"
#include "ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ADC_Y1,
	ADC_X1,
  ADC_CHN_MAX,
} ADC1_CHN;

typedef struct{
  uint16_t x_negative_min_adc_16bits;
  uint16_t x_negative_max_adc_16bits;
  uint16_t x_positive_min_adc_16bits;
  uint16_t x_positive_max_adc_16bits;
  uint16_t y_negative_min_adc_16bits;
  uint16_t y_negative_max_adc_16bits;
  uint16_t y_positive_min_adc_16bits;
  uint16_t y_positive_max_adc_16bits;
}adc_cal_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS 0x63
#define FIRMWARE_VERSION 1
#define APPLICATION_ADDRESS     ((uint32_t)0x08001800)
#define BOOTLOADER_VER_ADDR ((uint32_t)0x08001800 - 1)
uint32_t bootloader_version;
#define FLASH_DATA_SIZE 32

#define ADC_CHANNEL_NUMS                ADC_CHN_MAX
#define ADC_SAMPLES_NUMS                ADC_CHANNEL_NUMS

#define RGB_BUFFER_SIZE 10

#define X_DEFAULT_MIN 0
#define X_DEFAULT_MAX 4095
#define Y_DEFAULT_MIN 0
#define Y_DEFAULT_MAX 4095
#define X_DEFAULT_MID 2101
#define Y_DEFAULT_MID 1935

#define UINT_MAX 8191
#define UINT_MIN 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t i2c_address[1] = {0};
__IO uint32_t uiAdcValueBuf[ADC_SAMPLES_NUMS];
__IO uint16_t usAdcValue16[ADC_CHANNEL_NUMS];
__IO uint8_t usAdcValue8[ADC_CHANNEL_NUMS];
__IO uint16_t calAdcValue16[ADC_CHANNEL_NUMS];
__IO uint8_t calAdcValue8[ADC_CHANNEL_NUMS];
__IO int16_t offsetAdcValue16[ADC_CHANNEL_NUMS];
__IO int8_t offsetAdcValue8[ADC_CHANNEL_NUMS];
__IO uint8_t fm_version = FIRMWARE_VERSION;
volatile uint32_t i2c_stop_timeout_delay = 0;

volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;

uint32_t rgb_color_buffer[RGB_BUFFER_SIZE] = {0};
uint32_t rgb_color_buffer_index = 0;
uint32_t lastest_rgb_color = 0;

uint8_t flash_data[FLASH_DATA_SIZE] = {0};

adc_cal_t adc_cal_data = {130, 33330, 34000, 65440, 130, 33330, 34000, 65440};

__IO int32_t calMidValueX;
__IO int32_t calMidValueY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//????????SRAM???
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //??? SRAM ??? 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  long result;

  result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min)
    result = out_min;
  else if (result > out_max)
    result = out_max;
  
  return result;
}

void i2c_port_set_to_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;

    flash_data[0] = i2c_address[0];
    memcpy(&flash_data[1], (uint8_t *)&adc_cal_data.x_negative_min_adc_16bits, 16);
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    memcpy((uint8_t *)&adc_cal_data.x_negative_min_adc_16bits, &flash_data[1], 16);
  }
}

void flash_data_write_back(void)
{
  if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
    flash_data[0] = i2c_address[0];
    memcpy(&flash_data[1], (uint8_t *)&adc_cal_data.x_negative_min_adc_16bits, 16);
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  }     
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  uint64_t adcTotal[ADC_CHANNEL_NUMS]={0};
  int16_t tmp = 0;
  
	HAL_ADC_Stop_DMA(hadc);
	// for (uint8_t i = 0; i < ADC_SAMPLES_NUMS; i++) {
  //   adcTotal[i%ADC_CHANNEL_NUMS] += uiAdcValueBuf[i];
  // }
	for (uint8_t i = 0; i < ADC_CHANNEL_NUMS; i++) {
    usAdcValue16[i] = uiAdcValueBuf[i];
    usAdcValue16[i] = 65535 - usAdcValue16[i];
    usAdcValue8[i] = map(usAdcValue16[i],0,65535,0,255); 
  }

  if (usAdcValue16[ADC_X1] >= adc_cal_data.x_positive_min_adc_16bits) {
    offsetAdcValue16[ADC_X1] = map(usAdcValue16[ADC_X1],adc_cal_data.x_positive_min_adc_16bits,
    adc_cal_data.x_positive_max_adc_16bits,0,4095);
    offsetAdcValue8[ADC_X1] = map(usAdcValue16[ADC_X1],adc_cal_data.x_positive_min_adc_16bits,
    adc_cal_data.x_positive_max_adc_16bits,0,127);
  } else if (usAdcValue16[ADC_X1] <= adc_cal_data.x_negative_max_adc_16bits) {
    offsetAdcValue16[ADC_X1] = map(usAdcValue16[ADC_X1],adc_cal_data.x_negative_min_adc_16bits,
    adc_cal_data.x_negative_max_adc_16bits,-4095,0);
    offsetAdcValue8[ADC_X1] = map(usAdcValue16[ADC_X1],adc_cal_data.x_negative_min_adc_16bits,
    adc_cal_data.x_negative_max_adc_16bits,-127,0);
  } else {
    offsetAdcValue16[ADC_X1] = 0;
    offsetAdcValue8[ADC_X1] = 0;
  }

  if (usAdcValue16[ADC_Y1] >= adc_cal_data.y_positive_min_adc_16bits) {
    offsetAdcValue16[ADC_Y1] = map(usAdcValue16[ADC_Y1],adc_cal_data.y_positive_min_adc_16bits,
    adc_cal_data.y_positive_max_adc_16bits,0,4095);
    offsetAdcValue8[ADC_Y1] = map(usAdcValue16[ADC_Y1],adc_cal_data.y_positive_min_adc_16bits,
    adc_cal_data.y_positive_max_adc_16bits,0,127);
  } else if (usAdcValue16[ADC_Y1] <= adc_cal_data.y_negative_max_adc_16bits) {
    offsetAdcValue16[ADC_Y1] = map(usAdcValue16[ADC_Y1],adc_cal_data.y_negative_min_adc_16bits,
    adc_cal_data.y_negative_max_adc_16bits,-4095,0);
    offsetAdcValue8[ADC_Y1] = map(usAdcValue16[ADC_Y1],adc_cal_data.y_negative_min_adc_16bits,
    adc_cal_data.y_negative_max_adc_16bits,-127,0);
  } else {
    offsetAdcValue16[ADC_Y1] = 0;
    offsetAdcValue8[ADC_Y1] = 0;
  }

	HAL_ADC_Start_DMA(hadc, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t rx_buf[16];
  uint8_t tx_buf[32];
  uint8_t rx_mark[16] = {0}; 

  if (len > 1) {
    if (rx_data[0] == 0xFF)
    {
      if (len == 2) {
        if (rx_data[1] < 128) {
          i2c_address[0] = rx_data[1];
          flash_data_write_back();
          user_i2c_init();
        }
      }
    }
    else if (rx_data[0] == 0xFD)
    {
      if (rx_data[1] == 1) {
        flag_jump_bootloader = 1;
        if (flag_jump_bootloader) {
          LL_I2C_DeInit(I2C1);
          LL_I2C_DisableAutoEndMode(I2C1);
          LL_I2C_Disable(I2C1);
          LL_I2C_DisableIT_ADDR(I2C1);
          HAL_ADC_DeInit(&hadc1);
          __HAL_RCC_DMA1_CLK_DISABLE();
          HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
          HAL_ADC_Stop_DMA(&hadc1);
          i2c_port_set_to_input();
          while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
          {
            jump_bootloader_timeout++;
            if (jump_bootloader_timeout >= 60000) {
              flag_jump_bootloader = 0;
              break;
            }
          }
          if (jump_bootloader_timeout < 60000) {
            NVIC_SystemReset();
          } else {
            // NVIC_SystemReset();
            MX_DMA_Init();
            MX_ADC1_Init();          
            user_i2c_init();
            HAL_ADCEx_Calibration_Start(&hadc1);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);
            i2c1_it_enable();        
            jump_bootloader_timeout = 0;
          }
        }        
      }
    }      
    else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x33))
    {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x30+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x30+i] = 1;     
      }      
      if (rx_mark[0]) {
        rgb_color_buffer[rgb_color_buffer_index] &= ~0x000000ff;
        rgb_color_buffer[rgb_color_buffer_index] |= rx_buf[0];
      }
      if (rx_mark[1]) {
        rgb_color_buffer[rgb_color_buffer_index] &= ~0x0000ff00;
        rgb_color_buffer[rgb_color_buffer_index] |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        rgb_color_buffer[rgb_color_buffer_index] &= ~0x00ff0000;
        rgb_color_buffer[rgb_color_buffer_index] |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        rgb_color_buffer[rgb_color_buffer_index] &= ~0xff000000;
        rgb_color_buffer[rgb_color_buffer_index] |= (rx_buf[3] << 24);
      }   
      lastest_rgb_color = rgb_color_buffer[rgb_color_buffer_index];
      if (rgb_color_buffer_index < (RGB_BUFFER_SIZE-1))   
        ++rgb_color_buffer_index;
    }      
    else if ((rx_data[0] >= 0x40) && (rx_data[0] <= 0x4F))
    {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x40+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x40+i] = 1;     
      }      
      if (rx_mark[0]) {
        adc_cal_data.x_negative_min_adc_16bits &= ~0x000000ff;
        adc_cal_data.x_negative_min_adc_16bits |= rx_buf[0];
      }
      if (rx_mark[1]) {
        adc_cal_data.x_negative_min_adc_16bits &= ~0x0000ff00;
        adc_cal_data.x_negative_min_adc_16bits |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        adc_cal_data.x_negative_max_adc_16bits &= ~0x000000ff;
        adc_cal_data.x_negative_max_adc_16bits |= rx_buf[2];
      }
      if (rx_mark[3]) {
        adc_cal_data.x_negative_max_adc_16bits &= ~0x0000ff00;
        adc_cal_data.x_negative_max_adc_16bits |= (rx_buf[3] << 8);
      }   
      if (rx_mark[4]) {
        adc_cal_data.x_positive_min_adc_16bits &= ~0x000000ff;
        adc_cal_data.x_positive_min_adc_16bits |= rx_buf[4];
      }
      if (rx_mark[5]) {
        adc_cal_data.x_positive_min_adc_16bits &= ~0x0000ff00;
        adc_cal_data.x_positive_min_adc_16bits |= (rx_buf[5] << 8);
      }
      if (rx_mark[6]) {
        adc_cal_data.x_positive_max_adc_16bits &= ~0x000000ff;
        adc_cal_data.x_positive_max_adc_16bits |= rx_buf[6];
      }
      if (rx_mark[7]) {
        adc_cal_data.x_positive_max_adc_16bits &= ~0x0000ff00;
        adc_cal_data.x_positive_max_adc_16bits |= (rx_buf[7] << 8);
      }  
      if (rx_mark[8]) {
        adc_cal_data.y_negative_min_adc_16bits &= ~0x000000ff;
        adc_cal_data.y_negative_min_adc_16bits |= rx_buf[8];
      }
      if (rx_mark[9]) {
        adc_cal_data.y_negative_min_adc_16bits &= ~0x0000ff00;
        adc_cal_data.y_negative_min_adc_16bits |= (rx_buf[9] << 8);
      }
      if (rx_mark[10]) {
        adc_cal_data.y_negative_max_adc_16bits &= ~0x000000ff;
        adc_cal_data.y_negative_max_adc_16bits |= rx_buf[10];
      }
      if (rx_mark[11]) {
        adc_cal_data.y_negative_max_adc_16bits &= ~0x0000ff00;
        adc_cal_data.y_negative_max_adc_16bits |= (rx_buf[11] << 8);
      }
      if (rx_mark[12]) {
        adc_cal_data.y_positive_min_adc_16bits &= ~0x000000ff;
        adc_cal_data.y_positive_min_adc_16bits |= rx_buf[12];
      }
      if (rx_mark[13]) {
        adc_cal_data.y_positive_min_adc_16bits &= ~0x0000ff00;
        adc_cal_data.y_positive_min_adc_16bits |= (rx_buf[13] << 8);
      }
      if (rx_mark[14]) {
        adc_cal_data.y_positive_max_adc_16bits &= ~0x000000ff;
        adc_cal_data.y_positive_max_adc_16bits |= rx_buf[14];
      }
      if (rx_mark[15]) {
        adc_cal_data.y_positive_max_adc_16bits &= ~0x0000ff00;
        adc_cal_data.y_positive_max_adc_16bits |= (rx_buf[15] << 8);
      }
      flash_data_write_back(); 
    }      
  }
  if (len == 1) {
    if (rx_data[0] == 0xFF)
    {
      i2c1_set_send_data(i2c_address, 1); 
    }  
    else if (rx_data[0] == 0xFE)
    {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);
    } 
    else if (rx_data[0] == 0xFC)
    {
      bootloader_version = *(uint8_t*)BOOTLOADER_VER_ADDR;
      i2c1_set_send_data((uint8_t *)&bootloader_version, 1);
    }          
    else if (rx_data[0] <= 3)
    {
      memcpy(&tx_buf[0], (uint8_t *)&usAdcValue16[ADC_X1], 2);
      memcpy(&tx_buf[2], (uint8_t *)&usAdcValue16[ADC_Y1], 2);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0], 0x03-rx_data[0]+1);
    }      
    else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x11)
    {
      tx_buf[0] = usAdcValue8[ADC_X1];
      tx_buf[1] = usAdcValue8[ADC_Y1];
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x10], 0x11-rx_data[0]+1);       
    }
    else if (rx_data[0] == 0x20)
    {
      tx_buf[0] = HAL_GPIO_ReadPin(LEFT_SW_B_GPIO_Port, LEFT_SW_B_Pin);
      i2c1_set_send_data(tx_buf, 1);       
    }   
    else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x33))
    {
      memcpy(&tx_buf[0], (uint8_t *)&lastest_rgb_color, 4);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x30], 0x33-rx_data[0]+1);
    }   
    else if ((rx_data[0] >= 0x40) && (rx_data[0] <= 0x4F))
    {
      memcpy(&tx_buf[0], (uint8_t *)&adc_cal_data.x_negative_min_adc_16bits, 16);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x40], 0x4F-rx_data[0]+1);
    }                
    else if ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x53))
    {
      memcpy(&tx_buf[0], (uint8_t *)&offsetAdcValue16[ADC_X1], 2);
      memcpy(&tx_buf[2], (uint8_t *)&offsetAdcValue16[ADC_Y1], 2);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x50], 0x53-rx_data[0]+1);
    }                
    else if ((rx_data[0] >= 0x60) && (rx_data[0] <= 0x61))
    {
      tx_buf[0] = offsetAdcValue8[ADC_X1];
      tx_buf[1] = offsetAdcValue8[ADC_Y1];
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x60], 0x61-rx_data[0]+1);
    }                
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init_flash_data();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)uiAdcValueBuf, ADC_SAMPLES_NUMS);
  sk6812_init(TOTAL_RGB);
  user_i2c_init();
  i2c1_it_enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    i2c_timeout_counter = 0;
    if (i2c_stop_timeout_flag) {
      if (i2c_stop_timeout_delay < HAL_GetTick()) {
        i2c_stop_timeout_counter++;
        i2c_stop_timeout_delay = HAL_GetTick() + 10;
      }
    }
    if (i2c_stop_timeout_counter > 50) {
      LL_I2C_DeInit(I2C1);
      LL_I2C_DisableAutoEndMode(I2C1);
      LL_I2C_Disable(I2C1);
      LL_I2C_DisableIT_ADDR(I2C1);     
      user_i2c_init();    
      i2c1_it_enable();
      HAL_Delay(500);
    }  

    if (rgb_color_buffer_index) {
      uint32_t rgb_show_index = rgb_color_buffer_index;
      for (uint32_t i = 0; i < rgb_show_index; i++) {
        neopixel_set_color(0, rgb_color_buffer[i]);
        neopixel_show();
      }
      rgb_color_buffer_index = 0;
    }     
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
