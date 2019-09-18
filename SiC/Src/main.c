/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"
#include "stm32l0xx_hal_i2c.h"
#include "tools.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "piezo.h"
#include "msp_exp.h"
#include "eeprom_circular.h"
#include "msp_exp.h"
#include "msp_exp_state.h"
#include "interface_flags.h"
#include "tools.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USE_HAL_I2C_REGISTER_CALLBACKS = 1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t aBuffer[MSP_EXP_MAX_FRAME_SIZE];
unsigned long buffLength = 0;
uint8_t transferDirectionGlobal;
uint8_t volatile addr = 0x45; //defines the adress when only one i2c adress is used.
bool volatile has_function_to_execute = false;
void (* volatile command_ptr) () = NULL;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/



/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void SystemClock_Config(void);
void RECIVE_FINISHED(I2C_HandleTypeDef *hi2c);
uint8_t IF_DIRECTION_IS_RECIVE(I2C_HandleTypeDef *hi2c);
uint8_t IF_DIRECTION_IS_SEND(I2C_HandleTypeDef *hi2c);
void BUFF_LENTH(uint8_t* pBuffer, long *BufferLength);






extern volatile struct msp_exp_state_information msp_exp_state;
bool ready = false;
//piezo_sic_type volatile piezo_sic = NONE;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();



  /* Initialize all configured peripherals */
  // the following funtion call will initialize the eeprom by clearing it.
  // reset_EEPROM_buffer(void);
  restore_seqflags();
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  
 
 

    
  while (1)
  {
    if(HAL_I2C_Slave_TxRx_IT(&hi2c1, (uint8_t *)aBuffer, buffLength) != HAL_OK)
    { 
      Error_Handler();
    }
   
    //wait for the i2c reception to finish this must timeout at some point, otherwise there is risk for getting stuck.
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
   
    }

    buff_length((uint8_t *)aBuffer, &buffLength);
    
    
    if(!transferDirectionGlobal)//if we recived a command
    {
       //this funtion returnes the negative error codes in MSP.
       msp_recv_callback((uint8_t *)aBuffer, buffLength, addr);
       msp_send_callback((uint8_t *)aBuffer, &buffLength, addr);
       
       /* this tells us if we are in a msp transaktion, except for the time inbetween the msp code is running
        and the i2c code is running, needs to be covered by flow controll*/ 
    }
    else //if a command was sent
    {
       Flush_Buffer8((uint8_t *)aBuffer, buffLength);
       if(msp_exp_state.type == MSP_EXP_STATE_READY && has_function_to_execute)
       {
            (*command_ptr) ();
            has_function_to_execute = false;
       }
    }
  }
}


/**
  * @brief A callback from the hal i2c library
  * @param i2c handle
  * @param the direction of the communication
  * @param the adress that was called
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *i2cHandle, uint8_t transferDirection, uint16_t addrMatchCode)
{
  transferDirectionGlobal = transferDirection;
// the code bellow is neede if two i2c adresses should be used.
//  if (addrMatchCode == 138)//202
//  {
//    addr = 0x45;
//    piezo_sic = SIC;
//  }
//  if (addrMatchCode == 202)//138
//  {
//    addr = 0x65;
//    piezo_sic = PIEZO;
//  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;//DIV4
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;//6
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
