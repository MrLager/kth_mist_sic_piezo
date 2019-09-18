/****************************************************************************
 * POWER MANAGEMENT FUNCTIONS                                               *
 ****************************************************************************/

/**
 *****************************************************************************
 * @file power_management.c
 * @date 2019-06-26
 * @bug no known buggs
 * @brief power management
 *****************************************************************************
 * this file contains the inforamtion of the experiment power state, active / 
 * inactive, wich is neccesary inorder to stop removing power from a runnig 
 * experiment if they are runned simultaneously.
 * fucntions are defined in order to turn off the power buses induvidualy
 */


#include "power_management.h"
#include "stdbool.h"
#include "usart.h"

/* data section */
bool is_sic_running = false;
bool is_piezo_running = false;

/**
 * @brief turns on power for piezo
 */
void piezo_power_on(void)
{
  is_piezo_running = true;
  HAL_GPIO_WritePin(GPIOB, Battery_SW_ON_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, Piezo_48V_ON_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, Piezo_ON_Pin, GPIO_PIN_SET);
}

/**
 * @brief turns off power for piezo
 */
void piezo_power_off(void)
{
   HAL_GPIO_WritePin(GPIOB, Piezo_48V_ON_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, Piezo_ON_Pin, GPIO_PIN_RESET);
   if(!is_sic_running)
   {
     HAL_GPIO_WritePin(GPIOB, Battery_SW_ON_Pin, GPIO_PIN_RESET);
   }
   is_piezo_running = false;
}

/**
 * @brief turns on power for sic
 */
void sic_power_on(void)
{
  is_sic_running = true;
  HAL_GPIO_WritePin(GPIOB, Battery_SW_ON_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, Linear_10V_ON_Pin, GPIO_PIN_SET);
}

/**
 * @brief turns off power for sic
 */
void sic_power_off(void)
{
  HAL_GPIO_WritePin(GPIOB, Linear_10V_ON_Pin, GPIO_PIN_RESET);
  if(!is_piezo_running)
  {
     HAL_GPIO_WritePin(GPIOB, Battery_SW_ON_Pin, GPIO_PIN_RESET);
  }
  is_sic_running = false;
}

/**
 * @brief turns off 48v
 */
void turn_off_48v(void)
{
    HAL_GPIO_WritePin(GPIOB, Piezo_48V_ON_Pin, GPIO_PIN_RESET);
}

/**
 * @brief turns off 5v
 */
void turn_off_5v(void)
{
  HAL_GPIO_WritePin(GPIOB, Piezo_ON_Pin, GPIO_PIN_RESET); 
}

/**
 * @brief turns off 10v
 */
void turn_off_10v(void)
{
   HAL_GPIO_WritePin(GPIOB, Linear_10V_ON_Pin, GPIO_PIN_RESET);
}

/**
 * @brief turns off vbat
 */
void turn_off_vbat(void)
{
  HAL_GPIO_WritePin(GPIOB, Battery_SW_ON_Pin, GPIO_PIN_RESET);
}
