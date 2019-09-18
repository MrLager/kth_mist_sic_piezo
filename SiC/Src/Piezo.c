/****************************************************************************
 * Piezo-LEGS communication & handler written FOR KTH MIST (SiC in space)   *
 ****************************************************************************/

/**
 *****************************************************************************
 * @file piezo.c
 * @date 2019-06-15
 * @bug no known buggs
 * @brief Interface for the Piezo motor.
 *****************************************************************************
 * this file contains functions that can be used in order to interact with 
 * pizo-leggs via rs485
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "piezo.h"
#include "usart.h"
#include "power_management.h"   
#include "tools.h"

   
int NUMBER_OF_READ_ATTEMTS = 3;   
   
/* predifined commands for the motor */
char xm3_buffer[4]="XM3;";
char xm4_buffer[4]="XM4;";

/* data section */
uint8_t xu6_buffer[6]; // used for sending data request
uint8_t saveDataPointer[1];
uint8_t piezoData[200];
int piezoBufferRxInt[200]; 
uint8_t piezoBufferint8[200];
extern UART_HandleTypeDef huart1;
int dataLength = 0;


void clear_piezo_buffer (void)
{
    Flush_Buffer8(piezoData, (dataLength));
    Flush_Buffer8(piezoBufferint8, (dataLength));
}

/**
 * @brief starts the motor by sending xm3
 */
void piezo_start_exp(void)
{
  piezo_power_on();
  HAL_Delay(3000); // time it takes for the motor to turn on.
  HAL_UART_Transmit(&huart1, (uint8_t *)xm3_buffer, 4, 1000);
}

/**
 * @brief stops the motor by sending xm4
 */
void piezo_stop_exp(void)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)xm4_buffer, 4, 1000);
  dataLength = piezo_read_data_records();
  piezo_power_off();
  
}

/**
 * @brief retrieves the data collected data from the buffer
 * @prarm buffer to copy the data to
 * @param the data offset
 */
void piezo_get_data(unsigned char *buf, long data_offset)
{
  long i = data_offset; 
  while (i<dataLength)
  {
    buf[i] = piezoBufferint8[i];
    i++;
  }
}

/**
 * @brief retrieves the length of the buffer
 */
int piezo_get_data_length(void)
{
  return dataLength;
}

/**
 * @brief retrieves data from pizo leggs
 * @prarm buffer to copy the data to
 * @param the data offset
 *
 * this function will read data until an empty record is found 
 * or until the max read attempts is reached.
 * checksum is checked and the data is formated in 8bit integers
 */

unsigned char piezo_read_data_records(void)
{
  bool isThereMoreData = true;
  int record_counter = 0;
  uint16_t dataOffset = 0;
  int a=0;
  
  //read data records until a empty record is read.
  while(isThereMoreData)
  {
    //retry reading data until max attempts is reached if checksum was not ok.
    for (a =0; a<NUMBER_OF_READ_ATTEMTS; a++)
    {
      
      //we already know there is more data to read since we pased while, need to 
     //be reset because of checksum check since we want to exit if we fail checksum test more than max read attempts 
      isThereMoreData = true;
      
      
      sprintf((char *)xu6_buffer, "XU6,%d\r", record_counter);
      int i=0;
      HAL_UART_Transmit(&huart1, (uint8_t *)xu6_buffer, 6, 100);
   
      while(1)
      {
        if(HAL_UART_Receive(&huart1, (uint8_t *)saveDataPointer, 1, 100) != HAL_OK)
          break; // stop reading
        piezoData[i++] = saveDataPointer[0];
        if ('\r' == saveDataPointer[0]||i==199)//199 so we do not write outside array
           break; // stop reading
      }
      
      //check if record was empty
      if(record_was_empty((char *)&piezoData[6])) 
      {
        isThereMoreData = false;
        break; // break the attempt loop
      }
      //convert to integers, skip (xu6:) therefore 4 
      ascii_to_int((char *)&piezoData[4], (int *)&piezoBufferRxInt[dataOffset]);
      
      //calulate checksum
      if(piezo_checksum((int *)&piezoBufferRxInt[dataOffset]))
      {
        //if the checksum was correct, read the next record.
        dataOffset += 9;
        record_counter++;
        break; // break the attempt loop
      }
      else
      {
        //flush the last recived incorrect values
        Flush_Buffer8(piezoData, i);
        isThereMoreData = false;
      }
    }
  }
  convert_to_8bit(piezoBufferint8, dataOffset*2);
  return dataOffset*2;
}

/**
 * @brief check if the record that was read was empty, eather by containg 0,0,0
 * or by containing null.
 * @prarm buffer to copy the data to
 * @param the data offset
 */
bool record_was_empty(char * bufferIn)
{
   //ascii 0 * 8 = 384 
  return ((bufferIn[0] + bufferIn[2] + bufferIn[4] + bufferIn[6] + bufferIn[8] + bufferIn[10] + bufferIn[12] + bufferIn[14])==384)
    ||((bufferIn[0] + bufferIn[2] + bufferIn[4] + bufferIn[6] + bufferIn[8] + bufferIn[10] + bufferIn[12] + bufferIn[14])==0);
}



/**
 * @brief calculated the checksum for a command that was recived by from piezo
 * @prarm the buffer to calculate the checksum from
 * @param if the checksum was correct
 */
bool piezo_checksum(int *piezoBufferRx)
{
    piezoBufferRx++;
    int a=0;
    for(int x=0; x<9; x++)
    {
        a = a ^ *piezoBufferRx;
        
        piezoBufferRx++;
    }
    if (a==*piezoBufferRx)
    {
      return true;
    }
   return false;   
}

/**
 * @brief calculated the checksum for a command that was recived by from piezo
 * @prarm buffer with ascii caracters
 * @param buffer that the integere valued should be written to
 */
void ascii_to_int (char * bufferIn, int * bufferOut)
{
  for (int i = 0; i < 9; i++)
  {
      int temp = 0;
      while(((*bufferIn != ',')&&(*bufferIn != '\r')))
      {
          temp = temp*10;
          temp = temp + (*bufferIn - '0');
          bufferIn++;
      }
      *bufferOut=temp; 
      bufferOut++;
      bufferIn++;
  }
}

void convert_to_8bit(uint8_t * buffer, uint16_t length)
{
  for (int i = 0; i < length; i++)
  {
      *buffer = piezoBufferRxInt[i] >>8 & 0xFF  ;
      *++buffer = piezoBufferRxInt[i]  & 0xFF  ;
      buffer++;
  }
}


