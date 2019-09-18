/****************************************************************************
 * TOOLS / UTILITY FUNCTIONS                                               *
 ****************************************************************************/

/**
 *****************************************************************************
 * @file tools.c
 * @author Simon Lagerqvist
 * @date 2019-06-15
 * @bug no known buggs
 * @brief utility functions
 *****************************************************************************
 * this file contains utility functions that are used in several places 
 * in the project.
 */

/* includes */
#include "tools.h"
#include "msp_exp.h"


void Flush_Buffer(int* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer = 0;
    
    pBuffer++;
  }
}

void Flush_Buffer8(uint8_t* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer = 0;
    
    pBuffer++;
  }
}

void buff_length(uint8_t* pBuffer, long *BufferLength)
{
  for(int i=0; i<MSP_EXP_MAX_FRAME_SIZE; i++)
  {
    if(* pBuffer != '\0')
      *BufferLength = (i+1); 
    pBuffer++;
  }
}

