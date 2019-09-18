/****************************************************************************
 * HIGH ENDURANCE EEPROM DRIVER FOR KTH MIST (SiC in space)                 *
 ****************************************************************************/

/**
 *****************************************************************************
 * @file eeprom_circular.c
 * @date 2019-06-15
 * @bug no known buggs
 * @brief An implementation of a circular eeprom buffer
 *****************************************************************************
 * This is a High Endurance EEPROM driver based on AMTELs High Endurance
 * EEPROM driver example, aplicaiton note (AVR 101).
 * The implemenationt utiliezes STs HAL driver to interface with the
 * EEPROM. The driver was developed for KTHs MIST project. 
 * The EEPROM in the STM32l053c6 has a size of 2k and uses the adresses space:
 * 0x0808 0000 - 0x0808 07FF.
 *  
 * @see http://ww1.microchip.com/downloads/en/appnotes/doc2526.pdf
 * @see https://www.st.com/resource/en/datasheet/stm32l053c6.pdf
 */


/*includes section*/
#include "eeprom_circular.h"


/*defines (constants) section*/
#define WRITE_CYCLES            2000
#define EEPROM_START_ADRESS     0x08080000UL
#define EEPROM_END_ADRESS       0x080807FFUL
#define BUFFER_SIZE             0x00000044UL

#define INDEX_BUFFER_START_ADDR 0x08080000UL + (BUFFER_SIZE*0)
#define BUFFER_1_START_ADDR     0x08080000UL + (BUFFER_SIZE*1)
#define BUFFER_2_START_ADDR     0x08080000UL + (BUFFER_SIZE*2)
#define BUFFER_3_START_ADDR     0x08080000UL + (BUFFER_SIZE*3)
#define BUFFER_4_START_ADDR     0x08080000UL + (BUFFER_SIZE*4)
#define BUFFER_5_START_ADDR     0x08080000UL + (BUFFER_SIZE*5)
#define BUFFER_6_START_ADDR     0x08080000UL + (BUFFER_SIZE*6)
#define BUFFER_7_START_ADDR     0x08080000UL + (BUFFER_SIZE*7)
#define BUFFER_8_START_ADDR     0x08080000UL + (BUFFER_SIZE*8)
#define CHECKSUM_BUFFER         0x08080000UL + (BUFFER_SIZE*9)

/**
 * @brief simple checksum for EEPROM.
 * @param 8 bit pointer
 * @param the length of array
 * @return the checksum
 */
uint16_t checksum(uint16_t *ptr1, uint8_t size1, uint16_t *ptr2, uint8_t size2) 
{
    uint16_t chk = 0;
    while (size1-- != 0)
        chk -= *ptr1++;
    while (size2-- != 0)
        chk -= *ptr2++;
    return chk;
}

/**
 * @brief Finds the current index in the index buffer.
 * @return the offset from the base adress of the buffer
 *
 * finds the offset for the current memmory location
 */
static int find_current_EEPROM_address(void)
{
  int offset = 0;

  while (((*(__IO uint16_t *)(INDEX_BUFFER_START_ADDR + offset))+1) == (*(__IO uint16_t *)(INDEX_BUFFER_START_ADDR + offset +2)))
  {
    offset = (offset + 2)%BUFFER_SIZE;
  }
  return offset;
}




/** @brief Restarts eeprom buffer.
 *  
 *  Resets the EEPROM buffer by zeroing it.
 */
void reset_EEPROM_buffer(void)
{
  HAL_FLASHEx_DATAEEPROM_Unlock();
  for (int i =0x08080000UL; i<0x080802AAUL; i=i+4)
  {
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, i, 0);
  }
  HAL_FLASHEx_DATAEEPROM_Lock();
}




/**
 * @brief Reads data from the two data bufferts.
 * @param 32 bit of data to be written to the first buffer (pointer)
 * @param 32 bit of data to be written to the second buffer (pointer)
 * @return bool indicating if the checksum was correct
 *
 * function written inorder to save 64 bit of data. shoud
 * be used for saving msp sequence flags
 *  
 */
bool EEPROM_read_buffer(unsigned short *data1, unsigned short *data2)
{
  int offset = find_current_EEPROM_address();
  uint16_t checksum_stored;
  
  if (offset == 0)
  {
    data1[0] = (*(__IO uint16_t *)(BUFFER_1_START_ADDR));
    data1[1] = (*(__IO uint16_t *)(BUFFER_2_START_ADDR));
    data1[2] = (*(__IO uint16_t *)(BUFFER_3_START_ADDR));
    data1[3] = (*(__IO uint16_t *)(BUFFER_4_START_ADDR));
    
    data2[0] = (*(__IO uint16_t *)(BUFFER_5_START_ADDR));
    data2[1] = (*(__IO uint16_t *)(BUFFER_6_START_ADDR));
    data2[2] = (*(__IO uint16_t *)(BUFFER_7_START_ADDR));
    data2[3] = (*(__IO uint16_t *)(BUFFER_8_START_ADDR));
    
    checksum_stored = (*(__IO uint16_t *)(CHECKSUM_BUFFER));
  }  
  else
  {
    offset = offset-2;
    
    data1[0] = (*(__IO uint16_t *)(BUFFER_1_START_ADDR + offset));
    data1[1] = (*(__IO uint16_t *)(BUFFER_2_START_ADDR + offset));
    data1[2] = (*(__IO uint16_t *)(BUFFER_3_START_ADDR + offset));
    data1[3] = (*(__IO uint16_t *)(BUFFER_4_START_ADDR + offset));
    
    data2[0] = (*(__IO uint16_t *)(BUFFER_5_START_ADDR + offset));
    data2[1] = (*(__IO uint16_t *)(BUFFER_6_START_ADDR + offset));
    data2[2] = (*(__IO uint16_t *)(BUFFER_7_START_ADDR + offset));
    data2[3] = (*(__IO uint16_t *)(BUFFER_8_START_ADDR + offset));
    
    checksum_stored = (*(__IO uint16_t *)(CHECKSUM_BUFFER + offset));
  }
  

  uint16_t checksum_result = checksum(data1, 4, data2, 4);
  return ((checksum_result-checksum_stored) == 0);
}


/**
 * @brief Writes data to the two buffers.
 * @param 32 bit of data to be written to the first buffer (pointer)
 * @param 32 bit of data to be written to the second buffer (pointer)
 *
 * writes the data and checksum
 */
void EEPROM_write_buffer(unsigned short *data1, unsigned short *data2)
{ 
   HAL_FLASHEx_DATAEEPROM_Unlock();

  /*get the offset from the index buffer*/
  int offset = find_current_EEPROM_address();
  
  /*calculate the checksum*/
  uint16_t value_checksum = checksum (data1, 4, data2, 4); 
 
  /*write the data*/
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_1_START_ADDR+offset), data1[0]);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_2_START_ADDR+offset), data1[1]);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_3_START_ADDR+offset), data1[2]);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_4_START_ADDR+offset), data1[3]);
  
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_5_START_ADDR+offset), data2[0]);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_6_START_ADDR+offset), data2[1]);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_7_START_ADDR+offset), data2[2]);
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (BUFFER_8_START_ADDR+offset), data2[3]);
  
  /*write checksum*/
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, (CHECKSUM_BUFFER+offset), value_checksum);

  
  /*uppdate the index buffert*/
  uint16_t index_var = (*(__IO uint16_t *)((offset) + INDEX_BUFFER_START_ADDR));
  index_var++;
  uint32_t address = ((offset+2)%BUFFER_SIZE) + INDEX_BUFFER_START_ADDR;
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, address, index_var);
  int readvar = (*(__IO uint16_t *)(address));
  HAL_FLASHEx_DATAEEPROM_Lock();
}


