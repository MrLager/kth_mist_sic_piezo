
#include "stm32l0xx_hal.h"
# include <stdint.h>
#include <stdbool.h>
static int find_current_EEPROM_address(void);
void reset_EEPROM_buffer(void);
bool EEPROM_read_buffer(unsigned short *data1, unsigned short *data2);
void EEPROM_write_buffer(unsigned short *data1, unsigned short *data2);
uint16_t checksum (uint16_t *ptr1, uint8_t sz1, uint16_t *ptr2, uint8_t sz2);