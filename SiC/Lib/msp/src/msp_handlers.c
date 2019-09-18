//this file was written by  @author: Simon Lagerqvist.
#include "msp_exp_handler.h"
#include <stdint.h>
#include "main.h"
#include "start_test.h"
#include "msp_opcodes.h"
#include <stdbool.h>
#include "piezo.h"
#include "start_test.h"
#include "power_management.h"
#include "sicpiezo_global.h"
#include <interface_flags.h>



uint8_t sicBuffer[64];
uint8_t piezoBuffer[30];
extern bool volatile sic;
extern bool volatile piezo;
extern int piezoTick;
extern bool getData;
extern char piezoBufferRx[50]; 
extern int piezoBufferRxInt[50];
extern int aTxBuffer[100];
extern uint16_t buffLength;

extern bool volatile has_function_to_execute;
extern void (* volatile command_ptr) ();
bool volatile debuggvar = false;
bool debuggvar2 = false;
bool debuggvar3 = false;
extern piezo_sic_type volatile piezo_sic;
bool piezo_error = false;
bool sic_error = false;
int i = 0;


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
  if (opcode == REQ_PIEZO)
  { 
    *len = piezo_get_data_length();
  }
  else if (opcode == REQ_SIC)
  {
    *len = 64;
  }
}

void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
  if (opcode == REQ_PIEZO)
  { 
    piezo_get_data(buf, offset);
  }
  else if (opcode == REQ_SIC)
  {
     sic_get_data(buf, offset);
  }
}

void msp_expsend_complete(unsigned char opcode)
{
// add code to clear buffers
  if (opcode == REQ_PIEZO)
  { 
    clear_piezo_buffer();
  }
  else if (opcode == REQ_SIC)
  { 
     clear_sic_buffer();
  }
}

void msp_expsend_error(unsigned char opcode, int error)
{
  //add code to set an error 
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{

}

void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{

}

void msp_exprecv_complete(unsigned char opcode)
{

}

void msp_exprecv_error(unsigned char opcode, int error)
{

}

void msp_exprecv_syscommand(unsigned char opcode)
{
  switch(opcode) 
  {    
    case START_EXP_PIEZO:
      i = 1;
      command_ptr = &piezo_start_exp;
      has_function_to_execute = true;
      break;
      
    case STOP_EXP_PIEZO:
      i = 2;
      command_ptr = &piezo_stop_exp;
      has_function_to_execute = true;
      break;
      
    case START_EXP_SIC:
      i = 3;
      command_ptr = start_test;
      has_function_to_execute = true;
      break; 
    
    case MSP_OP_POWER_OFF:
      i = 4;
      command_ptr = save_seqflags;
      has_function_to_execute = true;
      break; 
    
    case SIC_10V_OFF:
      turn_off_10v();
      break;
      
    case PIEZO_5V_OFF:
      turn_off_5v();
      break;
      
    case PIEZO_48V_OFF:
      turn_off_48v();
      break;
      
    case VBAT_OFF:
      turn_off_vbat();
      break;
  }
}