
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "iwdg.h"
#include "gpio.h"
#include "usart.h"
//#include "SystemClock_Config.h"
//#include "SystemPower_Config.h"
#include "start_test.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "power_management.h"
#include "tools.h"
//#include "header.h"


#define EXPERIMENTSIZE sizeof(experiments)


/* Structs -------------------------------------------------------------------*/
struct experiment_package {
  /* 
  temperature
  Vrb (voltage over resistor on base)
  Vrc (voltage over resistor on collector)
  Ube (voltage frop from base to emitter)
  */
  uint16_t temperature;
  uint16_t Vbe;
  uint16_t Vb;
  uint16_t Vc;
 
};

/* Private variables ---------------------------------------------------------*/
uint16_t 	 number_of_tests = 16;
uint32_t 	 timerDelay = 1;
//uint8_t rxbuff[64];



/* External variables*/
extern ADC_ChannelConfTypeDef           sConfigAdc;
extern DAC_ChannelConfTypeDef 	        sConfigDac;
extern ADC_HandleTypeDef                hadc;
extern DAC_HandleTypeDef    		hdac;
extern UART_HandleTypeDef 		huart1;
extern I2C_HandleTypeDef 		hi2c1;
static struct experiment_package  	experiments[8];


void setDAC(uint32_t);
void readRollingADC(int);
void shiftAverages(void);
void send_message(uint8_t * message);
void convert_8bit(uint8_t * buffer);
uint8_t buffer[64];

void clear_sic_buffer (void)
{
    Flush_Buffer8(buffer, 64);
}


void sic_get_data(unsigned char *buf, long data_offset)
{
  int i =0;
  while (i<64)
  {
    buf[i] = buffer[i++];
  }
  
}

void start_test(void){
  
      //HAL_GPIO_WritePin(GPIOB,Piezo_48V_ON_Pin,GPIO_PIN_SET);
      //HAL_Delay(1000);
    //HAL_GPIO_WritePin(GPIOB,Piezo_ON_Pin,GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOB,Battery_SW_ON_Pin,GPIO_PIN_SET);  
//      HAL_GPIO_WritePin(GPIOB,Linear_10V_ON_Pin,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB,Piezo_48V_ON_Pin,GPIO_PIN_RESET);
   
   sic_power_on();
   HAL_Delay(1000);
  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    First test value for the measurment of the BJTs
    The setDAC range should bee between 0.5 to 2.8 
    The first value is 0.7V times 1241 to convert it to HEX 32 bits
   //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/ 
  setDAC(0.7*1241);
  
  readRollingADC(0);

 /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Second test value for the measurment of the BJTs
    The setDAC range should bee between 0.5 to 2.8 
    The first value is 1.2V times 1241 to convert it to HEX 32 bits
   //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/ 
    
  setDAC(1.2*1241);
    
  readRollingADC(2);

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Third test value for the measurment of the BJTs
    The setDAC range should bee between 0.5 to 2.8 
    The first value is 2.1 V times 1241 to convert it to HEX 32 bits
   //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/ 
  
  setDAC(2.1*1241);
   readRollingADC(4);
   
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Fourth test value for the measurment of the BJTs
    The setDAC range should bee between 0.5 to 2.8 
    The first value is 2.8V times 1241 to convert it to HEX 32 bits
   //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/ 
    
  setDAC(2.8*1241);
   readRollingADC(6);

    


  /**%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

This code below is for plotting the graphs in Matlab. A lot of 
point were needed to make a good plotting.  

*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
*/
  
  shiftAverages();
  
  HAL_Delay(30);
  
  //printf("this is whats in experiments: %d\n", experiments[1].Vb);
  convert_8bit(buffer);
  

//  printf("this is the result of sic: \n"); 
//  for(int i = 0; i<=32;){
//    printf("%f ",rxbuff[i]);
//  }
  //printf("\n");
  //send_message(rxbuff);
  setDAC(0);
  HAL_Delay(100);
  sic_power_off();
 // printf("\n");
 
  }
 /**
@brief convert 8bit, converts the 16 bit experiments measured values to 8 bits 
@param uint16_t, the measured experiments 
it saves the the experiments value in 8bit array which is converted back to 16 bits 
in the OBC
@return rxbuff 
*/

 void convert_8bit(uint8_t * buffer){
   //printf("\n temp si %d\n", experiments[0].temperature); 
  for(int i =0; i<=32;){
    for (int y = 0; y <=6 ; ){
    
   //printf("\n temp si %d\n", experiments[0].temperature);
   buffer [i]  = experiments[y].temperature>>8 & 0xFF   ; 
   //printf("\n temp si %d\n", (experiments[y].temperature & 0xFF)); 
   buffer [i+1]= experiments[y].temperature & 0xFF   ;
  // printf("this is text %d ", rxbuff[i+1]);
  
   
   buffer [i+2]= experiments[y].Vbe >>8 & 0xFF  ;
   buffer [i+3]= experiments[y].Vbe  & 0xFF  ;
   
   buffer [i+4]= experiments[y].Vb >>8 & 0xFF   ; 
   buffer [i+5] = experiments[y].Vb & 0xFF   ;  
   
   buffer [i+6]=experiments[y].Vc >>8 & 0xFF   ;
   buffer [i+7]= experiments[y].Vc & 0xFF  ;
 
   buffer [i+8]=experiments[y+1].temperature>>8 & 0xFF   ; 
   buffer [i+9]=experiments[y+1].temperature & 0xFF   ;
     
       
   buffer [i+10]=experiments[y+1].Vbe >>8 & 0xFF  ;
   buffer [i+11]=experiments[y+1].Vbe  & 0xFF  ;
         
           
   buffer [i+12]=experiments[y+1].Vb >>8 & 0xFF   ; 
   buffer [i+13]=experiments[y+1].Vb & 0xFF   ; 
             
           
   buffer [i+14]=experiments[y+1].Vc >>8 & 0xFF   ;
   buffer [i+15]=experiments[y+1].Vc & 0xFF  ;
 
     i=i+16;
     y=y+2;
    }
  }
  //printf("this is the result of sic: \n"); 
 // printf("%d", experiments[1].temperature>>8 & 0xFF);
////////////////////////////////  int i;
////////////////////////////////  for(int i = 0; i<=32;i++){
////////////////////////////////    printf("%d ", rxbuff[i]);
////////////////////////////////  }
 }


/**
@brief shift Averages, calculates the mean value of the experiments 
@param uint16_t , the measured experiments 
it saves the mean value of the experiments in the experiments array
@return void
*/

void shiftAverages(){
  for(int i = 0; i < 8; i++){
    experiments[i].temperature = (experiments[i].temperature >> 4);
    experiments[i].Vbe = (experiments[i].Vbe >> 4);
    experiments[i].Vb = (experiments[i].Vb >> 4);
    experiments[i].Vc = (experiments[i].Vc >> 4);
  }
}
/**
@brief read rolling ADC, Reads the ADC measured value adds them together for
each experiment.  
@param uint16_t , index for which elemnt in the array the values should be stored
in.
@ It has four functions, Calibrate the ADC, Start the ADC, Poll for convention
and get the adc value. This is made according to the initialization in HAL Library. 
@return void
*/
void readRollingADC(int index){
  
   //Calibrate ADCs in the beginning of every run
  	if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK){
          
		Error_Handler();
	}
   //Start ADC reading
      HAL_Delay(10);

  //Start ADC reading

	if(HAL_ADC_Start(&hadc) != HAL_OK){
		while(1) {

			Error_Handler();
		}
	}


	 HAL_ADC_PollForConversion(&hadc, 100);
	 experiments[0+index].temperature += HAL_ADC_GetValue(&hadc);
         //printf("\n temp si %d\n", experiments[0+index].temperature); 
         
         HAL_ADC_PollForConversion(&hadc, 100); 
	 experiments[0+index].Vbe += HAL_ADC_GetValue(&hadc);
         //printf("\n Vbe si %d\n", experiments[0+index].Vbe); 
      
         HAL_ADC_PollForConversion(&hadc, 100);
         experiments[0+index].Vb += HAL_ADC_GetValue(&hadc);
         //printf("\n Vb si %d\n", experiments[0+index].Vb); 
      
         HAL_ADC_PollForConversion(&hadc, 100);
         experiments[0+index].Vc += HAL_ADC_GetValue(&hadc);
         //printf("\n Vc si %d\n", experiments[0+index].Vc); 
      
     
         HAL_ADC_Start(&hadc);
	 HAL_ADC_PollForConversion(&hadc, 100);
	 experiments[1+index].temperature += HAL_ADC_GetValue(&hadc);
        // printf("\n temp sic %d\n", experiments[1+index].temperature); 
         
         HAL_ADC_PollForConversion(&hadc, 100); 
	 experiments[1+index].Vbe += HAL_ADC_GetValue(&hadc);
        // printf("\n ube sic %d\n", experiments[1+index].ube); 
      
         HAL_ADC_PollForConversion(&hadc, 100);
         experiments[1+index].Vb += HAL_ADC_GetValue(&hadc);
      //   printf("\n vrb sic %d\n", experiments[1+index].vrb); 
      
         HAL_ADC_PollForConversion(&hadc, 100);
         experiments[1+index].Vc += HAL_ADC_GetValue(&hadc);
       //  printf("\n vrc sic %d\n", experiments[1+index].vrc); 

		
	  HAL_Delay (2);
 
	  HAL_ADC_Stop (&hadc);
 
}
  /**
@brief SetDAC, Sets the voltage to the BJTS circuits
 
@param uint32_t , the Value the setDAC should have, max is 4095 which is 3.25V
in HEX. If instead the desired input value wants to be in decimal it should be 
multiplied with 1241.  
@ The setDAC function, transforms the digital output from the MCU to 
an Analog output which is needed for the BJT circuits. 
@return void
*/
void setDAC(uint32_t voltage){
  HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, voltage);
  HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
}

 /**
@brief send message, sends the experiments data to the OBC.  
 
@param uint8_t , The messeage that has to be sent is in 16 bits, and the 
HAL functions has 8 bit data transfer, therfore the experiments data has 
first to be coverted to 8 bits so it can be sent, which is done in function
convert_8bit      
 
@returns,
*/

//void send_message(uint8_t * message){
//	
//        for(int i =0; i<64;i++)
//        {
//          printf("%d ", *message++); //print the test result for sic.
//        }
//        printf("\n");
////	if(HAL_I2C_Slave_Transmit(&hi2c1,message,(uint16_t)EXPERIMENTSIZE, 5000)!= HAL_OK)
////	{
////	  Error_Handler();
////  
////
////	}
//	
//	 return;
//	
//}