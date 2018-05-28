/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MISO_Pin GPIO_PIN_5
#define MISO_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_6
#define SCK_GPIO_Port GPIOA
#define SS_Pin GPIO_PIN_7
#define SS_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_14
#define MOSI_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_5
#define LCD_SDA_GPIO_Port GPIOB
#define LCD_CLS_Pin GPIO_PIN_6
#define LCD_CLS_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_7
#define LCD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef struct {
	short pid_p;
	short pid_i;
	short pid_d;
	short pid_sumerr;
	short pid_period;
	short pid_lasterr;
	short pid_preerr;
	short pid_set;
} PID_Type;

typedef struct{
	PID_Type PID_Heater;
  short  offsetUA;   //0x08
	short offsetUR;//OX09
	short UR;//OX0a
	short UA;//OX0b
	short UR_Filt_Ratio;//OX0c
	short UA_Filt_Ratio;//OX0d
	short PWM_MAX;//0X0e
	short pump_current;////OX0f
	short ref_pump_current;//OX10
	short O2PP;//0X11
	short lambda;//OX12
	short T_Set;//OX13
	short R_Set;  //OX14
	short R_Current;//0X15
	short T_Current;//16
	short PWM_set;//17
	short O2PPRatio;//0x18
	short partial;//0x19
	//short saveFlag;
 
}Saved_Type;
extern  uint8_t uart_rx_buff[9];
extern uint8_t uart_tx_buff[50];
extern Saved_Type para_save;
extern uint8_t saveFlag;



/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
