/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "DataFlash.h"
#include "spi.h"
#include "timer.h"

#define STATUS_IDLE 0 
#define STATUS_INIT 1
#define STATUS_RAMPUP 2
#define STATUS_RUN 3
#define STATUS_ERR 4 
#define LSU49

#ifdef LSU49
  uint16_t temperaturePoint=7600;
 
  #define R_SET 300.0
 int temp_r[11][2]={
		 {6350,10000},
		 {6450,9000},
		 {6550,8000},
		 {6700,7000},
		 {6850,6000},
		 {7150,5000},
		 {7380,4000},
		 {7800,3000},
		 {8450,2000},
		 {9000,1500},
		 {9500,1200},
 };
  
#else
  uint16_t temperaturePoint=7400;
 
  #define R_SET 82.5
 int temp_r[11][2]={
		 {6255,3000},
		 {6320,2000},
		 {6500,1700},
		 {6700,1500},
		 {6850,1200},
		 {7000,1080},
		 {7300,900},
		 {7500,820},
		 {7750,800},
		 {9000,460},
		 {9500,420},
 };
  
#endif

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
  uint8_t saveFlag=0;
uint16_t ADC_buff[50][3]={0,0,0};
uint8_t uart_rx_buff[9];
uint8_t uart_tx_buff[50];
uint8_t version=0;
uint8_t Status_Machine=STATUS_IDLE;
PID_Type PID_Heater;
Saved_Type para_save;
int temp_ua,temp_ur,count_adj;
float urCurrentInit;
double  temp_UA=0,temp_UR=0;
double  k3=30;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int  getAdcAfterFilt(uint8_t chanal , uint8_t filt)
{
	uint32_t adcDate=0,i;
	for(i = 0;i<50;i++)
	adcDate+=ADC_buff[i][chanal];
	return  (int)(adcDate/50);
}


 void Save_PID_Arg(void)
 {
 		STMFLASH_Write(FLASH_BASE+FLASH_PAGE_SIZE*50,(uint16_t *)&para_save,sizeof(Saved_Type)/2);
 
 }
void Init_PID_Arg(void)
{
		 STMFLASH_Read(FLASH_BASE+FLASH_PAGE_SIZE*50,(uint16_t *)&para_save,sizeof(Saved_Type)/2);
	   if(para_save.lambda==-1)
			{
				para_save.lambda=0;
				para_save.PID_Heater.pid_p=100;
				para_save.PID_Heater.pid_i=10;
				para_save.PID_Heater.pid_d=50;
				para_save.PID_Heater.pid_sumerr=0;
				para_save.R_Set = 3000;
				para_save.offsetUA = 0;
				para_save.offsetUR = 0;
				para_save.PWM_MAX=800;
				para_save.UA_Filt_Ratio=92;
				para_save.UR_Filt_Ratio=30;
			  para_save.O2PPRatio=840;      //博世1号(亿科拆下来的1号)多项式拟合
			//	para_save.O2PPRatio=824;   //基本公式（原来用的公式）
				//para_save.O2PPRatio=102;   //上硅所1号（已坏）
      	//para_save.O2PPRatio=83;   //博世1号(亿科拆下来的1号)
				para_save.partial=21;
				Save_PID_Arg();
		}
}


uint16_t get_temp(int r )
{
	uint8_t index=0;
	uint16_t err=0;
 
	for(index=0;index<11;index++)
		{
			if(r>temp_r[index][1])
				{
					break;
				}
		}
	err=(long )(temp_r[index-1][1]-r)*(temp_r[index][0]-temp_r[index-1][0])/(temp_r[index-1][1]-temp_r[index][1]);
	return err+temp_r[index-1][0];
}

uint16_t get_lambda_O2PP(double O2PP)
{
  
  double lambda=0;
	if(O2PP>=0&&O2PP<13)   //1%时理论lambda为1.08
		lambda = (O2PP /100/ 3 +1) / (1-O2PP/100*4.77)*1000-10;
	else if(O2PP>=13)
		lambda =9999;
	//else if(O2PP<0)
	//	lambda = (O2PP /100*2.668+1.00906) *1000;

 // lambda=(0.839*pumpCurrent+10000l)*100/(1000l-0.3901*pumpCurrent);//原始的
   return  lambda;
}

uint16_t get_lambda_Ip(double ip)
{
  
  double lambda=0;
	
	//lambda = (1.64581*ip/1000*ip/1000*ip/10000+3.71251*ip/1000*ip/10000+4.27208*ip/10000+1.0399) *1000;
    lambda = (1.71725*ip/1000*ip/1000*ip/10+3.33748*ip/1000*ip/10+3.54254*ip/10+1007.01) ;
 // lambda=(0.839*pumpCurrent+10000l)*100/(1000l-0.3901*pumpCurrent);//原始的
   return  lambda;
}

uint8_t sentToTFT(void)
{
	uint8_t i;
	uart_tx_buff[0]=0xa5;
	uart_tx_buff[1]=0x5a;
	uart_tx_buff[2]=sizeof(Saved_Type)+3;
	uart_tx_buff[3]=0x82;
	uart_tx_buff[4]=0;
	uart_tx_buff[5]=0;
	for(i=0;i<sizeof(Saved_Type);i+=2)
	{
		uart_tx_buff[7+i] = *((uint8_t *)&para_save + i);
		uart_tx_buff[6+i] = *((uint8_t *)&para_save + i+1);
	}
	  HAL_UART_Transmit_DMA(&huart1,uart_tx_buff,sizeof(Saved_Type)+6);
	return 1;
}
int PWM_ctrl(int get_r)
{
	static short err;

 
	int result;
	err = get_r-para_save.PID_Heater.pid_set;
	para_save.PID_Heater.pid_sumerr+=err;
	if(para_save.PID_Heater.pid_sumerr>10000)
		para_save.PID_Heater.pid_sumerr=10000;
	if(para_save.PID_Heater.pid_sumerr<-10000)
		para_save.PID_Heater.pid_sumerr=-10000;
	result=(int)((para_save.PID_Heater.pid_p/100.0)*err+(para_save.PID_Heater.pid_i/100.0)*para_save.PID_Heater.pid_sumerr\
		+(para_save.PID_Heater.pid_d/100.0)*(err-para_save.PID_Heater.pid_lasterr));
	if(result>1600)
		result=1600;
	if(result<=0)
		result=0;
	return result;
}

void Heater_Ctrl(void )
{
	
}
void Status_main(void)
{

	switch(Status_Machine)
	{
		case STATUS_IDLE:
					//读出PID参数
					Init_PID_Arg();
					temp_ua=0;
					temp_ur=0;
					count_adj=0;
					Status_Machine=STATUS_INIT;
			break;
		case STATUS_INIT:
					if((CJ125readVersion()&0xf0)!=0x60)
					{
						Status_Machine=STATUS_ERR;
					}
					CJ125readALL();
					DelayTimer(20);
					reg1 = 0xd8;
					reg2 = 0x00;
					CJ125writeALL();
					CJ125readALL();
					if(reg1!=0xd8||reg2!= 0x00)
					{
						Status_Machine=STATUS_ERR;
					}
					DelayTimer(10);
					reg1 = 0xdc;
					reg2 = 0x00;
					CJ125writeALL();
					CJ125readALL();
					if(reg1!=0xdc||reg2!= 0x00)
					{
						Status_Machine=STATUS_ERR;
					}
					DelayTimer(10);
					reg1 = 0xdc;
					reg2 = 0x06;
					CJ125writeALL();
					CJ125readALL();
					if(reg1!=0xdc||reg2!= 0x06)
					{
						Status_Machine=STATUS_ERR;
					}
					
					Status_Machine=STATUS_RAMPUP;

					
			break;
		case STATUS_RAMPUP:
				 //AD采集UA UR  进行零点校正
				temp_ua+=getAdcAfterFilt(0,0); 
				temp_ur+=getAdcAfterFilt(1,0); 
				count_adj++;
				para_save.PWM_set=200+count_adj/2;
		    
				
				if(count_adj==500)
				{
					reg1 = 0xdc;
					reg2 = 0x02;
					CJ125writeALL();
					CJ125readALL();
					if(reg1!=0xdc||reg2!= 0x02)
					{
						Status_Machine=STATUS_ERR;
					}			
				
				}
				if(count_adj==1000)
				{
				  para_save.offsetUA = temp_ua/count_adj;
					para_save.offsetUR = temp_ur/count_adj;
					urCurrentInit=para_save.offsetUR/R_SET;
					para_save.PID_Heater.pid_set =(para_save.R_Set/10)*urCurrentInit;
					reg1 = 0x88;
					reg2 = 0x02;
					CJ125writeALL();
					CJ125readALL();
					if(reg1!=0x88||reg2!= 0x02)
					{
						Status_Machine=STATUS_ERR;
					}
					DelayTimer(1000);
					reg1 = 0x88;
					reg2 = 0x12;
					CJ125writeALL();
					CJ125readALL();
					if(reg1!=0x88||reg2!= 0x12)
					{
						Status_Machine=STATUS_ERR;
					}
					Status_Machine=STATUS_RUN;	
				}
					//线性斜率PWM输出
				if( para_save.PWM_set> para_save.PWM_MAX)
					 para_save.PWM_set= para_save.PWM_MAX;
				TIM1->CCR1  = para_save.PWM_set;
				//读取状态寄存器
				
			break;
		case STATUS_RUN:
				//PID 加热器
				temp_UR= getAdcAfterFilt(1,0)*(1-para_save.UR_Filt_Ratio/100.0)+temp_UR*(para_save.UR_Filt_Ratio/100.0);
				para_save.UR =temp_UR;
			
				   temp_UA= getAdcAfterFilt(0,0)*(1-para_save.UA_Filt_Ratio/100.0)+temp_UA*(para_save.UA_Filt_Ratio/100.0);
				para_save.UA = temp_UA;
		
		
				para_save.PID_Heater.pid_set =(para_save.R_Set/10)*urCurrentInit;
				para_save.PWM_set = PWM_ctrl(para_save.UR);
		    TIM1->CCR1  = para_save.PWM_set;
				//读取泵电流   数据处理
				//UI发
			break;
		case STATUS_ERR:
			break;
		default:break;
		
	}
	CJ125readStatus();
	if((reg_status&0xf0)!=0xf0)
	{
	//	Status_Machine=STATUS_ERR;
	}
	if(saveFlag==1)
	{
		saveFlag=0;
		Save_PID_Arg();
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1); 
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&ADC_buff,150);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_DMA(&huart1,uart_rx_buff,9);
	
	StartTimer(TIMER_RefreshTFT,100);
	StartTimer(TIMERSTATUS_Machine,10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(ReadTimer(TIMERSTATUS_Machine)==OK)
		{
			StartTimer(TIMERSTATUS_Machine,10);
			Status_main();
		}
		if(ReadTimer(TIMER_RefreshTFT)==OK)
		{
//			/*************************博世1号多项式*/
			double temp_O2,temp_current;
			temp_current=para_save.pump_current/10000000*para_save.pump_current;
			StartTimer(TIMER_RefreshTFT,100);
			para_save.pump_current =  (para_save.UA -para_save.offsetUA)*16.2694;
	   temp_O2=para_save.pump_current*((para_save.O2PPRatio+0.3)/10000.0)+para_save.partial\
			               -3.47934*temp_current;
		 para_save.O2PP=temp_O2;
			para_save.R_Current = (para_save.UR*10)/urCurrentInit;
			
//			double temp_O2 , temp_current;
//			temp_current=para_save.pump_current/10000000*para_save.pump_current;
//			StartTimer(TIMER_RefreshTFT,100);
//			para_save.pump_current =  (para_save.UA -para_save.offsetUA)*16.2694;
//	   temp_O2=para_save.pump_current*((para_save.O2PPRatio)/10000.0)+k3\
//			               -3.62784*temp_current;
//		 para_save.O2PP=temp_O2;
//			para_save.R_Current = (para_save.UR*10)/urCurrentInit;
			
			
//			/*************************上硅所1号************************/

//	  	StartTimer(TIMER_RefreshTFT,100);
//			para_save.pump_current =  (para_save.UA -para_save.offsetUA)*16.2694;
//		  para_save.O2PP=para_save.pump_current*((para_save.O2PPRatio)/1000.0)+k3;
//			para_save.R_Current = (para_save.UR*10)/urCurrentInit;	

			
					/**********************博世1号*/

//	  	StartTimer(TIMER_RefreshTFT,100);
//			para_save.pump_current =  (para_save.UA -para_save.offsetUA)*16.2694;
//		  para_save.O2PP=para_save.pump_current*((para_save.O2PPRatio)/1000.0)+para_save.partial;
//			para_save.R_Current = (para_save.UR*10)/urCurrentInit;			


///**************基本公式************************/
//			StartTimer(TIMER_RefreshTFT,100);
//			para_save.pump_current =  (para_save.UA -para_save.offsetUA)*16.2694;
//		  para_save.O2PP=para_save.pump_current*((para_save.O2PPRatio)/10000.0);
//			para_save.R_Current = (para_save.UR*10)/urCurrentInit;			
				   
#ifdef LSU49
			 if(para_save.R_Current>10000||para_save.R_Current<50)
#else
			if(para_save.R_Current>3000||para_save.R_Current<50)								
#endif
				{
					para_save.T_Current=0;
				 }
				else
				{
					para_save.T_Current=get_temp(para_save.R_Current);
				 }
				if(para_save.O2PP>=0)
		 	     para_save.lambda=get_lambda_O2PP((double )para_save.O2PP /100);	
        else				
         	 para_save.lambda	=	get_lambda_Ip(para_save.pump_current/10);
		  sentToTFT();
		}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
