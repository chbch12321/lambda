
#ifndef SPI_H_
#define SPI_H_
#include "main.h"
#include "stdint.h"
#include "gpio.h"

#define MISO_Pin GPIO_PIN_5
#define MISO_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_6
#define SCK_GPIO_Port GPIOA
#define SS_Pin GPIO_PIN_7
#define SS_GPIO_Port GPIOA


#define MOSI_Pin GPIO_PIN_14
#define MOSI_GPIO_Port GPIOB



#define CLK_SPI 100
#define PORT_SPI GPIOA
#define SPI_CLK SCK_Pin
#define SPI_NSS SS_Pin
#define SPI_MISO MISO_Pin
#define SPI_MOSI MOSI_Pin

 
 
#define SET_SPI_CLK(x)  HAL_GPIO_WritePin(PORT_SPI , SPI_CLK, x)
#define SET_SPI_NSS(x)  HAL_GPIO_WritePin(PORT_SPI , SPI_NSS, x)
#define SET_SPI_MOSI(x)  HAL_GPIO_WritePin(MOSI_GPIO_Port , SPI_MOSI, x)
#define GET_SPI_MISO HAL_GPIO_ReadPin(PORT_SPI , SPI_MISO)


extern void Spi_Init(void);
extern void SPI_Write_Byte(unsigned char add,unsigned char data);
extern uint8_t SPI_Send(uint8_t data);
extern uint8_t SPI_Read_Byte(unsigned char byte);
 
//¶Á¼Ä´æÆ÷
extern uint8_t CJ125readREG(uint8_t reg );
//Ð´¼Ä´æÆ÷
extern uint8_t CJ125writeREG(uint8_t reg ,uint8_t value);
extern uint8_t CJ125readStatus(void);
extern uint8_t CJ125readVersion(void);
uint8_t CJ125readALL(void);
uint8_t CJ125writeALL(void);
#ifdef SPI_GLO
uint8_t reg1=0;
uint8_t reg2=0;
uint8_t reg_status=0;
#else
extern uint8_t reg1;
extern uint8_t reg2;
extern uint8_t reg_status;
#endif
#endif 

