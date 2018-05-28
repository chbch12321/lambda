#define SPI_GLO
#include "main.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
void delay_us(uint8_t nus)
{
	while(nus--);
}
uint8_t SPI_Send(uint8_t data)
{
		uint8_t i=0;
		uint8_t read=0;
		for(i=0;i<8;i++)
			{	delay_us(CLK_SPI);
				SET_SPI_CLK(GPIO_PIN_SET);
				delay_us(20);
					read<<=1;
				if(GET_SPI_MISO==SET)
					{
					read+=1;
					}

				if((data&0x80)==0x80)
					{
					  SET_SPI_MOSI(GPIO_PIN_SET);
						 
					}
				else
					{
						SET_SPI_MOSI(GPIO_PIN_RESET);
					}
               data<<=1;
				delay_us(CLK_SPI);
				SET_SPI_CLK(GPIO_PIN_RESET);
			}
		return read;
}
  
void SPI_Write_Byte(unsigned char add,unsigned char data)
{ 
	  uint8_t stat=0;
		delay_us(CLK_SPI);
		SET_SPI_NSS(GPIO_PIN_RESET);
		stat=SPI_Send(add);
		stat=SPI_Send(data);
		SET_SPI_NSS(GPIO_PIN_SET);
}
uint8_t SPI_Read_Byte(unsigned char byte)
{
	 uint8_t data=0;
	 uint8_t stat=0;
	 delay_us(CLK_SPI);
	 SET_SPI_NSS(GPIO_PIN_RESET);
	 stat=SPI_Send(byte);
	 data=SPI_Send(0x00);
	 SET_SPI_NSS(GPIO_PIN_SET);
	 return data;
}

/*
以下为CJ125相关代码
*/
 

//读寄存器
uint8_t CJ125readREG(uint8_t reg )
{
	return SPI_Read_Byte(reg);
}
//写寄存器
uint8_t CJ125writeREG(uint8_t reg ,uint8_t value)
{
	SPI_Write_Byte( reg , value);
	return 1;
}

uint8_t CJ125readStatus(void)
{
	reg_status = CJ125readREG(0x78);
	return 1;
}
uint8_t CJ125readVersion(void)
{
	return SPI_Read_Byte(0x48);
}
uint8_t CJ125readALL()
{	
		reg1 = CJ125readREG(0x6c);
		reg2 = CJ125readREG(0x7e);
		reg_status = CJ125readREG(0x78);
	return 1;
}
uint8_t CJ125writeALL()
{
	SPI_Write_Byte(0x56,reg1);
	SPI_Write_Byte(0x5a,reg2);
	return 1;
}


