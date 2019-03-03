#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "../HARDWARE/TFT1_8.h"
 
//STM32F4工程-库函数版本
//淘宝店铺：http://mcudev.taobao.com


int main(void)
{
	u32 t=0;
	//GPIO_TypeDef gpio_typedef;
	//GPIO_InitTypeDef gpio_initypedef;
	
	uart_init(115200);
	delay_init(84);
	TFT1_8_init();

	while(1)
	{
		//LCD_MM();
		printf("t:%d\r\n",t);
		delay_ms(500);
		t++;
	}
}

/*
//STM32F4工程-库函数版本
//淘宝店铺：http://mcudev.taobao.com
*/
