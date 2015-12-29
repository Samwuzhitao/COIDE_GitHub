#include "led.h" 
#include "stm32f4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//初始化PG0为输出口.并使能这个口的时钟		    
//LED IO初始化
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOG时钟

  //GPIOG初始化设置
  GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;//普通输出模式
  GPIO_InitStructure.Pull = GPIO_PULLUP;//推挽输出
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;//100MHz化

  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
}






