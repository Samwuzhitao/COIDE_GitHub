#include "led.h" 
#include "stm32f4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//��ʼ��PG0Ϊ�����.��ʹ������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOGʱ��

  //GPIOG��ʼ������
  GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;//��ͨ���ģʽ
  GPIO_InitStructure.Pull = GPIO_PULLUP;//�������
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;//100MHz��

  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
}






