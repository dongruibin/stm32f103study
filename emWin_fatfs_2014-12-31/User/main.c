/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   emWin�ļ�ϵͳ
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO-MINI STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "stm32f10x.h"
#include "bsp_led.h"
#include "GUI.h"
#include "diskio.h"
#include "bsp_touch.h"
#include "bsp_SysTick.h"
#include "bsp_usart1.h"
#include "bsp_sdio_sdcard.h"


extern void Touch_MainTask(void);
extern void Fatfs_MainTask(void);

/**
  * @brief  ������
  * @param  ��  
  * @retval ��
  */
int main(void)
{	
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();	
	
	/* ��ʼ������ */
	Touch_Init();
	
	/* ��ʼ����ʱ�� */
	SysTick_Init();
	
	/*��ʼ��sd��*/
	disk_initialize(0);  
	
	/*CRC��emWinû�й�ϵ��ֻ������Ϊ�˿�ı��������ģ�����STemWin�Ŀ�ֻ������ST��оƬ���棬���оƬ���޷�ʹ�õġ� */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	
	/* ��ʼ��GUI */
	GUI_Init();
	
	/* ��ʼ������*/
	USART1_Config();
	
	DEBUG("\r\n wildfire ISO board emWin test \r\n");
	
	GUI_Delay (20);
	
	#if 0
	/* ����У׼demo */
	Touch_MainTask();
	#else
	Fatfs_MainTask();
	//LCD9341_Clear(0xF800);//RED
	//
	/* ��ʾ���� */
	//GUI_DispString("wildfire ISO board emWin test!");
	/*�趨λ�á�ָ��������ʾ����*/
	//GUI_GotoXY(5, 10);
  //GUI_SetFont(&GUI_Font8x16);
  //GUI_DispString("Hello world!");
	//
	while(1)
	{
		
    GUI_Delay(1000);
	}
	#endif

}


/*********************************************END OF FILE**********************/
