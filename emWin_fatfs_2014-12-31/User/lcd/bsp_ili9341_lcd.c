/*
*********************************************************************************************************
*
*	模块名称 : TFT液晶显示器驱动模块
*	文件名称 : bsp_ili9341_lcd.c
*	版    本 : V1.1
*	说    明 : 驱动芯片的访问地址为:  0x60000000
*				
*	修改记录 :
*		版本号    日期          作者                 说明
*		v1.0    2011-10-31   WildFire Team  ST固件库版本 V3.5.0版本。
*       v1.1    2011-11-07   WildFire Team  修正竖屏显示，通过直接将X,Y都设置为递减，加速GUI显示。
*
*	Copyright (C), 2012-2013, WildFire Team
*
*********************************************************************************************************
*/
#include "bsp_ili9341_lcd.h"


/* ILI9341寄存器地址 */
#define SWRESET 	0x01 //Software Reset
#define RDDIDIF 	0x04 //Read Display Identification Information
#define RDDST   	0x09 //Read Display Status
#define RDDPM   	0x0A //Read Display Power Mode
#define RDDMADCTL	0x0B //Read Display MADCTL
#define RDDCOLMOD	0x0C //Read Display Pixel Format
#define RDDIM		0x0D //Read Display Image Mode
#define RDDSM		0x0E //Read Display Signal Mode
#define RDDSDR		0x0F //Read Display Self-Diagnostic Result
#define SPLIN		0x10 //Enter Sleep Mode
#define SLPOUT		0x11 //Sleep Out
#define PTLON		0x12 //Partial Mode On
#define NORON		0x13 //Normal Display Mode On
#define DINVOFF		0x20 //Display Inversion OFF
#define DINVON		0x21 //Display Inversion ON
#define GAMSET      0x26 //Gamma Set
#define DISPOFF		0x28 //Display OFF
#define DISPON		0x29 //Display ON
#define CASET		0x2A //Column Address Set
#define PASET		0x2B //Page Address Set
#define RAMWR		0x2C //Memory Write
#define RGBSET		0x2D //Color Set
#define RAMRD       0x2E //Memory Read
#define PLTAR		0x30 //Partial Area
#define VSCRDEF		0x33 //Vertical Scrolling Definition
#define TEOFF		0x34 //Tearing Effect Line OFF
#define TEON		0x35 //Tearing Effect Line ON
#define MADCTL		0x36 //Memory Access Control
#define VSCRSADD	0x37 //Vertical Scrolling Start Address
#define IDMOFF		0x38 //Idle Mode OFF
#define IDMON		0x39 //Idle Mode ON
#define PIXSET		0x3A //Pixel Format Set
#define RAMWRC      0x3C //Write Memory Continue
#define RAMRDC		0x3E //Read  Memory Continue
#define STTS		0x44 //Set Tear Scanline 
#define GTSL		0x45 //Get Scan line
#define WRDISBV		0x51 //Write Display Brightness
#define RDDISBV		0x52 //Read Display Brightness Value
#define WRCTRLD		0x53 //Write Control Display
#define RDCTRLD		0x54 //Read Control Display
#define WRCABC		0x55 //Write Content Adaptive Brightness Control
#define RDCABC		0x56 //Read Content Adaptive Brightness Control
#define WRCABCMIN	0x5E //Write CABC Minimum Brightness
#define RDCABCMIN	0x5F //Read CABC Minimum Brightnes
#define RDID1		0xDA //Read ID1
#define RDID2		0xDB //Read ID2
#define RDID3		0xDC //Read ID3
#define IFMODE		0xB0 //Interface Mode Control
#define FRMCTR1		0xB1 //Frame Rate Control (In Normal Mode / Full colors
#define FRMCTR2		0xB2 //Frame Rate Control (In Idle Mode / 8l colors)
#define FRMCTR3		0xB3 //Frame Rate Control (In Partial Mode / Full colors)
#define INVTR		0xB4 //Display Inversion Control
#define PRCTR		0xB5 //Blanking Porch
#define DISCTRL		0xB6 //Display Function Control
#define ETMOD		0xB7 //Entry Mode Set
#define BKCTL1		0xB8 //Backlight Control 1 
#define BKCTL2		0xB9 //Backlight Control 2 
#define BKCTL3		0xBA //Backlight Control 3 
#define BKCTL4		0xBB //Backlight Control 4 
#define BKCTL5		0xBC //Backlight Control 5
#define BKCTL7		0xBE //Backlight Control 7
#define BKCTL8		0xBF //Backlight Control 8
#define PWCTRL1		0xC0 //Power Control 1
#define PWCTRL2		0xC1 //Power Control 2
#define VMCTRL1		0xC5 //VCOM Control 1
#define VMCTRL2		0xC7 //VCOM Control 2
#define NVMWR		0xD0 //NV Memory Write
#define NVMPKEY		0xD1 //NV Memory Protection Key
#define RDNVM		0xD2 //NV Memory Status Read
#define RDID4		0xD3 //Read ID4
#define PGAMCTRL	0xE0 //Positive Gamma Control
#define NGAMCTRL	0xE1 //Negative Gamma Correction
#define DGAMCTRL1	0xE2 //Digital Gamma Control 1
#define DGAMCTRL2	0xE3 //Digital Gamma Control 2
#define IFCTL		0xF6 //16bits Data Format Selection
#define PWCTLA		0xCB //Power control A 
#define PWCTLB		0xCF //Power control B 
#define DTIMCTLA	0xE8 //Driver timing control A 
#define DTIMCTLB	0xEA //Driver timing control B 
#define PWONSCTL	0xED //Power on sequence control 
#define EN3G		0xF2 //Enable_3G 
#define PRCTL		0xF7 //Pump ratio control 


/* 定义 */
#define LCD_ILI9341_CMD(index)       LCD_WR_REG(index)
#define LCD_ILI9341_Parameter(val)   LCD_WR_Data(val)

/* 定义LCD驱动器用到的延时 */
#define DEBUG_DELAY()   Delay(2000)


/*
*********************************************************************************************************
*	函 数 名: LCD_WR_REG
*	功能说明: 写寄存器地址函数
*	形    参: index 寄存器地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD_WR_REG(unsigned int index)
{
	ILI9341_REG = index;

}

/*
*********************************************************************************************************
*	函 数 名: LCD_WR_CMD
*	功能说明: 写寄存器数据函数
*	形    参: index 寄存器地址
*			  val   数据
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD_WR_CMD(unsigned int index,unsigned int val)
{	
	ILI9341_REG = index;	
	ILI9341_RAM = val;
}

/*
*********************************************************************************************************
*	函 数 名: LCD_WR_CMD
*	功能说明: 读GRAM数据-
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint16_t LCD_RD_data(void)	
{	
	uint16_t R=0, G=0, B=0 ;

	R = ILI9341_RAM; 	/*FIRST READ OUT DUMMY DATA*/
	R = ILI9341_RAM;  	/*READ OUT RED DATA  */
	B = ILI9341_RAM;  	/*READ OUT BLACK DATA*/
	G = ILI9341_RAM;  	/*READ OUT GREEN DATA*/
	
    return (((R>>11)<<11) | ((G>>10)<<5) | (B>>11)) ;
}

/*
*********************************************************************************************************
*	函 数 名: LCD_WR_Data
*	功能说明: 写16位数据函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD_WR_Data(unsigned int val)
{   
	ILI9341_RAM = val; 	
}
////////////////////////test/////
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
  /* Write 16-bit Index, then Write Reg */
  ILI9341_REG = LCD_Reg;
  /* Write 16-bit Reg */
  ILI9341_RAM = LCD_RegValue;
}
//
/**
  * @brief  Prepare to write to the LCD RAM.
  * @param  None
  * @retval None
  */
void LCD_WriteRAM_Prepare(void)
{
  ILI9341_REG =0x22 ;//LCD_REG_34
}
/**
  * @brief  Writes to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(uint16_t RGB_Code)
{
  /* Write 16-bit GRAM Reg */
  ILI9341_RAM = RGB_Code;
}
/////////////////////////////////
/*
*********************************************************************************************************
*	函 数 名: Delay
*	功能说明: 延时
*	形    参: nCount 延时计数
*	返 回 值: 无
*********************************************************************************************************
*/
static void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/*
*********************************************************************************************************
*	函 数 名: LCD_FSMCConfig
*	功能说明: FSMC初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD_FSMCConfig(void)
{	
	
	 FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;	
    FSMC_NORSRAMTimingInitTypeDef  p; 
    
    
    p.FSMC_AddressSetupTime = 0x01;	 //地址建立时间	//为ewim修改，原来的驱动数值为2  减小可提高速度
    p.FSMC_AddressHoldTime = 0x00;	 //地址保持时间
    p.FSMC_DataSetupTime = 0x02;		 //数据建立时间	//为ewim修改，原来的驱动数值为5  减小可提高速度
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;
    p.FSMC_AccessMode = FSMC_AccessMode_B;	 // 一般使用模式B来控制LCD
    
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    //FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
		FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;  
    
    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
    
    /* 使能 FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
		

	
}

/*
*********************************************************************************************************
*	函 数 名: LCD_CtrlLinesConfig
*	功能说明: TFT的GPIO初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD_CtrlLinesConfig(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能FSMC时钟*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    
    /* 使能FSMC对应相应管脚时钟*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD 
                            | RCC_APB2Periph_GPIOE 
	                          | RCC_APB2Periph_GPIOB,ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置LCD复位控制管脚*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 
    GPIO_Init(GPIOE, &GPIO_InitStructure);  		   
    
    /* 配置FSMC相对应的数据线,FSMC-D0~D15: PD 14 15 0 1,PE 7 8 9 10 11 12 13 14 15,PD 8 9 10*/	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | 
                                  GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                  GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure); 
    
		/* 配置FSMC相对应的控制线
		 * PD4-FSMC_NOE   :LCD-RD
		 * PD5-FSMC_NWE   :LCD-WR
		 * PD7-FSMC_NE1   :LCD-CS
		 * PD11-FSMC_A16  :LCD-DC
		*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
    
    /* 配置LCD背光控制管脚*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
}

/*
*********************************************************************************************************
*	函 数 名: LCD_Reset
*	功能说明: 复位
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD_Reset(void)
{			
	/*复位 LCD*/	 
		GPIO_ResetBits(GPIOE, GPIO_Pin_1);	 //低电平复位
    Delay(0xAFFf<<2); 					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1);		 	 
    Delay(0xAFFf<<2); 	

}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitLCD
*	功能说明: ili9341初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitLCD(void)
{
	uint16_t i;
	// 配置LCD控制口线GPIO //
	LCD_CtrlLinesConfig();

	// 配置FSMC接口，数据总线 //
	LCD_FSMCConfig();

	// 复位 //
	LCD_Reset();

Delay(60);
//////ili93//25
   // printf("\n\r This LCD is ili%x.", DeviceIdCode);
    	LCD_WriteReg(0x00e3,0x3008);
    	LCD_WriteReg(0x00e7,0x0012);
    	LCD_WriteReg(0x00ef,0x1231);//Set the internal vcore voltage
 // 		LCD_WriteReg(0x00e7,0x0010);      
        LCD_WriteReg(0x0000,0x0001);  			//start internal osc
        LCD_WriteReg(0x0001,0x0100);     
        LCD_WriteReg(0x0002,0x0700); 				//power on sequence                     
        LCD_WriteReg(0x0003,(1<<12)|(1<<5)|(1<<4) ); 	//65K 
        LCD_WriteReg(0x0004,0x0000);                                   
        LCD_WriteReg(0x0008,0x0207);	           
        LCD_WriteReg(0x0009,0x0000);         
        LCD_WriteReg(0x000a,0x0000); 				//display setting         
        LCD_WriteReg(0x000c,0x0001);				//display setting          
        LCD_WriteReg(0x000d,0x0000); 				//0f3c          
        LCD_WriteReg(0x000f,0x0000);
        //Power On sequence //
        LCD_WriteReg(0x0010,0x0000);   
        LCD_WriteReg(0x0011,0x0007);
        LCD_WriteReg(0x0012,0x0000);                                                                 
        LCD_WriteReg(0x0013,0x0000);                 
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0010,0x1590);   
        LCD_WriteReg(0x0011,0x0227);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0012,0x009c);                  
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0013,0x1900);   
        LCD_WriteReg(0x0029,0x0023);
        LCD_WriteReg(0x002b,0x000e);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0020,0x0000);                                                            
        LCD_WriteReg(0x0021,0x0000);           
        ///////////////////////////////////////////////////////      
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0030,0x0007); 
        LCD_WriteReg(0x0031,0x0707);   
        LCD_WriteReg(0x0032,0x0006);
        LCD_WriteReg(0x0035,0x0704);
        LCD_WriteReg(0x0036,0x1f04); 
        LCD_WriteReg(0x0037,0x0004);
        LCD_WriteReg(0x0038,0x0000);        
        LCD_WriteReg(0x0039,0x0706);     
        LCD_WriteReg(0x003c,0x0701);
        LCD_WriteReg(0x003d,0x000f);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        LCD_WriteReg(0x0050,0x0000);        
        LCD_WriteReg(0x0051,0x00ef);   
        LCD_WriteReg(0x0052,0x0000);     
        LCD_WriteReg(0x0053,0x013f);
        LCD_WriteReg(0x0060,0xa700);        
        LCD_WriteReg(0x0061,0x0001); 
        LCD_WriteReg(0x006a,0x0000);
        LCD_WriteReg(0x0080,0x0000);
        LCD_WriteReg(0x0081,0x0000);
        LCD_WriteReg(0x0082,0x0000);
        LCD_WriteReg(0x0083,0x0000);
        LCD_WriteReg(0x0084,0x0000);
        LCD_WriteReg(0x0085,0x0000);
      
        LCD_WriteReg(0x0090,0x0010);     
        LCD_WriteReg(0x0092,0x0600);  
    //	if(DeviceIdCode==0x9328)
      //  {   
       //        LCD_WriteReg(0x0093,0x0003);
        //       LCD_WriteReg(0x0095,0x0110);
        //       LCD_WriteReg(0x0097,0x0000);        
        //       LCD_WriteReg(0x0098,0x0000);  
       //  }
         //display on sequence     
        LCD_WriteReg(0x0007,0x0133);
    
        LCD_WriteReg(0x0020,0x0000);                                                            
        LCD_WriteReg(0x0021,0x0000);
				Delay(60);
///////////////
LCD9341_Clear(0xFFFF);//

}

/*
*********************************************************************************************************
*	函 数 名: Set_direction
*	功能说明: 设置屏幕座标
*	形    参: _usX1      行座标
*             _usY1      列座标
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_SetCursor(uint16_t _usX1 , uint16_t _usY1)	
{				
	/*	
	//竖屏//
	LCD_ILI9341_CMD(0X2A); 				 // 设置X坐标 //
	LCD_ILI9341_Parameter(_usX1>>8);	 // 先高8位，然后低8位 //
	LCD_ILI9341_Parameter(_usX1&0xff);	 // 设置起始点和结束点//
	LCD_ILI9341_Parameter(_usX1>>8);
	LCD_ILI9341_Parameter(_usX1&0xff);

    LCD_ILI9341_CMD(0X2B); 			     // 设置Y坐标//
	LCD_ILI9341_Parameter(_usY1>>8);
	LCD_ILI9341_Parameter(_usY1&0xff);
	LCD_ILI9341_Parameter(_usY1>>8);
	LCD_ILI9341_Parameter(_usY1&0xff);  
		*/ 
    //LCD_WriteReg(0x0020,_usY1); // 行
  	//LCD_WriteReg(0x0021,0x13f-_usX1); // 列	
	 //// LCD_WriteReg(0x0021,_usX1); // 列
	
	 LCD_WriteReg(R32,_usX1 );//Xpos
   LCD_WriteReg(R33, _usY1);//Ypos
}

/*
*********************************************************************************************************
*	函 数 名: LCD_Clear
*	功能说明: 将屏幕填充成指定的颜色，如清屏，则填充 0xffff
*	形    参: dat      填充值
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_Clear(uint16_t _usColor)	
{/*
    uint32_t n;

	////////////////竖屏////////////////
	LCD_ILI9341_CMD(0X2A); 			  // 设置X坐标 //
	LCD_ILI9341_Parameter(0);		  // 先高8位，然后低8位 //
	LCD_ILI9341_Parameter(0);		  // 设置起始点和结束点//
	LCD_ILI9341_Parameter(239>>8);
	LCD_ILI9341_Parameter(239&0xff);
	
	LCD_ILI9341_CMD(0X2B); 			 // 设置Y坐标//
	LCD_ILI9341_Parameter(0);
	LCD_ILI9341_Parameter(0);
	LCD_ILI9341_Parameter(319>>8);
	LCD_ILI9341_Parameter(319&0xff);
		
	LCD_ILI9341_CMD(0x2c);
	for(n=0; n<320*240; n++)
	{
		LCD_WR_Data(_usColor);
	} 
	*/
	u32 index=0;
  //ili9320_SetCursor(0,0);
  //LCD9341_SetCursor(0,0);	
	LCD9341_SetCursor(0,0x013F);	
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

 
  for(index=0;index<76800;index++)
   {
     
      //LCD_Write(Color);
		 LCD_WriteRAM(_usColor);
      
   }
}

/*
*********************************************************************************************************
*	函 数 名: LCD9341_GetPoint
*	功能说明: 获取指定座标的颜色值
*	形    参: _usX1      行座标
*             _usY1      列座标
*	返 回 值: 无
*********************************************************************************************************
*/
uint16_t LCD9341_GetPoint(uint16_t _usX1 , uint16_t _usY1)
{ 
    uint16_t temp;

	LCD9341_SetCursor(_usX1,_usY1);
   // LCD_ILI9341_CMD(0x2e); //0x2e        // 读数据 //
	LCD_ILI9341_CMD(0x22); 
	temp=LCD_RD_data();
    return (temp);
}

/*
*********************************************************************************************************
*	函 数 名: LCD9341_SetPoint
*	功能说明: 在指定座标画点
*	形    参: _usX1      行座标
*             _usY1      列座标
*             _usColor   点的颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_SetPoint(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usColor)	
{   	
	
	LCD9341_SetCursor(_usX1,_usY1);
	   //[下面代码，部分TFT需要，部分可以不用
  // LCD_WR_CmdPar(0x0050, x);//水平 GRAM起始位置
  // LCD_WR_CmdPar(0x0051, x);//水平GRAM终止位置
  // LCD_WR_CmdPar(0x0052, y);//垂直GRAM起始位置
  // LCD_WR_CmdPar(0x0053, y);//垂直GRAM终止位置
	//LCD_ILI9341_CMD(0x2c);//0x2c	         // 写数据 //
	LCD_ILI9341_CMD(0x22);
	LCD_WR_Data(_usColor);

}

/*
*********************************************************************************************************
*	函 数 名: LCD_SetDispWin
*	功能说明: 设置窗口区域
*	形    参: _usX1    窗户起点
           	  _usY1	   窗户起点
              _usX2    窗户结束点 
              _usY2    窗户结束点
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_SetDispHV(uint16_t _usX1, uint16_t _usY1, uint16_t _usX2, uint16_t _usY2, uint8_t _ucHV)
{                    
	/* 默认的扫描方式 */
	if(_ucHV == 1)
    {
		LCD_ILI9341_CMD(0x36); 
	    LCD_ILI9341_Parameter(0xc8);    
	    DEBUG_DELAY();

		LCD_ILI9341_CMD(0X2A); 				  /* 设置X坐标 */
		LCD_ILI9341_Parameter(_usX1>>8);	  /* 设置高8位和低8位*/
		LCD_ILI9341_Parameter(_usX1&0xff);	  /* 设置起始点和结束点*/
		LCD_ILI9341_Parameter(_usX2>>8);	
		LCD_ILI9341_Parameter(_usX2&0xff);
		
		LCD_ILI9341_CMD(0X2B); 				  /* 设置Y坐标*/
		LCD_ILI9341_Parameter(_usY1>>8);   
		LCD_ILI9341_Parameter(_usY1&0xff);
		LCD_ILI9341_Parameter(_usY2>>8);   
		LCD_ILI9341_Parameter(_usY2&0xff);
	
		LCD_ILI9341_CMD(0x2c);		
	}
	else
	{
		LCD_ILI9341_CMD(0x36); 
	    LCD_ILI9341_Parameter(0x08);    
	    DEBUG_DELAY();

		LCD_ILI9341_CMD(0X2A); 				  /* 设置X坐标 */
		LCD_ILI9341_Parameter(_usX1>>8);	  /* 设置高8位和低8位*/
		LCD_ILI9341_Parameter(_usX1&0xff);	  /* 设置起始点和结束点*/
		LCD_ILI9341_Parameter(_usX2>>8);	
		LCD_ILI9341_Parameter(_usX2&0xff);
		
		LCD_ILI9341_CMD(0X2B); 				  /* 设置Y坐标*/
		LCD_ILI9341_Parameter(_usY1>>8);   
		LCD_ILI9341_Parameter(_usY1&0xff);
		LCD_ILI9341_Parameter(_usY2>>8);   
		LCD_ILI9341_Parameter(_usY2&0xff);
	
		LCD_ILI9341_CMD(0x2c);
	}	
	     
}

/*
*********************************************************************************************************
*	函 数 名: LCD_SetDispWin
*	功能说明: 设置窗口区域
*	形    参: _usX1    窗户起点
           	  _usY1	   窗户起点
              _usX2    窗户结束点 
              _usY2    窗户结束点
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_SetDispWin(uint16_t _usX1, uint16_t _usY1, uint16_t _usX2, uint16_t _usY2)
{                    

	LCD_ILI9341_CMD(0X2A); 				  /* 设置X坐标 */
	LCD_ILI9341_Parameter(_usX1>>8);	  /* 设置高8位和低8位*/
	LCD_ILI9341_Parameter(_usX1&0xff);	  /* 设置起始点和结束点*/
	LCD_ILI9341_Parameter(_usX2>>8);	
	LCD_ILI9341_Parameter(_usX2&0xff);
	
	LCD_ILI9341_CMD(0X2B); 				  /* 设置Y坐标*/
	LCD_ILI9341_Parameter(_usY1>>8);   
	LCD_ILI9341_Parameter(_usY1&0xff);
	LCD_ILI9341_Parameter(_usY2>>8);   
	LCD_ILI9341_Parameter(_usY2&0xff);

	LCD_ILI9341_CMD(0x2c);     
}

/*
*********************************************************************************************************
*	函 数 名: LCD9341_DrawHLine
*	功能说明: 画水平线 用UCGUI的接口函数
*	形    参: X,Y的坐标和颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_DrawHLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usColor)
{
	uint16_t i;

#if 1
	
	LCD9341_SetDispWin(_usX1, _usY1, _usX2,_usY1);

	for (i = 0; i <_usX2-_usX1+1; i++)
	{
		ILI9341_RAM = _usColor;
	}

#else

	for (i = _usX1; i <=_usX2; i++)
	{	
		LCD9341_SetPoint(i, _usY1, _usColor);	
	}

#endif

}

/*
*********************************************************************************************************
*	函 数 名: LCD9341_DrawVLine
*	功能说明: 画垂直平线 用UCGUI的接口函数
*	形    参: X,Y的坐标和颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_DrawVLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usY2 , uint16_t _usColor)
{
	uint16_t i;
#if 1

	LCD9341_SetDispWin(_usX1, _usY1,_usX1,_usY2);

	for (i = 0; i <_usY2-_usY1+1; i++)
	{
		ILI9341_RAM = _usColor;
	}
#else

	for (i = _usY1; i <=_usY2; i++)
	{	
		LCD9341_SetPoint(_usX1, i, _usColor);	
	}

#endif
}

/*
*********************************************************************************************************
*	函 数 名: LCD9341_DrawVLine
*	功能说明: 画矩形填充 用UCGUI的接口函数
*	形    参: X,Y的坐标和颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_FillRect(uint16_t _usX1 , uint16_t _usY1 ,  uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor)
{                    
   uint32_t n, temp;
   
   LCD9341_SetDispWin(_usX1, _usY1,_usX2,_usY2); 
    
   temp = (u32)(_usX2-_usX1+1)*(_usY2 -_usY1+1);
       
   for(n=0; n<temp; n++)
   {
		ILI9341_RAM =_usColor;
   }
	 	  
}

/*
*********************************************************************************************************
*	函 数 名: LCD9341_DrawVLine
*	功能说明: 画矩形填充 用UCGUI的接口函数
*	形    参: X,Y的坐标和颜色
*	返 回 值: 无
*********************************************************************************************************
*/
void LCD9341_DrawJPG(uint16_t _usX1 , uint16_t _usY1 ,  uint16_t _usX2 , uint16_t _usY2 , uint16_t *_ptr)
{                    
   uint32_t n, temp;
   
   LCD9341_SetDispWin(_usX1, _usY1,_usX2,_usY2); 
    
   temp = (u32)(_usX2-_usX1+1)*(_usY2 -_usY1+1);
       
   for(n=0; n<temp; n++)
   {
		ILI9341_RAM = *_ptr++;
   }
	 	  
}
