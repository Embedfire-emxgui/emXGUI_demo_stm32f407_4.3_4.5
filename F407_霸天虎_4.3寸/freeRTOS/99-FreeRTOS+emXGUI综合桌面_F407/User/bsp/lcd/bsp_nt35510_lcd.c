/**
  ******************************************************************************
  * @file    bsp_NT35510_lcd.c
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   NT35510液晶屏驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./lcd/bsp_nt35510_lcd.h"

__inline void                 NT35510_Write_Cmd           ( uint16_t usCmd );
__inline void                 NT35510_Write_Data          ( uint16_t usData );
__inline uint16_t             NT35510_Read_Data           ( void );
static void                   NT35510_Delay               ( __IO uint32_t nCount );
static void                   NT35510_GPIO_Config         ( void );
static void                   NT35510_FSMC_Config         ( void );
static void                   NT35510_REG_Config          ( void );
static __inline void          NT35510_FillColor           ( uint32_t ulAmout_Point, uint16_t usColor );

/*=========================================================================================*/


#define	LCD_CMD_ADDR	FSMC_Addr_NT35510_CMD
#define	LCD_DAT_ADDR	FSMC_Addr_NT35510_DATA

#define	LCD_CMD	(*(volatile u16*)(LCD_CMD_ADDR))
#define	LCD_DAT	(*(volatile u16*)(LCD_DAT_ADDR))


/*=========================================================================================*/

static void delay_ms(unsigned int t)
{
	volatile int i;
	while(t-- > 0)
	{
		for(i=100;i>0;i--);
	}
}

static	INLINE void write_cmd(u16 a)
{
	LCD_CMD =a;
}

static	INLINE void write_dat(u16 a)
{
 	LCD_DAT =a;
}

/*=========================================================================================*/

#define	__lcd_write_cmd(cmd)    LCD_CMD =cmd
#define	__lcd_write_dat(dat) 	LCD_DAT =dat
#define	__lcd_read_dat()		LCD_DAT

#define	__lcd_map_rgb(r,g,b)	RGB565(r>>3,g>>2,b>>3)


static INLINE void __lcd_write_start(u16 sx,u16 sy,u16 ex,u16 ey)
{
	__lcd_write_cmd(CMD_SetCoordinateX);
	__lcd_write_dat(sx>>8);
  __lcd_write_cmd(CMD_SetCoordinateX + 1);
	__lcd_write_dat(sx);
  __lcd_write_cmd(CMD_SetCoordinateX + 2);
	__lcd_write_dat(ex>>8);
  __lcd_write_cmd(CMD_SetCoordinateX + 3);
	__lcd_write_dat(ex);

  __lcd_write_cmd(CMD_SetCoordinateY);
	__lcd_write_dat(sy>>8);
  __lcd_write_cmd(CMD_SetCoordinateY + 1);
	__lcd_write_dat(sy);
  __lcd_write_cmd(CMD_SetCoordinateY + 2);
	__lcd_write_dat(ey>>8);
  __lcd_write_cmd(CMD_SetCoordinateY + 3);
	__lcd_write_dat(ey);

	__lcd_write_cmd(CMD_SetPixel);
}


static INLINE void __lcd_read_start(u16 sx,u16 sy,u16 ex,u16 ey)
{
	volatile u16 c;

	__lcd_write_cmd(CMD_SetCoordinateX);
	__lcd_write_dat(sx>>8);
  __lcd_write_cmd(CMD_SetCoordinateX + 1);
	__lcd_write_dat(sx);
  __lcd_write_cmd(CMD_SetCoordinateX + 2);
	__lcd_write_dat(ex>>8);
  __lcd_write_cmd(CMD_SetCoordinateX + 3);
	__lcd_write_dat(ex);

	__lcd_write_cmd(CMD_SetCoordinateY);
	__lcd_write_dat(sy>>8);
  __lcd_write_cmd(CMD_SetCoordinateY + 1);
	__lcd_write_dat(sy);
  __lcd_write_cmd(CMD_SetCoordinateY + 2);
	__lcd_write_dat(ey>>8);
  __lcd_write_cmd(CMD_SetCoordinateY + 3);
	__lcd_write_dat(ey);

	__lcd_write_cmd(0x2E00);
	c =__lcd_read_dat(); //DummyRead
	for(c=0;c<2;c++);
}

static INLINE	void __lcd_write_rgb(U8 r,U8 g,U8 b)
{
	__lcd_write_dat(__lcd_map_rgb(r,g,b));
}


static INLINE u16 __lcd_read_pixel(void)
{

	u16 c1,c2;

	c1 =__lcd_read_dat(); //RG
	c2 =__lcd_read_dat(); //BX

	return	__lcd_map_rgb(c1>>8,c1,c2>>8);
}

static INLINE void __lcd_read_rgb(u8 *r,u8 *g,u8 *b)
{
	u16 c1,c2;

	c1 =__lcd_read_dat(); //RG
	c2 =__lcd_read_dat(); //BX

	*r =c1>>8;
	*g =c1;
	*b =c2>>8;
}



/*=========================================================================================*/

/*=========================================================================================*/

BOOL	LCD_SetRotate(int rotate)
{

	switch(rotate)
	{
		case ROTATE_0:
			write_cmd(0x3600); // Memory Access Control
			write_dat((0<<7)|(1<<6)|(0<<5)|(0<<3)|(0<<2)|(0<<1));	// 0
			return TRUE;
			////

		case ROTATE_90:
			write_cmd(0x3600); // Memory Access Control
			write_dat((0<<7)|(1<<6)|(1<<5)|(0<<3)|(0<<2)|(0<<1));	// 90
			return TRUE;
			////

		case ROTATE_180:
			write_cmd(0x3600); // Memory Access Control
			write_dat((1<<7)|(0<<6)|(0<<5)|(0<<3)|(0<<2)|(1<<1));	// 180
			return TRUE;
			////

		case ROTATE_270:
			write_cmd(0x3600); // Memory Access Control
			write_dat((1<<7)|(1<<6)|(1<<5)|(0<<3)|(0<<2)|(1<<1));	// 270
			return TRUE;
			////

		default:
			return FALSE;
	}
}

/**
  * @brief  简单延时函数
  * @param  nCount ：延时计数值
  * @retval 无
  */	
static void Delay ( __IO uint32_t nCount )
{
  for ( ; nCount != 0; nCount -- );
	
}

/**
  * @brief  向NT35510写入命令
  * @param  usCmd :要写入的命令（表寄存器地址）
  * @retval 无
  */	
__inline void NT35510_Write_Cmd ( uint16_t usCmd )
{
	* ( __IO uint16_t * ) ( FSMC_Addr_NT35510_CMD ) = usCmd;
	
}


/**
  * @brief  向NT35510写入数据
  * @param  usData :要写入的数据
  * @retval 无
  */	
__inline void NT35510_Write_Data ( uint16_t usData )
{
	* ( __IO uint16_t * ) ( FSMC_Addr_NT35510_DATA ) = usData;
	
}


/**
  * @brief  从NT35510读取数据
  * @param  无
  * @retval 读取到的数据
  */	
__inline uint16_t NT35510_Read_Data ( void )
{
	return ( * ( __IO uint16_t * ) ( FSMC_Addr_NT35510_DATA ) );
	
}


/**
  * @brief  用于 NT35510 简单延时函数
  * @param  nCount ：延时计数值
  * @retval 无
  */	
static void NT35510_Delay ( __IO uint32_t nCount )
{
  for ( ; nCount != 0; nCount -- );
	
}


/**
  * @brief  初始化NT35510的IO引脚
  * @param  无
  * @retval 无
  */
static void NT35510_GPIO_Config ( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能FSMC对应相应管脚时钟*/
	RCC_AHB1PeriphClockCmd ( 	
													/*控制信号*/
													NT35510_CS_CLK|NT35510_DC_CLK|NT35510_WR_CLK|
													NT35510_RD_CLK	|NT35510_BK_CLK|NT35510_RST_CLK|
													/*数据信号*/
													NT35510_D0_CLK|NT35510_D1_CLK|	NT35510_D2_CLK | 
													NT35510_D3_CLK | NT35510_D4_CLK|NT35510_D5_CLK|
													NT35510_D6_CLK | NT35510_D7_CLK|NT35510_D8_CLK|
													NT35510_D9_CLK | NT35510_D10_CLK|NT35510_D11_CLK|
													NT35510_D12_CLK | NT35510_D13_CLK|NT35510_D14_CLK|
													NT35510_D15_CLK	, ENABLE );
		
	
	/* 配置FSMC相对应的数据线,FSMC-D0~D15 */	
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_InitStructure.GPIO_Pin = NT35510_D0_PIN; 
    GPIO_Init(NT35510_D0_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D0_PORT,NT35510_D0_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D1_PIN; 
    GPIO_Init(NT35510_D1_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D1_PORT,NT35510_D1_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D2_PIN; 
    GPIO_Init(NT35510_D2_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D2_PORT,NT35510_D2_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D3_PIN; 
    GPIO_Init(NT35510_D3_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D3_PORT,NT35510_D3_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D4_PIN; 
    GPIO_Init(NT35510_D4_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D4_PORT,NT35510_D4_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D5_PIN; 
    GPIO_Init(NT35510_D5_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D5_PORT,NT35510_D5_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D6_PIN; 
    GPIO_Init(NT35510_D6_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D6_PORT,NT35510_D6_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D7_PIN; 
    GPIO_Init(NT35510_D7_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D7_PORT,NT35510_D7_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D8_PIN; 
    GPIO_Init(NT35510_D8_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D8_PORT,NT35510_D8_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D9_PIN; 
    GPIO_Init(NT35510_D9_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D9_PORT,NT35510_D9_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D10_PIN; 
    GPIO_Init(NT35510_D10_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D10_PORT,NT35510_D10_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D11_PIN; 
    GPIO_Init(NT35510_D11_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D11_PORT,NT35510_D11_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D12_PIN; 
    GPIO_Init(NT35510_D12_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D12_PORT,NT35510_D12_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D13_PIN; 
    GPIO_Init(NT35510_D13_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D13_PORT,NT35510_D13_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D14_PIN; 
    GPIO_Init(NT35510_D14_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D14_PORT,NT35510_D14_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_D15_PIN; 
    GPIO_Init(NT35510_D15_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_D15_PORT,NT35510_D15_PinSource,FSMC_AF);

	/* 配置FSMC相对应的控制线
	 * FSMC_NOE   :LCD-RD
	 * FSMC_NWE   :LCD-WR
	 * FSMC_NE1   :LCD-CS
	 * FSMC_A0    :LCD-DC
	 */
    GPIO_InitStructure.GPIO_Pin = NT35510_RD_PIN; 
    GPIO_Init(NT35510_RD_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_RD_PORT,NT35510_RD_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_WR_PIN; 
    GPIO_Init(NT35510_WR_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_WR_PORT,NT35510_WR_PinSource,FSMC_AF);

    GPIO_InitStructure.GPIO_Pin = NT35510_CS_PIN; 
    GPIO_Init(NT35510_CS_PORT, &GPIO_InitStructure);   
    GPIO_PinAFConfig(NT35510_CS_PORT,NT35510_CS_PinSource,FSMC_AF);  

    GPIO_InitStructure.GPIO_Pin = NT35510_DC_PIN; 
    GPIO_Init(NT35510_DC_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(NT35510_DC_PORT,NT35510_DC_PinSource,FSMC_AF);
	

  /* 配置LCD复位RST控制管脚*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = NT35510_RST_PIN; 
	GPIO_Init ( NT35510_RST_PORT, & GPIO_InitStructure );
		
	/* 配置LCD背光控制管脚BK*/
	GPIO_InitStructure.GPIO_Pin = NT35510_BK_PIN; 
	GPIO_Init ( NT35510_BK_PORT, & GPIO_InitStructure );

}


 /**
  * @brief  LCD  FSMC 模式配置
  * @param  无
  * @retval 无
  */
static void NT35510_FSMC_Config ( void )
{
#if 1
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 	
	
	/* 使能FSMC时钟*/
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);

	//地址建立时间（ADDSET）为1个HCLK 5/168M=30ns
	readWriteTiming.FSMC_AddressSetupTime      = 0x04;	 //地址建立时间
	//数据保持时间（DATAST）+ 1个HCLK = 12/168M=72ns	
	readWriteTiming.FSMC_DataSetupTime         = 0x0f;	 //数据建立时间
	//选择控制的模式
	//模式B,异步NOR FLASH模式，与NT35510的8080时序匹配
	readWriteTiming.FSMC_AccessMode            = FSMC_AccessMode_B;	
	
	/*以下配置与模式B无关*/
	//地址保持时间（ADDHLD）模式A未用到
	readWriteTiming.FSMC_AddressHoldTime       = 0x00;	 //地址保持时间
	//设置总线转换周期，仅用于复用模式的NOR操作
	readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
	//设置时钟分频，仅用于同步类型的存储器
	readWriteTiming.FSMC_CLKDivision           = 0x00;
	//数据保持时间，仅用于同步型的NOR	
	readWriteTiming.FSMC_DataLatency           = 0x00;	

	
	FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_Bank1_NORSRAMx;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_NOR;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &readWriteTiming;  
	
	FSMC_NORSRAMInit ( & FSMC_NORSRAMInitStructure ); 
	
	
	/* 使能 FSMC_Bank1_NORSRAM3 */
	FSMC_NORSRAMCmd ( FSMC_Bank1_NORSRAMx, ENABLE );  

#else
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
	FSMC_NORSRAMTimingInitTypeDef  writeTiming;
	
	/* 使能FSMC时钟*/
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);
  
	readWriteTiming.FSMC_AddressSetupTime = 0x00;	 //地址建立时间（ADDSET）为2个HCLK 1/36M=27ns
	readWriteTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（ADDHLD）模式A未用到	
	readWriteTiming.FSMC_DataSetupTime = 0x0f;		 // 数据保存时间为16个HCLK,因为液晶驱动IC的读数据的时候，速度不能太快，尤其对1289这个IC。
	readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
	readWriteTiming.FSMC_CLKDivision = 0x00;
	readWriteTiming.FSMC_DataLatency = 0x00;
	readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 	

	writeTiming.FSMC_AddressSetupTime = 0x01;	 //地址建立时间（ADDSET）为1个HCLK  
	writeTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（A		
	writeTiming.FSMC_DataSetupTime = 0x04;		 ////数据保存时间为4个HCLK	
	writeTiming.FSMC_BusTurnAroundDuration = 0x00;
	writeTiming.FSMC_CLKDivision = 0x00;
	writeTiming.FSMC_DataLatency = 0x00;
	writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A  

	FSMC_NORSRAMInitStructure.FSMC_Bank           = FSMC_Bank1_NORSRAMx;// FSMC_Bank1_NORSRAM4;//  这里使用NE1 ，也就对应BTCR[6],[7]。
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // 不复用数据地址
	FSMC_NORSRAMInitStructure.FSMC_MemoryType     =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM 
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;  //存储器数据宽度为16bit	
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait      = FSMC_AsynchronousWait_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;  
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;	//  存储器写使能
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Enable; // 读写使用不同的时序
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming; //读写时序
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &writeTiming;  //写时序

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //初始化FSMC配置

	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAMx, ENABLE);  // 使能BANK1  
#endif
		
}


/**
 * @brief  初始化NT35510寄存器
 * @param  无
 * @retval 无
 */
/**
 * @brief  初始化NT35510寄存器
 * @param  无
 * @retval 无
 */
static void NT35510_REG_Config ( void )
{	
///NT35510-HSD43
  //PAGE1
  NT35510_Write_Cmd(0xF000);    NT35510_Write_Data(0x0055);
  NT35510_Write_Cmd(0xF001);    NT35510_Write_Data(0x00AA);
  NT35510_Write_Cmd(0xF002);    NT35510_Write_Data(0x0052);
  NT35510_Write_Cmd(0xF003);    NT35510_Write_Data(0x0008);
  NT35510_Write_Cmd(0xF004);    NT35510_Write_Data(0x0001);

  //Set AVDD 5.2V
  NT35510_Write_Cmd(0xB000);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB001);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB002);    NT35510_Write_Data(0x000D);

  //Set AVEE 5.2V
  NT35510_Write_Cmd(0xB100);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB101);    NT35510_Write_Data(0x000D);
  NT35510_Write_Cmd(0xB102);    NT35510_Write_Data(0x000D);

  //Set VCL -2.5V
  NT35510_Write_Cmd(0xB200);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xB201);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xB202);    NT35510_Write_Data(0x0000);				

  //Set AVDD Ratio
  NT35510_Write_Cmd(0xB600);    NT35510_Write_Data(0x0044);
  NT35510_Write_Cmd(0xB601);    NT35510_Write_Data(0x0044);
  NT35510_Write_Cmd(0xB602);    NT35510_Write_Data(0x0044);

  //Set AVEE Ratio
  NT35510_Write_Cmd(0xB700);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB701);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB702);    NT35510_Write_Data(0x0034);

  //Set VCL -2.5V
  NT35510_Write_Cmd(0xB800);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB801);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB802);    NT35510_Write_Data(0x0034);
        
  //Control VGH booster voltage rang
  NT35510_Write_Cmd(0xBF00);    NT35510_Write_Data(0x0001); //VGH:7~18V	

  //VGH=15V(1V/step)	Free pump
  NT35510_Write_Cmd(0xB300);    NT35510_Write_Data(0x000f);		//08
  NT35510_Write_Cmd(0xB301);    NT35510_Write_Data(0x000f);		//08
  NT35510_Write_Cmd(0xB302);    NT35510_Write_Data(0x000f);		//08

  //VGH Ratio
  NT35510_Write_Cmd(0xB900);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB901);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xB902);    NT35510_Write_Data(0x0034);

  //VGL_REG=-10(1V/step)
  NT35510_Write_Cmd(0xB500);    NT35510_Write_Data(0x0008);
  NT35510_Write_Cmd(0xB501);    NT35510_Write_Data(0x0008);
  NT35510_Write_Cmd(0xB502);    NT35510_Write_Data(0x0008);

  NT35510_Write_Cmd(0xC200);    NT35510_Write_Data(0x0003);

  //VGLX Ratio
  NT35510_Write_Cmd(0xBA00);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xBA01);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xBA02);    NT35510_Write_Data(0x0034);

    //VGMP/VGSP=4.5V/0V
  NT35510_Write_Cmd(0xBC00);    NT35510_Write_Data(0x0000);		//00
  NT35510_Write_Cmd(0xBC01);    NT35510_Write_Data(0x0078);		//C8 =5.5V/90=4.8V
  NT35510_Write_Cmd(0xBC02);    NT35510_Write_Data(0x0000);		//01

  //VGMN/VGSN=-4.5V/0V
  NT35510_Write_Cmd(0xBD00);    NT35510_Write_Data(0x0000); //00
  NT35510_Write_Cmd(0xBD01);    NT35510_Write_Data(0x0078); //90
  NT35510_Write_Cmd(0xBD02);    NT35510_Write_Data(0x0000);

  //Vcom=-1.4V(12.5mV/step)
  NT35510_Write_Cmd(0xBE00);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xBE01);    NT35510_Write_Data(0x0064); //HSD:64;Novatek:50=-1.0V, 80  5f

  //Gamma (R+)
  NT35510_Write_Cmd(0xD100);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD101);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD102);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD103);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD104);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD105);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD106);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD107);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD108);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD109);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD10A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD10B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD10C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD10D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD10E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD10F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD110);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD111);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD112);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD113);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD114);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD115);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD116);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD117);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD118);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD119);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD11A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD11B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD11C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD11D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD11E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD11F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD120);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD121);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD122);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD123);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD124);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD125);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD126);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD127);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD128);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD129);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD12A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD12B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD12C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD12D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD12E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD12F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD130);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD131);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD132);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD133);    NT35510_Write_Data(0x006D);

  //Gamma (G+)
  NT35510_Write_Cmd(0xD200);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD201);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD202);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD203);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD204);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD205);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD206);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD207);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD208);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD209);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD20A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD20B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD20C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD20D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD20E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD20F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD210);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD211);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD212);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD213);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD214);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD215);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD216);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD217);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD218);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD219);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD21A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD21B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD21C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD21D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD21E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD21F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD220);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD221);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD222);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD223);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD224);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD225);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD226);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD227);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD228);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD229);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD22A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD22B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD22C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD22D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD22E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD22F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD230);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD231);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD232);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD233);    NT35510_Write_Data(0x006D);

  //Gamma (B+)
  NT35510_Write_Cmd(0xD300);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD301);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD302);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD303);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD304);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD305);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD306);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD307);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD308);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD309);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD30A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD30B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD30C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD30D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD30E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD30F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD310);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD311);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD312);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD313);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD314);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD315);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD316);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD317);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD318);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD319);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD31A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD31B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD31C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD31D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD31E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD31F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD320);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD321);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD322);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD323);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD324);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD325);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD326);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD327);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD328);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD329);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD32A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD32B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD32C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD32D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD32E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD32F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD330);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD331);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD332);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD333);    NT35510_Write_Data(0x006D);

  //Gamma (R-)
  NT35510_Write_Cmd(0xD400);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD401);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD402);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD403);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD404);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD405);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD406);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD407);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD408);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD409);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD40A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD40B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD40C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD40D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD40E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD40F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD410);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD411);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD412);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD413);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD414);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD415);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD416);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD417);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD418);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD419);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD41A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD41B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD41C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD41D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD41E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD41F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD420);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD421);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD422);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD423);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD424);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD425);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD426);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD427);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD428);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD429);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD42A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD42B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD42C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD42D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD42E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD42F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD430);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD431);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD432);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD433);    NT35510_Write_Data(0x006D);

  //Gamma (G-)
  NT35510_Write_Cmd(0xD500);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD501);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD502);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD503);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD504);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD505);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD506);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD507);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD508);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD509);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD50A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD50B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD50C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD50D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD50E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD50F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD510);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD511);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD512);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD513);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD514);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD515);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD516);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD517);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD518);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD519);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD51A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD51B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD51C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD51D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD51E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD51F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD520);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD521);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD522);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD523);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD524);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD525);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD526);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD527);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD528);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD529);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD52A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD52B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD52C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD52D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD52E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD52F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD530);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD531);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD532);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD533);    NT35510_Write_Data(0x006D);

  //Gamma (B-)
  NT35510_Write_Cmd(0xD600);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD601);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD602);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD603);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD604);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD605);    NT35510_Write_Data(0x003A);
  NT35510_Write_Cmd(0xD606);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD607);    NT35510_Write_Data(0x004A);
  NT35510_Write_Cmd(0xD608);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD609);    NT35510_Write_Data(0x005C);
  NT35510_Write_Cmd(0xD60A);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD60B);    NT35510_Write_Data(0x0081);
  NT35510_Write_Cmd(0xD60C);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD60D);    NT35510_Write_Data(0x00A6);
  NT35510_Write_Cmd(0xD60E);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD60F);    NT35510_Write_Data(0x00E5);
  NT35510_Write_Cmd(0xD610);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD611);    NT35510_Write_Data(0x0013);
  NT35510_Write_Cmd(0xD612);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD613);    NT35510_Write_Data(0x0054);
  NT35510_Write_Cmd(0xD614);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD615);    NT35510_Write_Data(0x0082);
  NT35510_Write_Cmd(0xD616);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD617);    NT35510_Write_Data(0x00CA);
  NT35510_Write_Cmd(0xD618);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD619);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0xD61A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD61B);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xD61C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD61D);    NT35510_Write_Data(0x0034);
  NT35510_Write_Cmd(0xD61E);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD61F);    NT35510_Write_Data(0x0067);
  NT35510_Write_Cmd(0xD620);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD621);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xD622);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD623);    NT35510_Write_Data(0x00A4);
  NT35510_Write_Cmd(0xD624);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD625);    NT35510_Write_Data(0x00B7);
  NT35510_Write_Cmd(0xD626);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD627);    NT35510_Write_Data(0x00CF);
  NT35510_Write_Cmd(0xD628);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD629);    NT35510_Write_Data(0x00DE);
  NT35510_Write_Cmd(0xD62A);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD62B);    NT35510_Write_Data(0x00F2);
  NT35510_Write_Cmd(0xD62C);    NT35510_Write_Data(0x0002);
  NT35510_Write_Cmd(0xD62D);    NT35510_Write_Data(0x00FE);
  NT35510_Write_Cmd(0xD62E);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD62F);    NT35510_Write_Data(0x0010);
  NT35510_Write_Cmd(0xD630);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD631);    NT35510_Write_Data(0x0033);
  NT35510_Write_Cmd(0xD632);    NT35510_Write_Data(0x0003);
  NT35510_Write_Cmd(0xD633);    NT35510_Write_Data(0x006D);

  //PAGE0
  NT35510_Write_Cmd(0xF000);    NT35510_Write_Data(0x0055);
  NT35510_Write_Cmd(0xF001);    NT35510_Write_Data(0x00AA);
  NT35510_Write_Cmd(0xF002);    NT35510_Write_Data(0x0052);
  NT35510_Write_Cmd(0xF003);    NT35510_Write_Data(0x0008);	
  NT35510_Write_Cmd(0xF004);    NT35510_Write_Data(0x0000); 

  //480x800
  NT35510_Write_Cmd(0xB500);    NT35510_Write_Data(0x0050);

  //NT35510_Write_Cmd(0x2C00);    NT35510_Write_Data(0x0006); //8BIT 6-6-6?

  //Dispay control
  NT35510_Write_Cmd(0xB100);    NT35510_Write_Data(0x00CC);	
  NT35510_Write_Cmd(0xB101);    NT35510_Write_Data(0x0000); // S1->S1440:00;S1440->S1:02

  //Source hold time (Nova non-used)
  NT35510_Write_Cmd(0xB600);    NT35510_Write_Data(0x0005);

  //Gate EQ control	 (Nova non-used)
  NT35510_Write_Cmd(0xB700);    NT35510_Write_Data(0x0077);  //HSD:70;Nova:77	 
  NT35510_Write_Cmd(0xB701);    NT35510_Write_Data(0x0077);	//HSD:70;Nova:77

  //Source EQ control (Nova non-used)
  NT35510_Write_Cmd(0xB800);    NT35510_Write_Data(0x0001);  
  NT35510_Write_Cmd(0xB801);    NT35510_Write_Data(0x0003);	//HSD:05;Nova:07
  NT35510_Write_Cmd(0xB802);    NT35510_Write_Data(0x0003);	//HSD:05;Nova:07
  NT35510_Write_Cmd(0xB803);    NT35510_Write_Data(0x0003);	//HSD:05;Nova:07

  //Inversion mode: column
  NT35510_Write_Cmd(0xBC00);    NT35510_Write_Data(0x0002);	//00: column
  NT35510_Write_Cmd(0xBC01);    NT35510_Write_Data(0x0000);	//01:1dot
  NT35510_Write_Cmd(0xBC02);    NT35510_Write_Data(0x0000); 

  //Frame rate	(Nova non-used)
  NT35510_Write_Cmd(0xBD00);    NT35510_Write_Data(0x0001);
  NT35510_Write_Cmd(0xBD01);    NT35510_Write_Data(0x0084);
  NT35510_Write_Cmd(0xBD02);    NT35510_Write_Data(0x001c); //HSD:06;Nova:1C
  NT35510_Write_Cmd(0xBD03);    NT35510_Write_Data(0x001c); //HSD:04;Nova:1C
  NT35510_Write_Cmd(0xBD04);    NT35510_Write_Data(0x0000);

  //LGD timing control(4H/4-delay_ms)
  NT35510_Write_Cmd(0xC900);    NT35510_Write_Data(0x00D0);	//3H:0x50;4H:0xD0	 //D
  NT35510_Write_Cmd(0xC901);    NT35510_Write_Data(0x0002);  //HSD:05;Nova:02
  NT35510_Write_Cmd(0xC902);    NT35510_Write_Data(0x0050);	//HSD:05;Nova:50
  NT35510_Write_Cmd(0xC903);    NT35510_Write_Data(0x0050);	//HSD:05;Nova:50	;STV delay_ms time
  NT35510_Write_Cmd(0xC904);    NT35510_Write_Data(0x0050);	//HSD:05;Nova:50	;CLK delay_ms time

  NT35510_Write_Cmd(0x3600);    NT35510_Write_Data(0x0000);
  NT35510_Write_Cmd(0x3500);    NT35510_Write_Data(0x0000);

  NT35510_Write_Cmd(0xFF00);    NT35510_Write_Data(0x00AA);
  NT35510_Write_Cmd(0xFF01);    NT35510_Write_Data(0x0055);
  NT35510_Write_Cmd(0xFF02);    NT35510_Write_Data(0x0025);
  NT35510_Write_Cmd(0xFF03);    NT35510_Write_Data(0x0001);

  NT35510_Write_Cmd(0xFC00);    NT35510_Write_Data(0x0016);
  NT35510_Write_Cmd(0xFC01);    NT35510_Write_Data(0x00A2);
  NT35510_Write_Cmd(0xFC02);    NT35510_Write_Data(0x0026);
  NT35510_Write_Cmd(0x3A00);    NT35510_Write_Data(0x0006);

  NT35510_Write_Cmd(0x3A00);    NT35510_Write_Data(0x0055);
  //Sleep out
  NT35510_Write_Cmd(0x1100);	   //?
  Delay(0xFFFFFF);

  //Display on
  NT35510_Write_Cmd(0x2900);
}


/*=========================================================================================*/

#if 1

#define		EXT_LCD_ARCH
#define		EXT_LCD_DMA

#include 	"EXT_LCD_Arch.c"

#endif


/*=========================================================================================*/


/**
 * @brief  NT35510初始化函数，如果要用到lcd，一定要调用这个函数
 * @param  无
 * @retval 无
 */
void LCD_Init ( void )
{
  static uint8_t i = 3;
	NT35510_GPIO_Config ();
	NT35510_FSMC_Config ();

	NT35510_Rst ();
	NT35510_REG_Config ();
  LCD_SetRotate(ROTATE_90);

	NT35510_Clear(0, 0, 800, 480);	/* 清屏，显示全黑 */
	LCD_BackLed_Control ( ENABLE );      //点亮LCD背光灯
  
#ifdef	EXT_LCD_DMA
	LCD_DMA_Init(LCD_DAT_ADDR);
#endif

}



/**
 * @brief  NT35510背光LED控制
 * @param  enumState ：决定是否使能背光LED
  *   该参数为以下值之一：
  *     @arg ENABLE :使能背光LED
  *     @arg DISABLE :禁用背光LED
 * @retval 无
 */
void LCD_BackLed_Control ( int  on )
{
	if ( on )
		 GPIO_SetBits( NT35510_BK_PORT, NT35510_BK_PIN );	
	else
		 GPIO_ResetBits( NT35510_BK_PORT, NT35510_BK_PIN );
		
}

/**
 * @brief  NT35510 软件复位
 * @param  无
 * @retval 无
 */
void NT35510_Rst ( void )
{			
	GPIO_ResetBits ( NT35510_RST_PORT, NT35510_RST_PIN );	 //低电平复位

	NT35510_Delay ( 0xAFF ); 					   

	GPIO_SetBits ( NT35510_RST_PORT, NT35510_RST_PIN );		 	 

	NT35510_Delay ( 0xAFF ); 	
	
}

/**
 * @brief  在NT35510显示器上开辟一个窗口
 * @param  usX ：在特定扫描方向下窗口的起点X坐标
 * @param  usY ：在特定扫描方向下窗口的起点Y坐标
 * @param  usWidth ：窗口的宽度
 * @param  usHeight ：窗口的高度
 * @retval 无
 */
void NT35510_OpenWindow ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight )
{	
	NT35510_Write_Cmd ( CMD_SetCoordinateX ); 				 /* 设置X坐标 */
	NT35510_Write_Data ( usX >> 8  );	 /* 先高8位，然后低8位 */
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 1 ); 
	NT35510_Write_Data ( usX & 0xff  );	 /* 设置起始点和结束点*/
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 2 );
	NT35510_Write_Data ( ( usX + usWidth - 1 ) >> 8  );
  NT35510_Write_Cmd ( CMD_SetCoordinateX + 3 );
	NT35510_Write_Data ( ( usX + usWidth - 1 ) & 0xff  );

	NT35510_Write_Cmd ( CMD_SetCoordinateY ); 			     /* 设置Y坐标*/
	NT35510_Write_Data ( usY >> 8  );
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 1);
	NT35510_Write_Data ( usY & 0xff  );
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 2);
	NT35510_Write_Data ( ( usY + usHeight - 1 ) >> 8 );
  NT35510_Write_Cmd ( CMD_SetCoordinateY + 3);
	NT35510_Write_Data ( ( usY + usHeight - 1) & 0xff );
	
}

/**
 * @brief  在NT35510显示器上以某一颜色填充像素点
 * @param  ulAmout_Point ：要填充颜色的像素点的总数目
 * @param  usColor ：颜色
 * @retval 无
 */
static __inline void NT35510_FillColor ( uint32_t ulAmout_Point, uint16_t usColor )
{
	uint32_t i = 0;

	/* memory write */
	NT35510_Write_Cmd ( CMD_SetPixel );	
		
	for ( i = 0; i < ulAmout_Point; i ++ )
		NT35510_Write_Data ( usColor );
}


/**
 * @brief  对NT35510显示器的某一窗口以某种颜色进行清屏
 * @param  usX ：在特定扫描方向下窗口的起点X坐标
 * @param  usY ：在特定扫描方向下窗口的起点Y坐标
 * @param  usWidth ：窗口的宽度
 * @param  usHeight ：窗口的高度
 * @note 可使用LCD_SetBackColor、LCD_SetTextColor、LCD_SetColors函数设置颜色
 * @retval 无
 */
void NT35510_Clear ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight )
{
	NT35510_OpenWindow ( usX, usY, usWidth, usHeight );

	NT35510_FillColor ( usWidth * usHeight, GREEN );		
}

/*********************end of file*************************/



