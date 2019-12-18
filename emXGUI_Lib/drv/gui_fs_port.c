/**
  *********************************************************************
  * @file    gui_fs_port.c
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   文件系统接口，需要实现FileSystem_Init函数
  *********************************************************************
  * @attention
  * 官网    :www.emXGUI.com
  *
  **********************************************************************
  */ 
#include "gui_fs_port.h"
#include "ff.h"



	/* FatFs文件系统对象 */
static FATFS fs ;										

void FileSystem_Test(void);



/**
  * @brief  文件系统初始化
  * @param  无
  * @retval TRUE  FALSE
  */
BOOL FileSystem_Init(void)
{ 

#if defined(STM32F429_439xx) || defined(STM32H743xx) || defined(STM32F767xx) || defined(STM32F10X_HD) || defined(STM32F40_41xxx)
	//在外部SPI Flash挂载文件系统，文件系统挂载时会对SPI设备初始化
  FRESULT res_sd; 
#if defined(STM32F767xx) || defined(STM32H750xx)
  FATFS_LinkDriver(&SD_Driver, SDPath);
#endif
	res_sd = f_mount(&fs,"0:",1);
	
	if(res_sd == FR_NO_FILESYSTEM)
	{
		printf("》SD卡还没有文件系统...\r\n");
    /* 格式化 */
		res_sd=f_mkfs("0:",0,0);							
		
		if(res_sd == FR_OK)
		{
			printf("》SD卡已成功格式化文件系统。\r\n");
      /* 格式化后，先取消挂载 */
			res_sd = f_mount(NULL,"0:",1);			
      /* 重新挂载	*/			
			res_sd = f_mount(&fs,"0:",1);
		}
		else
		{
//			LED_RED;
			printf("《《格式化失败。》》\r\n");
//			while(1);
		}
    return FALSE;
	}
  else if(res_sd!=FR_OK)
  {
    printf("！！SD卡挂载文件系统失败。(%d)\r\n",res_sd);
    printf("！！可能原因：没有接入SD卡。\r\n");
    return FALSE;

  }
  else
  {
    printf("》文件系统挂载成功\r\n");
    
#if 0
    /* 文件系统测试 */
    FileSystem_Test();
#endif 
    
    /* 尝试进行unicode编码转换，
      当使用extern_cc936时，可测试是否存在cc936资源*/
    ff_convert('a',1);
    
    
  }
#elif defined(CPU_MIMXRT1052DVL6B)
  f_mount_test(&fs);
#endif
  
  return TRUE;
}

/********************************END OF FILE****************************/

