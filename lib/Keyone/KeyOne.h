/*********************************************************************************************************
* 模块名称：KeyOne.h
* 摘    要：KeyOne模块，进行独立按键初始化，以及按键扫描函数实现
* 当前版本：1.0.0
* 作    者：Leyutek(COPYRIGHT 2018 - 2021 Leyutek. All rights reserved.)
* 完成日期：2021年07月01日 
* 内    容：
* 注    意：                                                                  
**********************************************************************************************************
* 取代版本： 
* 作    者：
* 完成日期： 
* 修改内容：
* 修改文件： 
*********************************************************************************************************/
#ifndef _KEY_ONE_H_
#define _KEY_ONE_H_

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "DataType.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
//各个按键按下的电平
#define  KEY_DOWN_LEVEL_KEY1    0xFF     //0xFF表示按下为高电平
#define  KEY_DOWN_LEVEL_KEY2    0x00     //0x00表示按下为低电平
#define  KEY_DOWN_LEVEL_KEY3    0x00     //0x00表示按下为低电平

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/
typedef enum
{
  KEY_NAME_KEY1 = 0,  //KEY1
  KEY_NAME_KEY2,      //KEY2
  KEY_NAME_KEY3,      //KEY3
  KEY_NAME_MAX
}EnumKeyOneName;

/*********************************************************************************************************
*                                              API函数声明
*********************************************************************************************************/
void  InitKeyOne(void);                                                                     //初始化KeyOne模块
void  ScanKeyOne(unsigned char keyName, void(*OnKeyOneUp)(void), void(*OnKeyOneDown)(void));//每10ms调用一次

#endif
