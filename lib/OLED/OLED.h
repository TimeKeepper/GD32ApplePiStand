/*********************************************************************************************************
* 模块名称：OLED.h
* 摘    要：OLED显示屏模块，4线串行接口，CS、DC、SCK、DIN、RES
* 当前版本：1.0.0
* 作    者：Leyutek(COPYRIGHT 2018 - 2021 Leyutek. All rights reserved.)
* 完成日期：2021年07月01日  
* 内    容：
* 注    意：OLED取模使用的是PCtoLCD2002完美版软件                                                                  
**********************************************************************************************************
* 取代版本：
* 作    者：
* 完成日期：
* 修改内容：
* 修改文件：
*********************************************************************************************************/
#ifndef _OLED_H_
#define _OLED_H_
 
/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include  "DataType.h" 
   
/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/

/*********************************************************************************************************
*                                              API函数声明
*********************************************************************************************************/	    
void  InitOLED(void);        //初始化OLED模块
void  OLEDDisplayOn(void);   //开启OLED显示
void  OLEDDisplayOff(void);  //关闭OLED显示
void  OLEDRefreshGRAM(void); //将微控制器的GRAM写入到SSD1306的GRAM

void  OLEDClear(void);  //清屏函数，清除屏幕上显示的所有内容
void  OLEDShowChar(unsigned char x, unsigned char y, unsigned char chr, unsigned char size, unsigned char mode); //在指定位置显示一个字符
void  OLEDShowNum(unsigned char x, unsigned char y, unsigned int num, unsigned char len, unsigned char size);  //在指定位置显示数字
void  OLEDShowString(unsigned char x, unsigned char y, const unsigned char* p);            //在指定位置显示字符串

#endif  
 



