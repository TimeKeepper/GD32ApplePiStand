/*********************************************************************************************************
* 模块名称：RunClock.h
* 摘    要：RunClock模块
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
#ifndef _RUN_CLOCK_H_
#define _RUN_CLOCK_H_

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/
//定义枚举
typedef enum  
{
  TIME_VAL_HOUR = 0,
  TIME_VAL_MIN,
  TIME_VAL_SEC,
  TIME_VAL_MAX
}EnumTimeVal;

/*********************************************************************************************************
*                                              API函数声明
*********************************************************************************************************/
void  InitRunClock(void);                                                     //初始化RunClock模块

void  RunClockPer2ms(void);                                                   //每2ms调用一次
void  PauseClock(unsigned char flag);                                         //flag，TRUE-暂停，FALSE-正常运行

signed short   GetTimeVal(unsigned char type);                                //获取时间值
void  SetTimeVal(unsigned char type, signed short timeVal);                   //设置时间值

void  DispTime(signed short hour, signed short min, signed short sec);        //显示当前的时间

#endif


