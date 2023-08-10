/*********************************************************************************************************
* 模块名称：LED.c
* 摘    要：LED模块
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

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "LED.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static  void  ConfigLEDGPIO(void);  //配置LED的GPIO

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigLEDGPIO
* 函数功能：配置LED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static  void  ConfigLEDGPIO(void)
{
  //使能RCU相关时钟
  rcu_periph_clock_enable(RCU_GPIOA);                                 //使能GPIOA的时钟
  rcu_periph_clock_enable(RCU_GPIOE);                                 //使能GPIOE的时钟
  
  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);  //设置GPIO输出模式及速度
  gpio_bit_set(GPIOA, GPIO_PIN_8);                                    //将LED1默认状态设置为点亮
  
  gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);  //设置GPIO输出模式及速度
  gpio_bit_reset(GPIOE, GPIO_PIN_6);                                  //将LED2默认状态设置为熄灭
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitLED
* 函数功能：初始化LED模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void InitLED(void)
{
  ConfigLEDGPIO();  //配置LED的GPIO
}

/*********************************************************************************************************
* 函数名称：LEDFlicker
* 函数功能：LED闪烁函数
* 输入参数：cnt
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：LEDFlicker在Proc2msTask中调用，cnt为250时表示每500ms更改一次LED状态
*********************************************************************************************************/
void LEDFlicker(unsigned short cnt)
{
  static unsigned short s_iCnt;  //定义静态变量s_iCnt作为计数器
  
  s_iCnt++;                      //计数器的计数值加1
  
  if(s_iCnt >= cnt)              //计数器的计数值大于cnt
  { 
    s_iCnt = 0;                  //重置计数器的计数值为0

    //LED1状态取反，实现LED1闪烁
    gpio_bit_write(GPIOA, GPIO_PIN_8, (FlagStatus)(1 - gpio_output_bit_get(GPIOA, GPIO_PIN_8)));

    //LED2状态取反，实现LED2闪烁
    gpio_bit_write(GPIOE, GPIO_PIN_6, (FlagStatus)(1 - gpio_output_bit_get(GPIOE, GPIO_PIN_6)));
  }
}
