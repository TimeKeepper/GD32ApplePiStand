/*********************************************************************************************************
* 模块名称：Capture.c
* 摘    要：Capture模块
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
#include "Capture.h"
#include "gd32f30x_conf.h"
#include <stdio.h>

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量定义
*********************************************************************************************************/
//s_iCaptureSts中的bit7为捕获完成的标志，bit6为捕获到上升沿标志，bit5-bit0为捕获到上升沿后定时器溢出的次数
static  unsigned char  s_iCaptureSts = 0;     //捕获状态 
static  unsigned short s_iCaptureVal;         //捕获值
 
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static  void ConfigTIMER1ForCapture(unsigned short arr, unsigned short psc);    //配置TIMER1

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigTIMER1ForCapture
* 函数功能：配置TIMER1
* 输入参数：arr-自动重装值，psc-预分频器值
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日 
* 注    意：此处暂定使用定时器TIMER1的CH0（PA0）来做输入捕获，捕获PA0上高电平的脉宽
*********************************************************************************************************/
static  void ConfigTIMER1ForCapture(unsigned short arr, unsigned short psc)
{ 
  //定义TIMER初始化结构体变量
  timer_ic_parameter_struct timer_icinitpara;  
  timer_parameter_struct timer_initpara;
  
  rcu_periph_clock_enable(RCU_GPIOA);   //使能GPIOA时钟
  rcu_periph_clock_enable(RCU_TIMER1);  //使能TIMER1时钟

  gpio_init(GPIOA, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_0);  //设置PA0输入模式及速度模式
  
  timer_deinit(TIMER1);                                     //TIMER1设置为默认值
  timer_struct_para_init(&timer_initpara);                  //TIMER1结构体设置为默认值
  timer_channel_input_struct_para_init(&timer_icinitpara);  //将输入结构体中的参数初始化为默认值
  
  //TIMER1配置
  timer_initpara.prescaler         = psc;                   //设置预分频器值
  timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;    //设置对齐模式
  timer_initpara.counterdirection  = TIMER_COUNTER_UP;      //设置向上计数模式
  timer_initpara.period            = arr;                   //设置自动重装载值
  timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;      //设置时钟分割
  timer_init(TIMER1, &timer_initpara);                      //初始化时钟

  timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;          //设置输入极性
  timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;       //设置通道输入模式
  timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;                 //设置预分频器值
  timer_icinitpara.icfilter    = 0x0;                               //设置输入捕获滤波
  timer_input_capture_config(TIMER1, TIMER_CH_0, &timer_icinitpara);//初始化通道

  timer_auto_reload_shadow_enable(TIMER1);                //使能自动重载影子寄存器

  timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH0); //清除CH0中断标志位
  timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);  //清除更新中断标志位
  
  timer_interrupt_enable(TIMER1, TIMER_INT_UP);           //使能定时器的更新中断
  timer_interrupt_enable(TIMER1, TIMER_INT_CH0);          //使能定时器的CH0输入通道中断
  nvic_irq_enable(TIMER1_IRQn, 2, 0);                     //使能TIMER1中断，并设置优先级

  timer_enable(TIMER1);  //使能TIMER1
}

/*********************************************************************************************************
* 函数名称：TIMER1_IRQHandler
* 函数功能：TIMER1中断服务函数
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日 
* 注    意：
*********************************************************************************************************/
void TIMER1_IRQHandler(void)
{ 
  if((s_iCaptureSts & 0x80) == 0) //最高位为0，表示捕获还未完成
  {  
    //高电平，定时器TIMER1发生了溢出事件
    if(timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) != RESET)        
    {    
      if(s_iCaptureSts & 0x40)    //发生溢出，并且前一次已经捕获到高电平
      {
        //TIMER_CAR 16位预装载值，即CNT > 65536-1（2^16 - 1）时溢出。
        //若不处理，(s_iCaptureSts & 0x3F)++等于0x40 ，溢出数等于清0
        if((s_iCaptureSts & 0x3F) == 0x3F)  //达到多次溢出，高电平太长
        {
          s_iCaptureSts |= 0x80;  //强制标记成功捕获了一次
          s_iCaptureVal = 0xFFFF; //捕获值为0xFFFF
        } 
        else
        {
          s_iCaptureSts++;        //标记计数器溢出一次
        }
      }
    }
    
    if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_CH0) != RESET) //发生捕获事件
    { 
      if(s_iCaptureSts & 0x40)  //bit6为1，即上次捕获到上升沿，那么这次捕获到下降沿
      {
        s_iCaptureSts |= 0x80;  //完成捕获，标记成功捕获到一次下降沿
        s_iCaptureVal = timer_channel_capture_value_register_read(TIMER1,TIMER_CH_0);  //s_iCaptureVa记录捕获比较寄存器的值
        //设置为上升沿捕获，为下次捕获做准备
        timer_channel_output_polarity_config(TIMER1, TIMER_CH_0, TIMER_IC_POLARITY_RISING);
      }
      else  //bit6为0，表示上次没捕获到上升沿，这是第一次捕获上升沿
      {
        s_iCaptureSts = 0;  //清空溢出次数
        s_iCaptureVal = 0;  //捕获值为0
                                                               
        timer_counter_value_config(TIMER1, 0);  //设置寄存器的值为0
                                                             
        s_iCaptureSts |= 0x40;    //bit6置为1，标记捕获到了上升沿
        
        //设置为下降沿捕获
        timer_channel_output_polarity_config(TIMER1, TIMER_CH_0, TIMER_IC_POLARITY_FALLING);
      }    
    } 
  }
 
  timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH0);  //清除更新CH0捕获中断标志位
  timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);   //清除更新中断标志位
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitCapture
* 函数功能：初始化Capture模块 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日 
* 注    意：
*********************************************************************************************************/
void  InitCapture(void)
{
  //计数器达到最大装载值0xFFFF，会产生溢出；以120MHz/（120-1+1）=1MHz的频率计数
  ConfigTIMER1ForCapture(0xFFFF, 120 - 1);  
}

/*********************************************************************************************************
* 函数名称：GetCaptureVal
* 函数功能：获取捕获时间，返回值为1表示捕获成功，此时*pCapVal才有意义 
* 输入参数：void
* 输出参数：pCalVal，捕获到的值的地址
* 返 回 值：ok：1-获取成功
* 创建日期：2021年07月01日 
* 注    意：
*********************************************************************************************************/
unsigned char  GetCaptureVal(signed int* pCapVal)
{
  unsigned char ok = 0;
  
  if(s_iCaptureSts & 0x80)              //最高位为1，表示成功捕获到了下降沿（获取到按键弹起标志）
  {
    ok = 1;                             //捕获成功
    (*pCapVal)  = s_iCaptureSts & 0x3F; //取出低6位计数器的值赋给(*pCapVal)，得到溢出次数
    //printf("溢出次数:%d\r\n",*pCapVal);
    (*pCapVal) *= 65536;                //计数器计数次数为2^16=65536，乘以溢出次数，得到溢出时间总和（以1/1MHz=1us为单位）
    (*pCapVal) += s_iCaptureVal;        //加上最后一次比较捕获寄存器的值，得到总的低电平时间

    s_iCaptureSts = 0;                  //设置为0，开启下一次捕获
  }

  return(ok);                           //返回是否捕获成功的标志
}
