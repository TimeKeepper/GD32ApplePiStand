/*********************************************************************************************************
* 模块名称：PWM.c
* 摘    要：PWM模块
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
#include "PWM.h"
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
static  signed short s_iDutyCycle = 0;  //用于存放占空比

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void ConfigTIMER1ForPWMPB10(unsigned short arr, unsigned short psc);  //配置PWM

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigTIMER1ForPWMPB10
* 函数功能：配置TIMER1
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static void ConfigTIMER1ForPWMPB10(unsigned short arr, unsigned short psc)
{
  //定义初始化结构体变量
  timer_oc_parameter_struct timer_ocinitpara;
  timer_parameter_struct timer_initpara;
  
  rcu_periph_clock_enable(RCU_GPIOB);    //使能GPIOB时钟
  rcu_periph_clock_enable(RCU_TIMER1);   //使能TIMER1时钟
  rcu_periph_clock_enable(RCU_AF);       //使能TIMER1时钟
  
  gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP1, ENABLE);  //TIMER1部分重映射TIMER1_CH2->PB10
  gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);  //设置GPIO输出模式及速度

  timer_deinit(TIMER1);                                  //将TIMER1配置为默认值
  timer_struct_para_init(&timer_initpara);               //timer_initpara配置为默认值
    
  timer_initpara.prescaler         = psc;                //设置预分频值
  timer_initpara.alignedmode       = TIMER_COUNTER_EDGE; //设置对齐模式
  timer_initpara.counterdirection  = TIMER_COUNTER_UP;   //设置向上计数
  timer_initpara.period            = arr;                //设置重装载值
  timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;   //设置时钟分频因子
  timer_initpara.repetitioncounter = 0;                  //设置重复计数值
  timer_init(TIMER1, &timer_initpara);                   //初始化定时器
    
  //将结构体参数初始化为默认值
  timer_channel_output_struct_para_init(&timer_ocinitpara);  
    
  timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;                    //设置通道输出状态
  timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;                  //设置互补通道输出状态
  timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;              //设置通道输出极性
  timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;             //设置互补通道输出极性
  timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;             //设置空闲状态下通道输出极性
  timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;            //设置空闲状态下互补通道输出极性
  timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocinitpara);  //初始化结构体
     
  timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 0);                   //设置占空比
  timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);         //设置通道比较模式
  timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);  //失能比较影子寄存器
  timer_auto_reload_shadow_enable(TIMER1);                                          //自动重载影子使能 

  timer_enable(TIMER1);  //使能定时器
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitPWM
* 函数功能：初始化PWM模块
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void  InitPWM(void)
{
  ConfigTIMER1ForPWMPB10(599, 999);  //配置TIMER1，120000000/(999+1)/(599+1)=200Hz
}

/*********************************************************************************************************
* 函数名称：SetPWM
* 函数功能：设置占空比
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void SetPWM(signed short val)
{
  s_iDutyCycle = val;                   //获取占空比的值
  
  timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, s_iDutyCycle);  //设置占空比
}

/*********************************************************************************************************
* 函数名称：IncPWMDutyCycle
* 函数功能：递增占空比，每次递增方波周期的1/12，直至高电平输出 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void IncPWMDutyCycle(void)
{
  if(s_iDutyCycle >= 600)               //如果占空比不小于600
  {                                     
    s_iDutyCycle = 600;                 //保持占空比值为600
  }                                     
  else                                  
  {                                     
    s_iDutyCycle += 50;                 //占空比递增方波周期的1/12
  }
  
  timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, s_iDutyCycle);  //设置占空比
}

/*********************************************************************************************************
* 函数名称：DecPWMDutyCycle
* 函数功能：递减占空比，每次递减方波周期的1/12，直至低电平输出 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void DecPWMDutyCycle(void)
{
  if(s_iDutyCycle <= 0)               //如果占空比不大于0
  {                                   
    s_iDutyCycle = 0;                 //保持占空比值为0
  }                                   
  else                                
  {                                   
    s_iDutyCycle -= 50;               //占空比递减方波周期的1/12
  }
  
  timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, s_iDutyCycle);//设置占空比
}
