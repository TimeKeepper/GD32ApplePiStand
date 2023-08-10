/*********************************************************************************************************
* 模块名称：OLED.c   
* 摘    要：OLED显示屏模块，4线串行接口，CS、DC、SCK、DIN、RES
* 当前版本：1.0.0
* 作    者：Leyutek(COPYRIGHT 2018 - 2021 Leyutek. All rights reserved.)
* 完成日期：2021年07月01日
* 内    容：
* 注    意：OLED的显存
            存放格式如下.
            [0]0 1 2 3 ... 127
            [1]0 1 2 3 ... 127
            [2]0 1 2 3 ... 127
            [3]0 1 2 3 ... 127
            [4]0 1 2 3 ... 127
            [5]0 1 2 3 ... 127
            [6]0 1 2 3 ... 127
            [7]0 1 2 3 ... 127                                                                   
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
#include "OLED.h"
#include "gd32f30x_conf.h"
#include "OLEDFont.h"
#include "SysTick.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/         
#define OLED_CMD    0 //命令
#define OLED_DATA   1 //数据

//OLED端口定义  
#define CLR_OLED_RES()  gpio_bit_reset(GPIOC, GPIO_PIN_10)   //RES，复位
#define SET_OLED_RES()  gpio_bit_set(GPIOC, GPIO_PIN_10)

#define CLR_OLED_CS()   gpio_bit_reset(GPIOC, GPIO_PIN_12)   //CS，片选
#define SET_OLED_CS()   gpio_bit_set(GPIOC, GPIO_PIN_12)

#define CLR_OLED_SCK()  gpio_bit_reset(GPIOB, GPIO_PIN_3)    //SCK，时钟
#define SET_OLED_SCK()  gpio_bit_set(GPIOB, GPIO_PIN_3)

#define CLR_OLED_DC()   gpio_bit_reset(GPIOB, GPIO_PIN_4)    //DC，命令数据标志（0-命令/1-数据）
#define SET_OLED_DC()   gpio_bit_set(GPIOB, GPIO_PIN_4)

#define CLR_OLED_DIN()  gpio_bit_reset(GPIOB, GPIO_PIN_5)    //DIN，数据
#define SET_OLED_DIN()  gpio_bit_set(GPIOB, GPIO_PIN_5)

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/   

/*********************************************************************************************************
*                                              内部变量定义
*********************************************************************************************************/
static  unsigned char  s_arrOLEDGRAM[128][8];    //OLED显存缓冲区
 
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/ 
static  void  ConfigOLEDGPIO(void);             //配置OLED的GPIO
static  void  ConfigOLEDReg(void);              //配置OLED的SSD1306寄存器
                                                
static  void  OLEDWriteByte(unsigned char dat, unsigned char cmd);    //向SSD1306写入一字节命令或数据
static  void  OLEDDrawPoint(unsigned char x, unsigned char y, unsigned char t);  //在OLED屏指定位置画点
                                                
static  unsigned int   CalcPow(unsigned char m, unsigned char n);              //计算m的n次方

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigOLEDGPIO 
* 函数功能：配置OLED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void 
* 创建日期：2021年07月01日
* 注    意：PC10（OLED_RES）、PC12（OLED_CS）、PB3（OLED_SCK）、PB4（OLED_DC）、PB5（OLED_DIN）
*********************************************************************************************************/
static  void  ConfigOLEDGPIO(void)
{
  rcu_periph_clock_enable(RCU_AF);
  gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);		//EMC用到了PB3和PB4,需要关闭JTAG；
  
  //使能RCU相关时钟
  rcu_periph_clock_enable(RCU_GPIOB);  //使能GPIOB的时钟
  rcu_periph_clock_enable(RCU_GPIOC);  //使能GPIOC的时钟
  
  //配置RES
  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);             //设置GPIO输出模式及速度
  gpio_bit_set(GPIOC, GPIO_PIN_10);                                               //设置初始状态为高电平
  
  //配置CS
  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);             //设置GPIO输出模式及速度
  gpio_bit_set(GPIOC, GPIO_PIN_12);                                               //设置初始状态为高电平
  
  //配置SCK
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);              //设置GPIO输出模式及速度
  gpio_bit_set(GPIOB, GPIO_PIN_3);                                                //设置初始状态为高电平
  
  //配置DC
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);              //设置GPIO输出模式及速度
  gpio_bit_set(GPIOB, GPIO_PIN_4);                                                //设置初始状态为高电平
  
  //配置DIN
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);              //设置GPIO输出模式及速度
  gpio_bit_set(GPIOB, GPIO_PIN_5);                                                //设置初始状态为高电平
}

/*********************************************************************************************************
* 函数名称：ConfigOLEDReg 
* 函数功能：配置OLED的寄存器 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void 
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static  void  ConfigOLEDReg( void )
{
  OLEDWriteByte(0xAE, OLED_CMD); //关闭显示
  
  OLEDWriteByte(0xD5, OLED_CMD); //设置时钟分频因子，振荡频率
  OLEDWriteByte(0x50, OLED_CMD); //[3:0]为分频因子，[7:4]为振荡频率
  
  OLEDWriteByte(0xA8, OLED_CMD); //设置驱动路数
  OLEDWriteByte(0x3F, OLED_CMD); //默认0x3F（1/64） 
  
  OLEDWriteByte(0xD3, OLED_CMD); //设置显示偏移
  OLEDWriteByte(0x00, OLED_CMD); //默认为0

  OLEDWriteByte(0x40, OLED_CMD); //设置显示开始行，[5:0]为行数
  
  OLEDWriteByte(0x8D, OLED_CMD); //设置电荷泵
  OLEDWriteByte(0x14, OLED_CMD); //bit2用于设置开启（1）/关闭（0）
  
  OLEDWriteByte(0x20, OLED_CMD); //设置内存地址模式
  OLEDWriteByte(0x02, OLED_CMD); //[1:0]，00-列地址模式，01-行地址模式，10-页地址模式（默认值）
  
  OLEDWriteByte(0xA1, OLED_CMD); //设置段重定义，bit0为0，列地址0->SEG0，bit0为1，列地址0->SEG127
  
  OLEDWriteByte(0xC0, OLED_CMD); //设置COM扫描方向，bit3为0，普通模式，bit3为1，重定义模式
  
  OLEDWriteByte(0xDA, OLED_CMD); //设置COM硬件引脚配置
  OLEDWriteByte(0x12, OLED_CMD); //[5:4]为硬件引脚配置信息
  
  OLEDWriteByte(0x81, OLED_CMD); //设置对比度
  OLEDWriteByte(0xEF, OLED_CMD); //1～255，默认为0x7F（亮度设置，越大越亮）
  
  OLEDWriteByte(0xD9, OLED_CMD); //设置预充电周期
  OLEDWriteByte(0xf1, OLED_CMD); //[3:0]为PHASE1，[7:4]为PHASE2
  
  OLEDWriteByte(0xDB, OLED_CMD); //设置VCOMH电压倍率
  OLEDWriteByte(0x30, OLED_CMD); //[6:4]，000-0.65*vcc，001-0.77*vcc，011-0.83*vcc

  OLEDWriteByte(0xA4, OLED_CMD); //全局显示开启，bit0为1，开启，bit0为0，关闭
  
  OLEDWriteByte(0xA6, OLED_CMD); //设置显示方式，bit0为1，反相显示，bit0为0，正常显示 
  
  OLEDWriteByte(0xAF, OLED_CMD); //开启显示
}

/*********************************************************************************************************
* 函数名称：OLEDWriteByte
* 函数功能：向SSD1306写入一字节数据
* 输入参数：dat为要写入的数据或命令，cmd为数据/命令标志，0-命令，1-数据
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static  void  OLEDWriteByte(unsigned char dat, unsigned char cmd)
{
  signed short i;
                                
  //判断要写入数据还是写入命令    
  if(OLED_CMD == cmd)         //如果标志cmd为传入命令时
  {                             
    CLR_OLED_DC();            //DC输出低电平用来读写命令  
  }                             
  else if(OLED_DATA == cmd)   //如果标志cmd为传入数据时
  {                             
    SET_OLED_DC();            //DC输出高电平用来读写数据  
  }                             
                                
  CLR_OLED_CS();              //CS输出低电平为写入数据或命令作准备
                                
  for(i = 0; i < 8; i++)      //循环8次，从高到低取出要写入的数据或命令的8个bit
  {                             
    CLR_OLED_SCK();           //SCK输出低电平为写入数据作准备
                                
    if(dat & 0x80)            //判断要写入的数据或命令的最高位是1还是0
    {                           
      SET_OLED_DIN();         //要写入的数据或命令的最高位是1，DIN输出高电平表示1
    }                           
    else                        
    {                           
      CLR_OLED_DIN();         //要写入的数据或命令的最高位是0，DIN输出低电平表示0
    }                           
    SET_OLED_SCK();           //SCK输出高电平，DIN的状态不再变化，此时写入数据线的数据
                                
    dat <<= 1;                //左移一位，次高位移到最高位
  }                             
                                
  SET_OLED_CS();              //OLED的CS输出高电平，不再写入数据或命令  
  SET_OLED_DC();              //OLED的DC输出高电平
} 

/*********************************************************************************************************
* 函数名称：OLEDDrawPoint
* 函数功能：在指定位置画点
* 输入参数：x取值范围为0～127，y取值范围为0～63，t为1表示填充，t为0表示清空
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：（1）x-y轴体系的原点坐标在屏幕左上角；
*           （2）物理显示的y与SSD1306显存的COMn的n相加为63，当然，屏幕的行号（按字节）与SSD1306的行号（按
*                字节）相加为7。
*********************************************************************************************************/         
static  void  OLEDDrawPoint(unsigned char x, unsigned char y, unsigned char t)
{
  unsigned char pos;                           //存放点所在的页数
  unsigned char bx;                            //存放点所在的屏幕的行号
  unsigned char temp = 0;                      //用来存放画点位置相对于字节的位
                                    
  if(x > 127 || y > 63)             //如果指定位置超过额定范围
  {                                 
    return;                         //返回空，函数结束
  }                                 
                                    
  pos = 7 - y / 8;                  //求指定位置所在页数
  bx = y % 8;                       //求指定位置在上面求出页数中的行号
  temp = 1 << (7 - bx);             //（7-bx）求出相应SSD1306的行号，并在字节中相应的位置为1
                                    
  if(t)                             //判断填充标志为1还是0
  {                                 
    s_arrOLEDGRAM[x][pos] |= temp;  //如果填充标志为1，指定点填充
  }                                 
  else                              
  {                                 
    s_arrOLEDGRAM[x][pos] &= ~temp; //如果填充标志为0，指定点清空  
  }
}

/*********************************************************************************************************
* 函数名称：CalcPow
* 函数功能：计算m的n次方
* 输入参数：m，n 
* 输出参数：void
* 返 回 值：m的n次方结果 
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static  unsigned int CalcPow(unsigned char m, unsigned char n)
{
  unsigned int result = 1;     //定义用来存放结果的变量      
                      
  while(n--)          //随着每次循环，n递减，直至为0
  {                   
    result *= m;      //循环n次，相当于n个m相乘
  }                   
                      
  return result;      //返回m的n次幂的值
}
 
/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/  
/*********************************************************************************************************
* 函数名称：InitOLED
* 函数功能：初始化OLED模块
* 输入参数：void 
* 输出参数：void
* 返 回 值：void 
* 创建日期：2021年07月01日
* 注    意:
*********************************************************************************************************/   
void  InitOLED(void)
{ 
  ConfigOLEDGPIO();     //配置OLED的GPIO
  
  CLR_OLED_RES();
  DelayNms(10);  
  SET_OLED_RES();       //RES引脚务必拉高
  DelayNms(10);
  
  ConfigOLEDReg();      //配置OLED的寄存器
  
  OLEDClear();          //清除OLED屏内容
}  

/*********************************************************************************************************
* 函数名称：OLEDDisplayOn
* 函数功能：开启OLED显示
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/   
void  OLEDDisplayOn( void )
{
  //打开关闭电荷泵，第一字节为命令字，0x8D，第二字节设置值，0x10-关闭电荷泵，0x14-打开电荷泵
  OLEDWriteByte(0x8D, OLED_CMD);  //第一字节0x8D为命令
  OLEDWriteByte(0x14, OLED_CMD);  //0x14-打开电荷泵

  //设置显示开关，0xAE-关闭显示，0xAF-开启显示
  OLEDWriteByte(0xAF, OLED_CMD);  //开启显示
}

/*********************************************************************************************************
* 函数名称：OLEDDisplayOff
* 函数功能：关闭OLED显示
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/ 
void  OLEDDisplayOff(void)
{
  //打开关闭电荷泵，第一字节为命令字，0x8D，第二字节设置值，0x10-关闭电荷泵，0x14-打开电荷泵
  OLEDWriteByte(0x8D, OLED_CMD);  //第一字节为命令字，0x8D
  OLEDWriteByte(0x10, OLED_CMD);  //0x10-关闭电荷泵

  //设置显示开关，0xAE-关闭显示，0xAF-开启显示
  OLEDWriteByte(0xAE, OLED_CMD);  //关闭显示
}

/*********************************************************************************************************
* 函数名称：OLEDRefreshGRAM
* 函数功能：将STM32的GRAM写入到SSD1306的GRAM 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void  OLEDRefreshGRAM(void)
{
  unsigned char i;
  unsigned char n;
                                          
  for(i = 0; i < 8; i++)                  //遍历每一页
  {                                       
    OLEDWriteByte(0xb0 + i, OLED_CMD);    //设置页地址（0～7）
    OLEDWriteByte(0x00, OLED_CMD);        //设置显示位置—列低地址
    OLEDWriteByte(0x10, OLED_CMD);        //设置显示位置—列高地址 
    for(n = 0; n < 128; n++)              //遍历每一列
    {
      //通过循环将微控制器的GRAM写入到SSD1306的GRAM
      OLEDWriteByte(s_arrOLEDGRAM[n][i], OLED_DATA); 
    }
  }   
}

/*********************************************************************************************************
* 函数名称：OLEDClear
* 函数功能：清屏函数，清完屏整个屏幕是黑色的，和没点亮一样
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/  
void  OLEDClear(void)  
{  
  unsigned char i;
  unsigned char n;
                                  
  for(i = 0; i < 8; i++)          //遍历每一页
  {                               
    for(n = 0; n < 128; n++)      //遍历每一列
    {                             
      s_arrOLEDGRAM[n][i] = 0x00;   //将指定点清零
    }                             
  }                               
                                  
  OLEDRefreshGRAM();  //将微控制器的GRAM写入到SSD1306的GRAM
}

/*********************************************************************************************************
* 函数名称：OLEDShowChar
* 函数功能：在指定位置显示一个字符
* 输入参数：x取值范围为0～127，y取值范围为0～63，chr为待显示的字符，size用于选择字体（16/12），mode为取模方式
*           mode为0反白显示，mode为1正常显示
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：有6种取模方式
*********************************************************************************************************/
void  OLEDShowChar(unsigned char x, unsigned char y, unsigned char chr, unsigned char size, unsigned char mode)
{ 
  unsigned char  temp;                         //用来存放字符顺向逐列式的相对位置
  unsigned char  t1;                           //循环计数器1
  unsigned char  t2;                           //循环计数器2
  unsigned char  y0 = y;                       //当前操作的行数
                                      
  chr = chr - ' ';                  //得到相对于空格（ASCII为0x20）的偏移值，求出要chr在数组中的索引
                                      
  for(t1 = 0; t1 < size; t1++)      //循环逐列显示
  {                                   
    if(size == 12)                  //判断字号大小，选择相对的顺向逐列式
    {                                 
      temp = g_iASCII1206[chr][t1]; //取出字符在g_iASCII1206数组中的第t1列
    }                                 
    else                              
    {                                 
      temp = g_iASCII1608[chr][t1]; //取出字符在g_iASCII1608数组中的第t1列                   
    }                                 
                                      
    for(t2 = 0; t2 < 8; t2++)       //在一个字符的第t2列的横向范围（8个像素）内显示点
    {                                 
      if(temp & 0x80)               //取出temp的最高位，并判断为0还是1
      {                             
        OLEDDrawPoint(x, y, mode);  //如果temp的最高位为1填充指定位置的点
      }                               
      else                            
      {                               
        OLEDDrawPoint(x, y, !mode); //如果temp的最高位为0清除指定位置的点
      }                               
                                      
      temp <<= 1;                   //左移一位，次高位移到最高位
      y++;                          //进入下一行
                                    
      if((y - y0) == size)          //如果显示完一列
      {                               
        y = y0;                     //行号回到原来的位置
        x++;                        //进入下一列
        break;                      //跳出上面带#的循环
      }
    } 
  } 
}

/*********************************************************************************************************
* 函数名称：OLEDShowNum
* 函数功能：显示数字
* 输入参数：x-y为起点坐标，len为数字的位数，size为字体大小，mode为模式（0-填充模式，1-叠加模式）
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：num：数值（0～4294967295）
*********************************************************************************************************/         
void  OLEDShowNum(unsigned char x, unsigned char y, unsigned int num, unsigned char len, unsigned char size)
{
  unsigned char t;                                                       //循环计数器
  unsigned char temp;                                                    //用来存放要显示数字的各个位
  unsigned char enshow = 0;                                              //区分0是否为高位0标志位
                                                                
  for(t = 0; t < len; t++)                                      
  {                                                             
    temp = (num / CalcPow(10, len - t - 1) ) % 10;            //按从高到低取出要显示数字的各个位，存到temp中
    if(enshow == 0 && t < (len - 1))                          //如果标记enshow为0并且还未取到最后一位
    {                                                           
      if(temp == 0 )                                          //如果temp等于0
      {                                                         
        OLEDShowChar(x + (size / 2) * t, y, ' ', size, 1);    //此时的0在高位，用空格替代
        continue;                                             //提前结束本次循环，进入下一次循环
      }                                                         
      else                                                      
      {                                                         
        enshow = 1;                                           //否则将标记enshow置为1
      }                                                                                                                        
    }                                                           
    OLEDShowChar(x + (size / 2) * t, y, temp + '0', size, 1); //在指定位置显示得到的数字
  }
}

/*********************************************************************************************************
* 函数名称：OLEDShowString
* 函数功能：显示字符串
* 输入参数：x-y为起点坐标，p为字符串起始地址
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：用16号字体显示
*********************************************************************************************************/
void  OLEDShowString(unsigned char x, unsigned char y, const unsigned char* p)
{
#define MAX_CHAR_POSX 122             //OLED屏幕横向的最大范围
#define MAX_CHAR_POSY 58              //OLED屏幕纵向的最大范围
                                      
  while(*p != '\0')                   //指针不等于结束符时，循环进入
  {                                   
    if(x > MAX_CHAR_POSX)             //如果x超出指定最大范围，x赋值为0
    {                                 
      x  = 0;                         
      y += 16;                        //显示到下一行左端
    }                                 
                                      
    if(y > MAX_CHAR_POSY)             //如果y超出指定最大范围，x和y均赋值为0
    {                                 
      y = x = 0;                      //清除OLED屏幕内容
      OLEDClear();                    //显示到OLED屏幕左上角
    }                                 
                                      
    OLEDShowChar(x, y, *p, 16, 1);    //指定位置显示一个字符
                                      
    x += 8;                           //一个字符横向占8个像素点
    p++;                              //指针指向下一个字符
  }  
}
