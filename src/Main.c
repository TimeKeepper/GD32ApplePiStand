#include "Main.h"

void InitSoftware(void)
{
    InitEXTI();          //初始化EXTI模块
    SystemInit();        //系统初始化
    InitRCU();           //初始化RCU模块
    InitNVIC();          //初始化NVIC模块  
    InitUART0(115200);   //初始化UART模块
    InitTimer();         //初始化Timer模块
    InitLED();           //初始化LED模块
    InitSysTick();       //初始化SysTick模块
    InitKeyOne();        //初始化KeyOne模块
    InitProcKeyOne();    //初始化ProcKeyOne模块
    InitOLED();          //初始化OLED模块
}

void InitHardware(void)
{

}

static signed short s_iCnt5 = 0;
static void Proc2msTask(void){
    unsigned char recData;
    if(Get2msFlag()){
        RunClockPer2ms();
        if(s_iCnt5++>=5){
            s_iCnt5 = 0;
            ScanKeyOne(KEY_NAME_KEY1, ProcKeyUpKey1, ProcKeyDownKey1);
            ScanKeyOne(KEY_NAME_KEY2, ProcKeyUpKey2, ProcKeyDownKey2);
            ScanKeyOne(KEY_NAME_KEY3, ProcKeyUpKey3, ProcKeyDownKey3);
        }
        while(ReadUART0(&recData, 1)){
            recData++;
            WriteUART0(&recData, 1);
        }
        LEDFlicker(250);
        Clr2msFlag();
    }
}

static void Proc1sTask(void){
    if(Get1SecFlag()){
        DispTime(GetTimeVal(TIME_VAL_HOUR), GetTimeVal(TIME_VAL_MIN), GetTimeVal(TIME_VAL_SEC));
        Clr1SecFlag();
    }
}

int main(void)
{
    InitSoftware();
    InitHardware();

    OLEDShowString(8, 0, (const unsigned char*)"Hello World!");
    
    PauseClock(FALSE);
    SetTimeVal(TIME_VAL_HOUR, 0);
    SetTimeVal(TIME_VAL_MIN, 0);
    SetTimeVal(TIME_VAL_SEC, 0);

    while (1){
        OLEDRefreshGRAM();
        Proc2msTask();
        Proc1sTask();
    }
}
