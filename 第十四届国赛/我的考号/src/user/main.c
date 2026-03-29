// 包含 STC15F2K60S2 单片机的头文件，提供了寄存器定义等
#include <STC15F2K60S2.H>


// 包含自定义的初始化、外设驱动和功能模块的头文件
#include "init.h" // 系统初始化
#include "led.h"  // LED 模块
#include "seg.h"  // 数码管模块
#include "key.h"  // 按键模块

#include "ds1302.h"     // DS1302 实时时钟模块
#include "onewire.h"    // 单总线协议，用于温度传感器
#include "iic.h"        // I2C 协议，用于 ADC/DAC 和 EEPROM
#include "ultrasound.h" // 超声波模块
#include "uart.h"       // 串口通信模块

// 包含标准库头文件
#include "string.h" // 字符串处理函数
#include "stdio.h"  // 标准输入输出函数，如 printf 和 sscanf

// 全局变量定义

// `idata` 存储在内部高速 RAM 中，访问速度快
// 系统滴答计时器，由 Timer1 中断每毫秒增加一次
idata unsigned long int uwTick;

// `pdata` 存储在外部 RAM 的分页区域，访问速度较快
// LED灯的状态数组，对应8个LED的亮灭
pdata unsigned char ucLed[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// 数码管显示缓冲区，存储要在8位数码管上显示的数字或符号
pdata unsigned char Seg_Buf[8] = {10, 10, 10, 10, 10, 10, 10, 10}; // 10代表不显示
// 当前正在刷新的数码管位置索引 (0-7)
idata unsigned char Seg_Pos = 0;

// 按键状态变量
idata unsigned char Key_Val;  // 当前按键值
idata unsigned char Key_Old;  // 上一次的按键值，用于检测边沿
idata unsigned char Key_Up;   // 按键抬起事件标志
idata unsigned char Key_Down; // 按键按下事件标志

// `idata` 存储在内部高速 RAM 中
// 存储放大10倍的温度值，以处理一位小数
idata unsigned int Temperature_10x;

// `idata` 存储在内部高速 RAM 中
// 存储超声波测量的距离
idata unsigned int Distance;
idata unsigned int distance_old;
pdata unsigned int distance_data[32];
idata unsigned char distance_count;
idata bit distance_record_flag;
idata unsigned int time_6000ms;
idata bit recording_completed_flag;
idata unsigned char distance_output_index;
idata bit dac_out_flag;
idata unsigned int time_1000ms;

// 数码管显示模式
// 0:测距界面 1:参数界面 2:工厂模式界面
idata unsigned char Seg_Show_Mode;

// 测距模式
idata bit distance_mode; // 0:cm 1:m

// 参数界面
idata unsigned char para_mode; // 参数编号
idata unsigned char distance_para = 40;
idata unsigned char temperature_para = 30;

// 工厂模式
idata unsigned char factory_mode;         // 工厂模式编号 0:校准值 1：介质设置 2：DAC输出
idata char calibration_data;              // 校准值
idata unsigned char calibration_data_abs; // 校准值绝对值

idata unsigned int transmission_speed = 340;

idata unsigned char dac_output_lower_limit = 10;

idata bit S89_flag;
idata unsigned int time_2000ms;

idata bit factory_flag;
idata bit led_flag;
idata unsigned char time_100ms;

/**
 * @brief 按键处理函数
 * 检测按键的按下和抬起事件，并根据按键执行相应操作。
 */
void Key_Proc()
{
    Key_Val = Key_Read(); // 读取当前按key的状态
    // 使用异或运算检测按键状态变化，& Key_Val 检测下降沿（按下）
    Key_Down = Key_Val & (Key_Val ^ Key_Old);
    // 使用异或运算检测按键状态变化，& ~Key_Val 检测上降沿（抬起）
    Key_Up = ~Key_Val & (Key_Val ^ Key_Old);
    Key_Old = Key_Val; // 更新旧的按键值

    if (distance_record_flag == 1)
        return;

    if (Key_Old == 89)
    {
        S89_flag = 1;
        if (time_2000ms >= 2000)
        {
            Seg_Show_Mode = 0;
            para_mode = 0;
            distance_para = 40;
            temperature_para = 30;
            factory_mode = 0;
            calibration_data = 0;
            transmission_speed = 340;
            dac_output_lower_limit = 10;
            S89_flag = 0;
        }
    }

    switch (Key_Down)
    {
    // 界面切换按键
    case 4:
        Seg_Show_Mode = (++Seg_Show_Mode) % 3;
        // 出测距界面
        if (Seg_Show_Mode == 1)
            distance_mode = 0;
        // 出参数界面
        if (Seg_Show_Mode == 2)
        {
            para_mode = 0;
            factory_flag = 1;
        }
        // 出工厂模式
        if (Seg_Show_Mode == 0)
        {
            factory_mode = 0;
            factory_flag = 0;
        }
        break;

    // 子界面切换按键
    case 5:
        switch (Seg_Show_Mode)
        {
        // 测距界面切换单位
        case 0:
            distance_mode ^= 1;
            break;
        // 参数界面切换参数
        case 1:
            para_mode ^= 1;
            break;
        // 工厂模式切换
        case 2:
            factory_mode = (++factory_mode) % 3;
            break;
        }
        break;

    // 加按键
    case 8:
        switch (Seg_Show_Mode)
        {
        // 测距界面
        case 0:
            distance_record_flag = 1;
						time_6000ms = 0;
						recording_completed_flag = 0;
					  distance_count = 0;
						distance_old = 0xffff;
						distance_output_index = 0;
            break;

        // 参数界面
        case 1:
            // 距离参数
            if (para_mode == 0)
                distance_para = (distance_para == 90) ? 10 : distance_para + 10;
            // 温度参数
            else
                temperature_para = (temperature_para == 80) ? 0 : temperature_para + 1;
            break;

        // 工厂模式
        case 2:
            // 校准值
            if (factory_mode == 0)
                calibration_data = (calibration_data == 90) ? -90 : calibration_data + 5;
            // 传输速度值
            else if (factory_mode == 1)
                transmission_speed = (transmission_speed == 9990) ? 10 : transmission_speed + 10;
            else
                dac_output_lower_limit = (dac_output_lower_limit == 20) ? 1 : dac_output_lower_limit + 1;
            break;
        }
        break;

    // 减按键
    case 9:
        switch (Seg_Show_Mode)
        {
        // 测距界面
        case 0:
						dac_out_flag = 1;

            break;

        // 参数界面
        case 1:
            // 距离参数
            if (para_mode == 0)
                distance_para = (distance_para == 10) ? 90 : distance_para - 10;
            // 温度参数
            else
                temperature_para = (temperature_para == 0) ? 80 : temperature_para - 1;
            break;

        // 工厂模式
        case 2:
            // 校准值
            if (factory_mode == 0)
                calibration_data = (calibration_data == -90) ? 90 : calibration_data - 5;
            // 传输速度值
            else if (factory_mode == 1)
                transmission_speed = (transmission_speed == 10) ? 9990 : transmission_speed - 10;
            else
                dac_output_lower_limit = (dac_output_lower_limit == 1) ? 20 : dac_output_lower_limit - 1;
            break;
        }

        break;
    }
}

/**
 * @brief 数码管显示处理函数
 * 根据当前的 Seg_Show_Mode，准备要显示在数码管上的数据。
 */
void Seg_Proc()
{
    // 根据显示模式来填充数码管显示缓冲区 Seg_Buf
    switch (Seg_Show_Mode)
    {
    // 0:测距界面
    case 0:
        Seg_Buf[0] = Temperature_10x / 100 % 10;
        Seg_Buf[1] = Temperature_10x / 10 % 10 + ',';
        Seg_Buf[2] = Temperature_10x % 10;
        Seg_Buf[3] = 11;
        // cm
        if (distance_mode == 0)
        {
            Seg_Buf[4] = (Distance / 1000 % 10 == 0) ? 10 : Distance / 1000 % 10;
            Seg_Buf[5] = (Distance / 100 % 10 == 0 && Seg_Buf[4] == 10) ? 10 : Distance / 100 % 10;
            Seg_Buf[6] = (Distance / 10 % 10 == 0 && Seg_Buf[5] == 10) ? 10 : Distance / 10 % 10;
            Seg_Buf[7] = Distance % 10;
        }
        // m
        else
        {
            Seg_Buf[4] = (Distance / 1000 % 10 == 0) ? 10 : Distance / 1000 % 10;
            Seg_Buf[5] = Distance / 100 % 10 + ',';
            Seg_Buf[6] = Distance / 10 % 10;
            Seg_Buf[7] = Distance % 10;
        }
        break;

    // 1：参数界面
    case 1:
        Seg_Buf[0] = 12;
        Seg_Buf[1] = para_mode + 1;
        Seg_Buf[2] = 10;
        Seg_Buf[3] = 10;
        Seg_Buf[4] = 10;
        Seg_Buf[5] = 10;
        if (para_mode == 0)
        {
            Seg_Buf[6] = distance_para / 10 % 10;
            Seg_Buf[7] = distance_para % 10;
        }
        else
        {
            Seg_Buf[6] = (temperature_para / 10 % 10 == 0) ? 10 : temperature_para / 10 % 10;
            Seg_Buf[7] = temperature_para % 10;
        }
        break;

    // 2:工厂模式界面
    case 2:
        Seg_Buf[0] = 13;
        Seg_Buf[1] = factory_mode + 1;
        Seg_Buf[2] = 10;
        Seg_Buf[3] = 10;
        if (factory_mode == 0)
        {
            Seg_Buf[4] = 10;
            if (calibration_data >= 0)
            {
                Seg_Buf[5] = (calibration_data / 100 % 10 == 0) ? 10 : calibration_data / 100 % 10;
                Seg_Buf[6] = (calibration_data / 10 % 10 == 0 && Seg_Buf[5] == 10) ? 10 : calibration_data / 10 % 10;
                Seg_Buf[7] = calibration_data % 10;
            }
            else if (calibration_data > -10)
            {
                calibration_data_abs = -calibration_data;
                Seg_Buf[5] = 10;
                Seg_Buf[6] = 11;
                Seg_Buf[7] = calibration_data_abs % 10;
            }
            else
            {
                calibration_data_abs = -calibration_data;
                Seg_Buf[5] = 11;
                Seg_Buf[6] = calibration_data_abs / 10 % 10;
                Seg_Buf[7] = calibration_data_abs % 10;
            }
        }
        else if (factory_mode == 1)
        {
            Seg_Buf[4] = (transmission_speed / 1000 % 10 == 0) ? 10 : transmission_speed / 1000 % 10;
            Seg_Buf[5] = (transmission_speed / 100 % 10 == 0 && Seg_Buf[4] == 10) ? 10 : transmission_speed / 100 % 10;
            Seg_Buf[6] = (transmission_speed / 10 % 10 == 0 && Seg_Buf[5] == 10) ? 10 : transmission_speed / 10 % 10;
            Seg_Buf[7] = transmission_speed % 10;
        }
        else
        {
            Seg_Buf[4] = 10;
            Seg_Buf[5] = 10;
            Seg_Buf[6] = dac_output_lower_limit / 10 % 10 + ',';
            Seg_Buf[7] = dac_output_lower_limit % 10;
        }
        break;
    }
}

/**
 * @brief LED处理函数
 * 设置LED的状态，这里是固定的交替亮灭模式。
 */
void Led_Proc()
{
    unsigned char i;

    switch (Seg_Show_Mode)
    {
    // 测距界面
    case 0:
        if (Distance >= 255)
        {
            for (i = 0; i < 8; i++)
            {
                ucLed[i] = 1;
            }
        }
        else
        {
            for (i = 0; i < 8; i++)
            {
                ucLed[i] = (Distance >> i) & 1;
            }
        }
        break;
    // 参数界面
    case 1:
        for (i = 0; i < 7; i++)
            ucLed[i] = 0;
        ucLed[7] = 1;
        break;
    // 工厂模式
    case 2:
        for (i = 1; i < 8; i++)
            ucLed[i] = 0;
        ucLed[0] = led_flag;
        break;
    }
    Led_Disp(ucLed); // 实际的显示操作在Timer1中断中根据PWM进行

    if ((Distance >= distance_para - 5) && (Distance <= distance_para + 5) && (Temperature_10x <= temperature_para * 10))
        Relay(1);
    else
        Relay(0);
}

/**
 * @brief 获取DS18B20温度
 */
void Get_Temperature()
{
    // 调用单总线驱动函数读取温度，并放大10倍
    Temperature_10x = rd_temperature() * 10;
}

/**
 * @brief ADC/DAC处理函数
 * 读取光敏电阻和电位器的AD值，并设置DAC输出。
 */
void AD_DA()
{
}

/**
 * @brief 获取超声波测量的距离
 */
void Get_Distance()
{
    Distance = Ut_Wave_Data(transmission_speed) + calibration_data; // 调用超声波驱动函数
	
		if(distance_record_flag == 1 && distance_count < 32 && Distance != distance_old)
		{
			distance_data[distance_count] = Distance;
			distance_old = Distance;
			distance_count++;
		}			
}

/**
 * @brief 定时器1初始化函数
 * 配置为1ms中断的定时器，作为系统的心跳。
 */
void Timer1_Init(void) // 1毫秒@12.000MHz
{
    AUXR &= 0xBF; // 定时器时钟12T模式
    TMOD &= 0x0F; // 设置定时器模式 (清空T1的设置)
    TL1 = 0x18;   // 设置定时初始值 (65536-1000) -> FC18H
    TH1 = 0xFC;   // 设置定时初始值
    TF1 = 0;      // 清除TF1溢出标志
    TR1 = 1;      // 定时器1开始计时
    ET1 = 1;      // 使能定时器1中断
    EA = 1;       // 开启总中断
}

/**
 * @brief 定时器1中断服务函数
 * 每1ms执行一次，处理需要定时执行的任务。
 */
void Timer1_Isr(void) interrupt 3
{
    uwTick++; // 系统滴答计时器加1

    // 数码管动态扫描
    Seg_Pos = (++Seg_Pos) % 8; // 切换到下一位数码管
    // 判断显示缓冲区的值是否需要显示小数点
    // (通过给数字加上一个大偏移量','作为标志)
    if (Seg_Buf[Seg_Pos] > 20)
        // 显示数字，并点亮小数点
        Seg_Disp(Seg_Pos, Seg_Buf[Seg_Pos] - ',', 1);
    else
        // 显示数字，不点亮小数点
        Seg_Disp(Seg_Pos, Seg_Buf[Seg_Pos], 0);

    if (distance_record_flag == 1)
    {
        if (++time_6000ms == 6000)
        {
            distance_record_flag = 0;
            recording_completed_flag = 1;
        }
    }
    else
        time_6000ms = 0;

    if (S89_flag == 1)
    {
        if (++time_2000ms >= 2000)
            time_2000ms = 2001;
    }
    else
        time_2000ms = 0;

    if (factory_flag == 1)
    {
        if (++time_100ms == 100)
        {
            led_flag ^= 1;
            time_100ms = 0;
        }
    }
    else
    {
        time_100ms = 0;
        led_flag = 0;
    }
		
		if(dac_out_flag == 1)
		{
			if(++time_1000ms == 1000)
			{
				time_1000ms = 0;
				if (recording_completed_flag == 1 && distance_count> 0)
        {
					if (distance_data[distance_output_index] <= 10)
            Da_Write(((float)dac_output_lower_limit / 10.0) * 51.0);
          else if (distance_data[distance_output_index] >= 90)
            Da_Write(5 * 51);
          else
            Da_Write(((float)(5.0 - (float)dac_output_lower_limit / 10.0) * (float)(distance_data[distance_output_index] - 10) / 80.0 + (float)dac_output_lower_limit / 10.0) * 51.0);   
					if(distance_output_index < distance_count - 1)
						distance_output_index++;
				}
				else
					dac_out_flag = 0;
			}
		}
		else
			time_1000ms = 0;
}

// 简单任务调度器结构体定义
typedef struct
{
    void (*task_func)(void);   // 任务函数指针
    unsigned long int rate_ms; // 任务执行周期（毫秒）
    unsigned long int last_ms; // 上次执行的时间戳
} task_t;

// 任务列表
idata task_t Scheduler_Task[] = {
    {Led_Proc, 1, 0},          // LED任务，每1ms
    {Key_Proc, 10, 0},         // 按键任务，每10ms
    {Seg_Proc, 20, 0},         // 数码管任务，每20ms
    {Get_Temperature, 300, 0}, // 获取温度任务，每300ms
    {AD_DA, 150, 0},           // AD/DA任务，每150ms
    {Get_Distance, 120, 0},    // 获取距离任务，每120ms

};

idata unsigned char task_num; // 任务数量

/**
 * @brief 调度器初始化
 * 计算任务列表中的任务数量。
 */
void Scheduler_Init()
{
    task_num = sizeof(Scheduler_Task) / sizeof(task_t);
}

/**
 * @brief 调度器运行函数
 * 循环检查每个任务是否到达执行时间。
 */
void Scheduler_Run()
{
    unsigned char i;
    for (i = 0; i < task_num; i++)
    {
        unsigned long int now_time = uwTick; // 获取当前时间
        // 判断是否到达任务执行时间 (当前时间 >= 上次执行时间 + 周期)
        if (now_time >= (Scheduler_Task[i].rate_ms + Scheduler_Task[i].last_ms))
        {
            Scheduler_Task[i].last_ms = now_time; // 更新上次执行时间
            Scheduler_Task[i].task_func();        // 执行任务函数
        }
    }
}

/**
 * @brief 主函数
 */
void main()
{
    System_Init();    // 系统底层初始化
    Scheduler_Init(); // 任务调度器初始化
    Timer1_Init();    // 定时器1（系统心跳）初始化

    // 主循环
    while (1)
    {
        Scheduler_Run(); // 循环执行任务调度器
    }
}
