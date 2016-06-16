#ifndef	 _MC_INIT_8051_H
#define  _MC_INIT_8051_H

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_common_define.h"

extern float Pitch, Roll, Yaw;

/* 初始化所有和8051有关的硬件资源，包括I/O配置，看门狗，晶振，串口，IIC等 */

/*********************************************
* name:	 		mc_init_watchdog
* description:	初始化配置看门狗
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_watchdog(void);

/*********************************************
* name:	 		mc_init_sysclk
* description:	初始化配置系统时钟
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sysclk(void);

/*********************************************
* name:	 		mc_init_port
* description:	初始化配置系统时钟
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_port(void);

/*********************************************
* name:	 		mc_init_uart0
* description:	初始化配置uart0
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_uart0(void);

/*********************************************
* name:	 		mc_init_timer2
* description:	16位自动重载，用于每10ms向串口发送四元数
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer2();

/*********************************************
* name:	 		mc_init_timer3
* description:	16位自动重载，用于每1ms从IIC总线读取传感器的数据
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer3();

/*********************************************
* name:	 		mc_uart0_interrupt
* description:	uart0中断处理函数，每10ms发送一次四元数
* input:		NONE
* return:		NONE
*********************************************/
void mc_uart0_ISR(void);

/*********************************************
* name:	 		mc_uart0_interrupt
* description:	i2c中断处理函数，每隔1ms读一次传感器数据
* input:		NONE
* return:		NONE
*********************************************/
void mc_i2c_interrupt(void);

/*********************************************
* name:	 		delay_5us
* description:	5us延时
* input:		NONE
* return:		NONE
*********************************************/
void delay_5us(void);

/*********************************************
* name:	 		delay_ms
* description:	5us 的倍数
* input:		需要延时的us数，必须是5的倍数
* return:		NONE
*********************************************/
void delay_us(uchar n);

/*********************************************
* name:	 		delay_5ms
* description:	5ms延时
* input:		NONE
* return:		NONE
*********************************************/
void delay_5ms(void);

/*********************************************
* name:	 		delay_ms
* description:	5ms 的倍数
* input:		需要延时的ms数，必须是5的倍数
* return:		NONE
*********************************************/
void delay_ms(uchar n);

/**************************************
起始信号
**************************************/
void mc_smbus_start();

/**************************************
停止信号
**************************************/
void mc_smbus_stop();

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void mc_smbus_sendack(bit ack);

/**************************************
接收应答信号
**************************************/
bit mc_smbus_recvack();

/**************************************
向IIC总线发送一个字节数据
**************************************/
void mc_smbus_sendbyte(BYTE dat);

/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE mc_smbus_recvbyte();

//******单字节写入*******************************************

void single_write_smbus(uchar SlaveAddress, uchar REG_Address, uchar REG_data);

//********单字节读取*****************************************
uchar single_read_smbus(uchar SlaveAddress, uchar REG_Address);

//*********************************************************
//
//连续读出传感器内部数据，每个传感器是6个字节
//
//*********************************************************
void multiple_read_smbus(uchar SlaveAddress, uchar REG_Address, uchar *buf);

/*********************************************
* name:	 		float_to_ascii
* description:	分别取出每一位数，用于串口输出，串口输出是ASCII形式
* input:		NONE
* return:		NONE
*********************************************/
void float_to_ascii(float input, uchar *s);

#endif /* _MC_INIT_8051_H */