#ifndef	 _MC_INIT_8051_H
#define  _MC_INIT_8051_H

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_common_define.h"

extern float Pitch, Roll, Yaw;

/* ��ʼ�����к�8051�йص�Ӳ����Դ������I/O���ã����Ź������񣬴��ڣ�IIC�� */

/*********************************************
* name:	 		mc_init_watchdog
* description:	��ʼ�����ÿ��Ź�
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_watchdog(void);

/*********************************************
* name:	 		mc_init_sysclk
* description:	��ʼ������ϵͳʱ��
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sysclk(void);

/*********************************************
* name:	 		mc_init_port
* description:	��ʼ������ϵͳʱ��
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_port(void);

/*********************************************
* name:	 		mc_init_uart0
* description:	��ʼ������uart0
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_uart0(void);

/*********************************************
* name:	 		mc_init_timer2
* description:	16λ�Զ����أ�����ÿ10ms�򴮿ڷ�����Ԫ��
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer2();

/*********************************************
* name:	 		mc_init_timer3
* description:	16λ�Զ����أ�����ÿ1ms��IIC���߶�ȡ������������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer3();

/*********************************************
* name:	 		mc_uart0_interrupt
* description:	uart0�жϴ�������ÿ10ms����һ����Ԫ��
* input:		NONE
* return:		NONE
*********************************************/
void mc_uart0_ISR(void);

/*********************************************
* name:	 		mc_uart0_interrupt
* description:	i2c�жϴ�������ÿ��1ms��һ�δ���������
* input:		NONE
* return:		NONE
*********************************************/
void mc_i2c_interrupt(void);

/*********************************************
* name:	 		delay_5us
* description:	5us��ʱ
* input:		NONE
* return:		NONE
*********************************************/
void delay_5us(void);

/*********************************************
* name:	 		delay_ms
* description:	5us �ı���
* input:		��Ҫ��ʱ��us����������5�ı���
* return:		NONE
*********************************************/
void delay_us(uchar n);

/*********************************************
* name:	 		delay_5ms
* description:	5ms��ʱ
* input:		NONE
* return:		NONE
*********************************************/
void delay_5ms(void);

/*********************************************
* name:	 		delay_ms
* description:	5ms �ı���
* input:		��Ҫ��ʱ��ms����������5�ı���
* return:		NONE
*********************************************/
void delay_ms(uchar n);

/**************************************
��ʼ�ź�
**************************************/
void mc_smbus_start();

/**************************************
ֹͣ�ź�
**************************************/
void mc_smbus_stop();

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void mc_smbus_sendack(bit ack);

/**************************************
����Ӧ���ź�
**************************************/
bit mc_smbus_recvack();

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void mc_smbus_sendbyte(BYTE dat);

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE mc_smbus_recvbyte();

//******���ֽ�д��*******************************************

void single_write_smbus(uchar SlaveAddress, uchar REG_Address, uchar REG_data);

//********���ֽڶ�ȡ*****************************************
uchar single_read_smbus(uchar SlaveAddress, uchar REG_Address);

//*********************************************************
//
//���������������ڲ����ݣ�ÿ����������6���ֽ�
//
//*********************************************************
void multiple_read_smbus(uchar SlaveAddress, uchar REG_Address, uchar *buf);

/*********************************************
* name:	 		float_to_ascii
* description:	�ֱ�ȡ��ÿһλ�������ڴ�����������������ASCII��ʽ
* input:		NONE
* return:		NONE
*********************************************/
void float_to_ascii(float input, uchar *s);

#endif /* _MC_INIT_8051_H */