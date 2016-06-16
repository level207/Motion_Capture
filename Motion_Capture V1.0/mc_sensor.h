#ifndef _MC_SENSOR_H
#define _MC_SENSOR_H

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_common_define.h"
#include "mc_init_8051.h"


/* ���ڿ��ƴ����������ĺ��� */

/*********************************************
* name:	 		mc_init_sensor
* description:	��ʼ��������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sensor(void);

/*********************************************
* name:	 		mc_init_ADXL345
* description:	��ʼ�������߼��ٶȴ�����
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ADXL345(void);

/*********************************************
* name:	 		mc_init_ITG3205
* description:	��ʼ�����������ǣ��Ƕȴ�������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ITG3205(void);

/*********************************************
* name:	 		mc_init_HMC5883L
* description:	��ʼ������������̣��ų���������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_HMC5883L(void);

#endif /* _MC_SENSOR_H */