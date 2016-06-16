#ifndef _MC_SENSOR_H
#define _MC_SENSOR_H

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_common_define.h"
#include "mc_init_8051.h"


/* 用于控制传感器操作的函数 */

/*********************************************
* name:	 		mc_init_sensor
* description:	初始化传感器
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sensor(void);

/*********************************************
* name:	 		mc_init_ADXL345
* description:	初始化三轴线加速度传感器
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ADXL345(void);

/*********************************************
* name:	 		mc_init_ITG3205
* description:	初始化三轴陀螺仪（角度传感器）
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ITG3205(void);

/*********************************************
* name:	 		mc_init_HMC5883L
* description:	初始化三轴电子罗盘（磁场传感器）
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_HMC5883L(void);

#endif /* _MC_SENSOR_H */