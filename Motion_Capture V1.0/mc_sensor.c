/* 用于控制传感器操作的函数 */

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_sensor.h"

/*********************************************
* name:	 		mc_init_sensor
* description:	初始化传感器
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sensor(void)
{
	mc_init_ADXL345(); 		//三轴线加速度传感器
	mc_init_ITG3205();		//三轴陀螺仪（角速度传感器）
	mc_init_HMC5883L();		//三轴电子罗盘（磁场传感器）

	return;
}

/*********************************************
* name:	 		mc_init_ADXL345
* description:	初始化三轴线加速度传感器
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ADXL345(void)
{
	
	//single_write_smbus(0xA6, 0x31, 0x0B);   //测量范围,正负16g，13位模式
	/* 设置采样频率为 1600Hz 即1.6 sample per ms */
   	//single_write_smbus(0xA6, 0x2C, 0x0E);   //速率设定为1600Hz 参考pdf13页  /* Table 7和Table 8设置速率 */
   	//single_write_smbus(0xA6, 0x2D, 0x08);   //选择电源模式   参考pdf24页
   	//single_write_smbus(0xA6, 0x2E, 0x80);   //使能 DATA_READY 中断
   	//single_write_smbus(0xA6, 0x1E, 0x00);   //X 偏移量 根据测试传感器的状态写入pdf29页
   	//single_write_smbus(0xA6, 0x1F, 0x00);   //Y 偏移量 根据测试传感器的状态写入pdf29页
   	//single_write_smbus(0xA6, 0x20, 0x05);   //Z 偏移量 根据测试传感器的状态写入pdf29页
	
	single_write_smbus(0xA6, 0x31, 0x0B);   //测量范围,正负16g，13位模式
   	single_write_smbus(0xA6, 0x2C, 0x08);   //速率设定为12.5 参考pdf13页
   	single_write_smbus(0xA6, 0x2D, 0x08);   //选择电源模式   参考pdf24页
   	single_write_smbus(0xA6, 0x2E, 0x80);   //使能 DATA_READY 中断
   	single_write_smbus(0xA6, 0x1E, 0x00);   //X 偏移量 根据测试传感器的状态写入pdf29页
   	single_write_smbus(0xA6, 0x1F, 0x00);   //Y 偏移量 根据测试传感器的状态写入pdf29页
   	single_write_smbus(0xA6, 0x20, 0x05);   //Z 偏移量 根据测试传感器的状态写入pdf29页

	return;
}

/*********************************************
* name:	 		mc_init_ITG3205
* description:	初始化三轴陀螺仪（角度传感器）
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ITG3205(void)
{
 	single_write_smbus(0xD0, 0x3E, 0x80);   // 0x80
    /* 设置采样频率为 F = 1kHz / (0 + 1) = 1000Hz, or 1ms per sample 低通滤波带宽为 5Hz*/
	//single_write_smbus(0xD0, 0x15, 0x07);	//SMPLRT_DIV（0-255） 
	single_write_smbus(0xD0, 0x15, 0x00);	//SMPLRT_DIV（0-255） 
	single_write_smbus(0xD0, 0x16, 0x1E);   //DLPF_FS
	single_write_smbus(0xD0, 0x17, 0x00);	//G_INIT_CFG
	single_write_smbus(0xD0, 0x3E, 0x00);   //

	return;
}

/*********************************************
* name:	 		mc_init_HMC5883L
* description:	初始化三轴电子罗盘（磁场传感器）
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_HMC5883L(void)
{
    /*设置为连续输出模式，如果有必要，数据可以从数据输出寄存器重新读取 输出频率为 30Hz*/
	single_write_smbus(0x3C, 0x00, 0x14);   //
	single_write_smbus(0x3C, 0x02, 0x00);   //

	return;
}