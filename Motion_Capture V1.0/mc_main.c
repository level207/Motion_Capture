/* 程序的入口函数 */

#include <c8051f340.h>
#include <stdio.h>
#include "mc_common_define.h"
#include "mc_init_8051.h"
#include "mc_sensor.h"
//#include "mc_algorithm.h"
#include "test.h"

uchar data_write[18] = {0};

void main()
{
	float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
    signed short int gyro[3] = {0}, accel[3] = {0}, mag[3] = {0};
	int i = 0;

	mc_init_watchdog();		/* 初始化配置看门狗 */
	mc_init_sysclk();		/* 初始化配置晶振 */

	mc_init_port();			/* 初始化IO和交叉开关 */
	mc_init_uart0(); 		/* 初始化配置uart0 timer1*/
	delay_ms(200);


	mc_init_timer2();		/* 16位自动重载，用于每10ms向串口发送四元数 */
	//mc_init_timer3();		/* 16位自动重载，用于每1ms从IIC总线读取传感器的数据 */

	mc_init_sensor();		/* 初始化配置9轴传感器 */
	delay_ms(200);
	
    //init_quaternion();      /* 初始化四元数	*/

	//EA = 1; /* 中断总开关 */

	while(1)				/* 循环读出传感器的值，并处理 */
	{
		multiple_read_smbus(0xA6, 0x32, data_write);       		//连续读出ADXL345数据，存储在BUF中
		multiple_read_smbus(0xD0, 0x1D, &(data_write[6]));      //连续读出ITG3205数据，存储在BUF中
		multiple_read_smbus(0x3C, 0x03, &(data_write[12]));     //连续读出HMC5883L数据，存储在BUF中

	    accel[0]=(signed short int)((data_write[1]<<8) | data_write[0]);//加速度计的合成注意顺序
	    accel[1]=(signed short int)((data_write[3]<<8) | data_write[2]);
		accel[2]=(signed short int)((data_write[5]<<8) | data_write[4])   + Accel_Zout_Offset;

		gyro[0] =(signed short int)((data_write[6]<<8) | data_write[7])   + Gyro_Xout_Offset;
		gyro[1] =(signed short int)((data_write[8]<<8) | data_write[9])   + Gyro_Yout_Offset;
		gyro[2] =(signed short int)((data_write[10]<<8) | data_write[11]) + Gyro_Zout_Offset;

	    mag[0]  =(signed short int)((data_write[12]<<8) | data_write[13]);
	    mag[1]  =(signed short int)((data_write[14]<<8) | data_write[15]);
		mag[2]  =(signed short int)((data_write[16]<<8) | data_write[17]);

		//单位转化成重力加速度的单位g：m/s2
		init_ax=(float)(accel[0] * 3.9 / 1000);	   
	    init_ay=(float)(accel[1] * 3.9 / 1000);
        init_az=(float)(accel[2] * 3.9 / 1000);

		//单位转化成：弧度/s，1/(14.375 * 57.3)
		init_gx=(float)(gyro[0] / (14.375 * 57.3));    
		init_gy=(float)(gyro[1] / (14.375 * 57.3));
		init_gz=(float)(gyro[2] / (14.375 * 57.3));

        //进行x y轴的校准，未对z轴进行校准，参考ST的校准方法 
		//磁场传感器为原始数据
        init_mx =(float)mag[0];						
    	init_my =(float)mag[1];
    	init_mz =(float)mag[2];

		 //SBUF0 = 'a';
		 //delay_5ms();
		printf("ax=%f, ay=%f, az=%f\n", init_ax, init_ay, init_az);
		//printf("gx=%f, gy=%f, gz=%f\n", init_gx, init_gy, init_gz);
		//printf("mx=%f, my=%f, mz=%f\n", init_mx, init_my, init_mz);


		//计算出欧拉角
    	//Roll  = atan2(init_ay, init_az);
    	//Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
    	//Yaw   = -atan2(init_mx*cos(Roll) + init_my*sin(Roll)*sin(Pitch) + init_mz*sin(Roll)*cos(Pitch),
        //               init_my*cos(Pitch) - init_mz*sin(Pitch));                       //atan2(mx, my);
		  
		//printf("Yaw=%f, Pitch=%f, Roll=%f\n", Yaw, Pitch, Roll);
		//printf("Yaw=%f, Pitch=%f, Roll=%f\n", Yaw*57.3, Pitch*57.3, Roll*57.3);
	
		//四元数更新算法
        //AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz);

		//delay_ms(10);
		 
	}

	return;
}
