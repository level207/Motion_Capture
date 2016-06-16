#ifndef	 _MC_COMMON_DEFINE_H
#define  _MC_COMMON_DEFINE_H

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include <math.h>

#define SYSCLK      	12000000 /* 系统时钟 Hz */
#define BAUDRATE        115200   /* UART0的波特率 bps，我们其实18000  == 180bit * 100就够了，可以每秒采集100组数据 */
#define SAMPLE			144000  /* 采集的波特率  18 * 8 * 1000 = 144000, 即1 sample per 1ms */ 
#define SMB_FREQUENCY	100000 	 /* SMBus的 SCL 时钟 10kHz~100KHz */
#define uchar 			unsigned char
#define uint  			unsigned int
typedef unsigned char   BYTE;
typedef unsigned short  WORD;
#define WRITE           0x00     /* SMBus WRITE command */ 
#define READ            0x01     /* SMBus READ command */ 

//定义传感器的地址，SLAVE地址 
#define SLAVE_ADDR_ADXL345		0xA6	    //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改。ALT  ADDRESS引脚接地时地址为0xA6，接电源时地址为0x3A
#define	SLAVE_ADDR_ITG3205      0xD0        //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define	SLAVE_ADDR_HMC5883L     0x3C        //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

#define Accel_Zout_Offset		0 //600
#define Gyro_Xout_Offset	    0 //-70
#define Gyro_Yout_Offset		0 //25
#define Gyro_Zout_Offset		0 //-10

#define Kp 2.0f                  // 比例增益支配收敛率accelerometer/magnetometer
#define Ki 0.005f                // 积分增益执政速率陀螺仪的衔接gyroscopeases
#define halfT 0.5f//18.0f                // 采样周期的一半，可理解为两次运行姿态算法的时间差

#endif  /* _MC_COMMON_DEFINE_H */