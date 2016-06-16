#ifndef	 _MC_COMMON_DEFINE_H
#define  _MC_COMMON_DEFINE_H

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include <math.h>

#define SYSCLK      	12000000 /* ϵͳʱ�� Hz */
#define BAUDRATE        115200   /* UART0�Ĳ����� bps��������ʵ18000  == 180bit * 100�͹��ˣ�����ÿ��ɼ�100������ */
#define SAMPLE			144000  /* �ɼ��Ĳ�����  18 * 8 * 1000 = 144000, ��1 sample per 1ms */ 
#define SMB_FREQUENCY	100000 	 /* SMBus�� SCL ʱ�� 10kHz~100KHz */
#define uchar 			unsigned char
#define uint  			unsigned int
typedef unsigned char   BYTE;
typedef unsigned short  WORD;
#define WRITE           0x00     /* SMBus WRITE command */ 
#define READ            0x01     /* SMBus READ command */ 

//���崫�����ĵ�ַ��SLAVE��ַ 
#define SLAVE_ADDR_ADXL345		0xA6	    //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸ġ�ALT  ADDRESS���Žӵ�ʱ��ַΪ0xA6���ӵ�Դʱ��ַΪ0x3A
#define	SLAVE_ADDR_ITG3205      0xD0        //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
#define	SLAVE_ADDR_HMC5883L     0x3C        //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

#define Accel_Zout_Offset		0 //600
#define Gyro_Xout_Offset	    0 //-70
#define Gyro_Yout_Offset		0 //25
#define Gyro_Zout_Offset		0 //-10

#define Kp 2.0f                  // ��������֧��������accelerometer/magnetometer
#define Ki 0.005f                // ��������ִ�����������ǵ��ν�gyroscopeases
#define halfT 0.5f//18.0f                // �������ڵ�һ�룬�����Ϊ����������̬�㷨��ʱ���

#endif  /* _MC_COMMON_DEFINE_H */