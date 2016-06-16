/* 初始化所有和8051有关的硬件资源，包括I/O配置，看门狗，晶振，串口，IIC等 */

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_init_8051.h"
#include "mc_algorithm.h"

sbit	  SCL=P0^0;      //IIC时钟引脚定义
sbit 	  SDA=P0^1;      //IIC数据引脚定义
float Pitch = 0, Roll = 0, Yaw = 0;
//sfr WDTCN    = 0xFF;

/*********************************************
* name:	 		mc_init_watchdog
* description:	初始化配置看门狗
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_watchdog(void)
{
	PCA0MD &= ~0x40; /* 关闭看门狗 */
  	return;
}

/*********************************************
* name:	 		mc_init_sysclk
* description:	初始化配置系统时钟
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sysclk(void)
{
    OSCICN |= 0x03; /* SYSCLK为内部高频振荡器输出（不分频）,SYSCLK == 12MHz */ 
//	RSTSRC = 0x04; /* 检测到时钟丢失条件时触发复位 */  
	             
	return;
}

/*********************************************
* name:	 		mc_init_port
* description:	初始化配置GPIO
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_port(void)
{
	/* uart0 prot init */
	P0MDOUT |= 0x10; /* P0.4为推挽输出 */
    XBR0 |= 0x01; /* 端口I/O交叉开关寄存器0,UART TX0, RX0 连到端口引脚 P0.4 和 P0.5 */                                          
    XBR1 |= 0x40; /* 端口I/O交叉开关寄存器1,交叉开关使能 */
//	P0MDIN |= 0x30; /* P0.4和P0.5配置为数字I/O,默认就是如此 */

	/* SMBus port init */
//	XBR0 |= 0x04; /* 交叉开关，SMBus I/O 连接到端口引脚 */
//  XBR1 |= 0x40; /* 端口I/O交叉开关寄存器1,交叉开关使能 */
//	P0MDIN |= 0x03; /* P0.0和P0.1配置为数字I/O,默认就是如此 */
//  P0MDOUT &= ~0x03; /* P0.0和P0.1配置为漏极开路,默认就是如此 */ 

	/* P2.2 led灯 init */
//	P2MDOUT |= 0x04;
	           
	return;
}

/*********************************************
* name:	 		mc_init_uart0
* description:	初始化配置uart0（初始化timer1）
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_uart0(void)
{
 	/* uart0的波特率又定时器1提供 */
	  
	SCON0 |= 0x10; /* UART0工作方式为0（波特率可编程的8位UART），UART0接收允许 */ 
//	CKCON = 0x01; /* 定时器1的分频位，设置为系统时钟/4分频 */

   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
      CKCON |=  0x08;
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   }
   TL1 = TH1;

	/* UART0的波特率又定时器1工作在8位自动重载方式产生 */
	TMOD &= ~0xf0; 	/* 定时器1方式选择为8位自动重载方式 */                        
 	TMOD = 0x20; 	/* 定时器1方式选择为8位自动重载方式 */ 
	TR1 = 1; 		/* 启动定时器1 TR1为寄存器TCON中的第6位*/
    TI0 = 1;        /* Indicate TX0 ready */	
	//IP |= 0x10; 	/* 置定时器1中断为高优先级 */
	//ES0 = 1;		/* 定时器1中断允许 */

	return;
}

/*********************************************
* name:	 		mc_init_timer2
* description:	16位自动重载，用于每30ms向串口发送四元数
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer2()
{
	TMR2CN &= ~0x01;  /* T2XCLK = 0; 选择定时器 2 外部时钟为系统时钟/12 (SYSCLK = 12Mhz) */
 
 	/* T2ML = 0; 定时器2 低字节使用TMR2CN中的T2XCLK 位定义的时钟 */
	/* T2MH = 0; 定时器2 高字节使用TMR2CN中的T2XCLK 位定义的时钟 */
	CKCON &= ~0x30;

	/* TMR2RL = 55535; 10000 = (2^16 - X)*(12Mhz/12)  X = 55535 */
	TMR2RLH = 55535/256; /* 定时器T2的重载寄存器高八位赋初值 */
	TMR2RLL = 55535%256; /* 定时器T2的重载寄存器低八位赋初值 */
	TMR2H   = TMR2RLH;
	TMR2L   = TMR2RLL;

	TMR2CN |= 0x04;   /* 使能timer2 为16位自动重载定时器 */
    ET2 = 1;		 /* 允许timer2中断 */
	
	return;
}

/*********************************************
* name:	 		mc_init_timer3
* description:	16位自动重载，用于每1ms从IIC总线读取传感器的数据
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer3()
{
	TMR3CN &= ~0x01; /* T3XCLK = 0; 选择定时器 3 外部时钟为系统时钟/12 (SYSCLK = 12Mhz) */

 	/* T3ML = 0; 定时器3 低字节使用TMR3CN中的T3XCLK 位定义的时钟 */
	/* T3MH = 0; 定时器3 高字节使用TMR3CN中的T3XCLK 位定义的时钟 */
	CKCON &= ~0xC0;

	/* TMR3RL = 64535; 10000 = (2^16 - X)*(12Mhz/12)  X = 64535 */
	TMR3RLH = 64535/256; /* 定时器T3的重载寄存器高八位赋初值 */
	TMR3RLL = 64535%256; /* 定时器T3的重载寄存器低八位赋初值 */
	TMR3H   = TMR3RLH;
	TMR3L   = TMR3RLL;

	TMR3CN |= 0x04;   /* 使能timer3 为16位自动重载定时器 */
    
	//EIE1 |= 0x80; /* ET3 = 1; 允许timer3中断 */
	
	return;
}

/*********************************************
* name:	 		mc_uart0_interrupt
* description:	uart0中断处理函数，每10ms发送一次欧拉角,发送至少需要4ms
* input:		NONE
* return:		NONE
*********************************************/
void mc_uart0_ISR (void) interrupt 5
{
	uchar i = 0;
	uchar count = 0;
	uchar send[12] = {0};  	/* 用于转换float型四元数为ascii，顺序是q0,q1,q2,q3 */

	float_to_ascii(Yaw, send);  			/* Yaw --> send[0]--send[3] */
	float_to_ascii(Pitch, &(send[4]));  	/* Pitch --> send[4]--send[7] */
	float_to_ascii(Roll, &(send[8]));  	/* Roll --> send[8]--send[11] */

	//printf("q0=%f, q1=%f, q2=%f, q3=%f\n", q0, q1, q2, q3);	
	SBUF0 = 0xAA;  /* 两字节帧头 */
	delay_us(250);
	SBUF0 = 0x55;
	delay_us(250);
	for (i = 0; i < 12; i++)
	{
		SBUF0 = send[i];
		count++;
		delay_us(250);
	}
	SBUF0 = count;  /* 校验位，有效数据位的长度 0x0C*/
	delay_us(250);
	//printf("\n");

	TF2H = 0;  /* 重启中断 */

   return;
}

/*********************************************
* name:	 		mc_i2c_interrupt
* description:	i2c中断处理函数，每隔1ms读一次传感器数据
* input:		NONE
* return:		NONE
*********************************************/
void mc_i2c_interrupt (void) interrupt 6
{
   return;
}

/*********************************************
* name:	 		delay_5us
* description:	5us延时
* input:		NONE
* return:		NONE
*********************************************/
void delay_5us(void)
{
    _nop_();_nop_();_nop_();_nop_();
    _nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();

	/*
    _nop_();_nop_();_nop_();_nop_();
    _nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
    _nop_();_nop_();_nop_();_nop_();
    _nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
	*/

	return;
}

/*********************************************
* name:	 		delay_5ms
* description:	5ms延时
* input:		NONE
* return:		NONE
*********************************************/
void delay_5ms(void)
{
    unsigned short n = 1698;

    while (n--);

	return;
}

/*********************************************
* name:	 		delay_ms
* description:	5us 的倍数
* input:		需要延时的us数，必须是5的倍数
* return:		NONE
*********************************************/
void delay_us(uchar n)
{
	uchar i = 0;
	for (i = 0; i < (n / 5); i++)
	{
		delay_5us();
	}

	return;
}

/*********************************************
* name:	 		delay_ms
* description:	5ms 的倍数
* input:		需要延时的ms数，必须是5的倍数
* return:		NONE
*********************************************/
void delay_ms(uchar n)
{
	uchar i = 0;
	for (i = 0; i < (n / 5); i++)
	{
		delay_5ms();
	}

	return;
}

/**************************************
起始信号
**************************************/
void mc_smbus_start()
{
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    delay_5us();                 //延时
    SDA = 0;                    //产生下降沿
    delay_5us();                 //延时
    SCL = 0;                    //拉低时钟线
}

/**************************************
停止信号
**************************************/
void mc_smbus_stop()
{
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    delay_5us();                 //延时
    SDA = 1;                    //产生上升沿
    delay_5us();                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void mc_smbus_sendack(bit ack)
{
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    delay_5us();                 //延时
    SCL = 0;                    //拉低时钟线
    delay_5us();                 //延时
}

/**************************************
接收应答信号
**************************************/
bit mc_smbus_recvack()
{
    SCL = 1;                    //拉高时钟线
    delay_5us();                 //延时
    CY = SDA;                   //读应答信号
    SCL = 0;                    //拉低时钟线
    delay_5us();                 //延时

    return CY;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
void mc_smbus_sendbyte(BYTE dat)
{
    BYTE i;

    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        delay_5us();             //延时
        SCL = 0;                //拉低时钟线
        delay_5us();             //延时
    }
    mc_smbus_recvack();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE mc_smbus_recvbyte()
{
    BYTE i;
    BYTE dat = 0;

    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        delay_5us();             //延时
        dat |= SDA;             //读数据               
        SCL = 0;                //拉低时钟线
        delay_5us();             //延时
    }
    return dat;
}

//******单字节写入*******************************************

void single_write_smbus(uchar SlaveAddress, uchar REG_Address, uchar REG_data)
{
    mc_smbus_start();                  		//起始信号
    mc_smbus_sendbyte(SlaveAddress + WRITE);//发送设备地址+写信号
    mc_smbus_sendbyte(REG_Address);    		//内部寄存器地址，请参考中文pdf22页 
    mc_smbus_sendbyte(REG_data);       		//内部寄存器数据，请参考中文pdf22页 
    mc_smbus_stop();                   		//发送停止信号
}

#if 0
//********单字节读取*****************************************
uchar single_read_smbus(uchar SlaveAddress, uchar REG_Address)
{  uchar REG_data;
    mc_smbus_start();                          //起始信号
    mc_smbus_sendbyte(SlaveAddress + WRITE);   //发送设备地址+写信号
    mc_smbus_sendbyte(REG_Address);            //发送存储单元地址，从0开始	
    mc_smbus_start();                          //起始信号
    mc_smbus_sendbyte(SlaveAddress + READ);    //发送设备地址+读信号
    REG_data=mc_smbus_recvbyte();              //读出寄存器数据
	mc_smbus_sendack(1);   
	mc_smbus_stop();                           //停止信号
    return REG_data; 
}
#endif
//*********************************************************
//
//连续读出传感器内部加速度数据，ADXL345地址范围0x32~0x37
//
//*********************************************************
void multiple_read_smbus(uchar SlaveAddress, uchar REG_Address, uchar *buf)
{   
	uchar i;
    mc_smbus_start();                          //起始信号
    mc_smbus_sendbyte(SlaveAddress + WRITE);   //发送设备地址+写信号
    mc_smbus_sendbyte(REG_Address);            //发送存储单元地址，从REG_Address开始	
    mc_smbus_start();                          //起始信号
    mc_smbus_sendbyte(SlaveAddress + READ);    //发送设备地址+读信号
	 for (i=0; i<6; i++)                       //连续读取6个地址数据，存储中BUF
    {
        buf[i] = mc_smbus_recvbyte();          //BUF[0]存储REG_Address地址中的数据
        if (i == 5)
        {
           mc_smbus_sendack(1);                //最后一个数据需要回NOACK
        }
        else
        {
          mc_smbus_sendack(0);                //回应ACK
       }
   }
    mc_smbus_stop();                          //停止信号
    delay_ms(15);
}

/*********************************************
* name:	 		float_to_ascii
* description:	分别取出每一位数，用于串口输出，串口输出是ASCII形式
* input:		NONE
* return:		NONE
*********************************************/
void float_to_ascii(float input, uchar *s)
{
	 uchar *p;

	 p = (uchar *)&input;

    *s = *p;

    *(s + 1) = *(p + 1);

    *(s + 2) = *(p + 2);

    *(s + 3) = *(p + 3);

	return;
}

