/* ��ʼ�����к�8051�йص�Ӳ����Դ������I/O���ã����Ź������񣬴��ڣ�IIC�� */

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_init_8051.h"
#include "mc_algorithm.h"

sbit	  SCL=P0^0;      //IICʱ�����Ŷ���
sbit 	  SDA=P0^1;      //IIC�������Ŷ���
float Pitch = 0, Roll = 0, Yaw = 0;
//sfr WDTCN    = 0xFF;

/*********************************************
* name:	 		mc_init_watchdog
* description:	��ʼ�����ÿ��Ź�
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_watchdog(void)
{
	PCA0MD &= ~0x40; /* �رտ��Ź� */
  	return;
}

/*********************************************
* name:	 		mc_init_sysclk
* description:	��ʼ������ϵͳʱ��
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sysclk(void)
{
    OSCICN |= 0x03; /* SYSCLKΪ�ڲ���Ƶ�������������Ƶ��,SYSCLK == 12MHz */ 
//	RSTSRC = 0x04; /* ��⵽ʱ�Ӷ�ʧ����ʱ������λ */  
	             
	return;
}

/*********************************************
* name:	 		mc_init_port
* description:	��ʼ������GPIO
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_port(void)
{
	/* uart0 prot init */
	P0MDOUT |= 0x10; /* P0.4Ϊ������� */
    XBR0 |= 0x01; /* �˿�I/O���濪�ؼĴ���0,UART TX0, RX0 �����˿����� P0.4 �� P0.5 */                                          
    XBR1 |= 0x40; /* �˿�I/O���濪�ؼĴ���1,���濪��ʹ�� */
//	P0MDIN |= 0x30; /* P0.4��P0.5����Ϊ����I/O,Ĭ�Ͼ������ */

	/* SMBus port init */
//	XBR0 |= 0x04; /* ���濪�أ�SMBus I/O ���ӵ��˿����� */
//  XBR1 |= 0x40; /* �˿�I/O���濪�ؼĴ���1,���濪��ʹ�� */
//	P0MDIN |= 0x03; /* P0.0��P0.1����Ϊ����I/O,Ĭ�Ͼ������ */
//  P0MDOUT &= ~0x03; /* P0.0��P0.1����Ϊ©����·,Ĭ�Ͼ������ */ 

	/* P2.2 led�� init */
//	P2MDOUT |= 0x04;
	           
	return;
}

/*********************************************
* name:	 		mc_init_uart0
* description:	��ʼ������uart0����ʼ��timer1��
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_uart0(void)
{
 	/* uart0�Ĳ������ֶ�ʱ��1�ṩ */
	  
	SCON0 |= 0x10; /* UART0������ʽΪ0�������ʿɱ�̵�8λUART����UART0�������� */ 
//	CKCON = 0x01; /* ��ʱ��1�ķ�Ƶλ������Ϊϵͳʱ��/4��Ƶ */

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

	/* UART0�Ĳ������ֶ�ʱ��1������8λ�Զ����ط�ʽ���� */
	TMOD &= ~0xf0; 	/* ��ʱ��1��ʽѡ��Ϊ8λ�Զ����ط�ʽ */                        
 	TMOD = 0x20; 	/* ��ʱ��1��ʽѡ��Ϊ8λ�Զ����ط�ʽ */ 
	TR1 = 1; 		/* ������ʱ��1 TR1Ϊ�Ĵ���TCON�еĵ�6λ*/
    TI0 = 1;        /* Indicate TX0 ready */	
	//IP |= 0x10; 	/* �ö�ʱ��1�ж�Ϊ�����ȼ� */
	//ES0 = 1;		/* ��ʱ��1�ж����� */

	return;
}

/*********************************************
* name:	 		mc_init_timer2
* description:	16λ�Զ����أ�����ÿ30ms�򴮿ڷ�����Ԫ��
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer2()
{
	TMR2CN &= ~0x01;  /* T2XCLK = 0; ѡ��ʱ�� 2 �ⲿʱ��Ϊϵͳʱ��/12 (SYSCLK = 12Mhz) */
 
 	/* T2ML = 0; ��ʱ��2 ���ֽ�ʹ��TMR2CN�е�T2XCLK λ�����ʱ�� */
	/* T2MH = 0; ��ʱ��2 ���ֽ�ʹ��TMR2CN�е�T2XCLK λ�����ʱ�� */
	CKCON &= ~0x30;

	/* TMR2RL = 55535; 10000 = (2^16 - X)*(12Mhz/12)  X = 55535 */
	TMR2RLH = 55535/256; /* ��ʱ��T2�����ؼĴ����߰�λ����ֵ */
	TMR2RLL = 55535%256; /* ��ʱ��T2�����ؼĴ����Ͱ�λ����ֵ */
	TMR2H   = TMR2RLH;
	TMR2L   = TMR2RLL;

	TMR2CN |= 0x04;   /* ʹ��timer2 Ϊ16λ�Զ����ض�ʱ�� */
    ET2 = 1;		 /* ����timer2�ж� */
	
	return;
}

/*********************************************
* name:	 		mc_init_timer3
* description:	16λ�Զ����أ�����ÿ1ms��IIC���߶�ȡ������������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_timer3()
{
	TMR3CN &= ~0x01; /* T3XCLK = 0; ѡ��ʱ�� 3 �ⲿʱ��Ϊϵͳʱ��/12 (SYSCLK = 12Mhz) */

 	/* T3ML = 0; ��ʱ��3 ���ֽ�ʹ��TMR3CN�е�T3XCLK λ�����ʱ�� */
	/* T3MH = 0; ��ʱ��3 ���ֽ�ʹ��TMR3CN�е�T3XCLK λ�����ʱ�� */
	CKCON &= ~0xC0;

	/* TMR3RL = 64535; 10000 = (2^16 - X)*(12Mhz/12)  X = 64535 */
	TMR3RLH = 64535/256; /* ��ʱ��T3�����ؼĴ����߰�λ����ֵ */
	TMR3RLL = 64535%256; /* ��ʱ��T3�����ؼĴ����Ͱ�λ����ֵ */
	TMR3H   = TMR3RLH;
	TMR3L   = TMR3RLL;

	TMR3CN |= 0x04;   /* ʹ��timer3 Ϊ16λ�Զ����ض�ʱ�� */
    
	//EIE1 |= 0x80; /* ET3 = 1; ����timer3�ж� */
	
	return;
}

/*********************************************
* name:	 		mc_uart0_interrupt
* description:	uart0�жϴ�������ÿ10ms����һ��ŷ����,����������Ҫ4ms
* input:		NONE
* return:		NONE
*********************************************/
void mc_uart0_ISR (void) interrupt 5
{
	uchar i = 0;
	uchar count = 0;
	uchar send[12] = {0};  	/* ����ת��float����Ԫ��Ϊascii��˳����q0,q1,q2,q3 */

	float_to_ascii(Yaw, send);  			/* Yaw --> send[0]--send[3] */
	float_to_ascii(Pitch, &(send[4]));  	/* Pitch --> send[4]--send[7] */
	float_to_ascii(Roll, &(send[8]));  	/* Roll --> send[8]--send[11] */

	//printf("q0=%f, q1=%f, q2=%f, q3=%f\n", q0, q1, q2, q3);	
	SBUF0 = 0xAA;  /* ���ֽ�֡ͷ */
	delay_us(250);
	SBUF0 = 0x55;
	delay_us(250);
	for (i = 0; i < 12; i++)
	{
		SBUF0 = send[i];
		count++;
		delay_us(250);
	}
	SBUF0 = count;  /* У��λ����Ч����λ�ĳ��� 0x0C*/
	delay_us(250);
	//printf("\n");

	TF2H = 0;  /* �����ж� */

   return;
}

/*********************************************
* name:	 		mc_i2c_interrupt
* description:	i2c�жϴ�������ÿ��1ms��һ�δ���������
* input:		NONE
* return:		NONE
*********************************************/
void mc_i2c_interrupt (void) interrupt 6
{
   return;
}

/*********************************************
* name:	 		delay_5us
* description:	5us��ʱ
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
* description:	5ms��ʱ
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
* description:	5us �ı���
* input:		��Ҫ��ʱ��us����������5�ı���
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
* description:	5ms �ı���
* input:		��Ҫ��ʱ��ms����������5�ı���
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
��ʼ�ź�
**************************************/
void mc_smbus_start()
{
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    delay_5us();                 //��ʱ
    SDA = 0;                    //�����½���
    delay_5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
}

/**************************************
ֹͣ�ź�
**************************************/
void mc_smbus_stop()
{
    SDA = 0;                    //����������
    SCL = 1;                    //����ʱ����
    delay_5us();                 //��ʱ
    SDA = 1;                    //����������
    delay_5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void mc_smbus_sendack(bit ack)
{
    SDA = ack;                  //дӦ���ź�
    SCL = 1;                    //����ʱ����
    delay_5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
    delay_5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
bit mc_smbus_recvack()
{
    SCL = 1;                    //����ʱ����
    delay_5us();                 //��ʱ
    CY = SDA;                   //��Ӧ���ź�
    SCL = 0;                    //����ʱ����
    delay_5us();                 //��ʱ

    return CY;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void mc_smbus_sendbyte(BYTE dat)
{
    BYTE i;

    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;              //�Ƴ����ݵ����λ
        SDA = CY;               //�����ݿ�
        SCL = 1;                //����ʱ����
        delay_5us();             //��ʱ
        SCL = 0;                //����ʱ����
        delay_5us();             //��ʱ
    }
    mc_smbus_recvack();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE mc_smbus_recvbyte()
{
    BYTE i;
    BYTE dat = 0;

    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        delay_5us();             //��ʱ
        dat |= SDA;             //������               
        SCL = 0;                //����ʱ����
        delay_5us();             //��ʱ
    }
    return dat;
}

//******���ֽ�д��*******************************************

void single_write_smbus(uchar SlaveAddress, uchar REG_Address, uchar REG_data)
{
    mc_smbus_start();                  		//��ʼ�ź�
    mc_smbus_sendbyte(SlaveAddress + WRITE);//�����豸��ַ+д�ź�
    mc_smbus_sendbyte(REG_Address);    		//�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
    mc_smbus_sendbyte(REG_data);       		//�ڲ��Ĵ������ݣ���ο�����pdf22ҳ 
    mc_smbus_stop();                   		//����ֹͣ�ź�
}

#if 0
//********���ֽڶ�ȡ*****************************************
uchar single_read_smbus(uchar SlaveAddress, uchar REG_Address)
{  uchar REG_data;
    mc_smbus_start();                          //��ʼ�ź�
    mc_smbus_sendbyte(SlaveAddress + WRITE);   //�����豸��ַ+д�ź�
    mc_smbus_sendbyte(REG_Address);            //���ʹ洢��Ԫ��ַ����0��ʼ	
    mc_smbus_start();                          //��ʼ�ź�
    mc_smbus_sendbyte(SlaveAddress + READ);    //�����豸��ַ+���ź�
    REG_data=mc_smbus_recvbyte();              //�����Ĵ�������
	mc_smbus_sendack(1);   
	mc_smbus_stop();                           //ֹͣ�ź�
    return REG_data; 
}
#endif
//*********************************************************
//
//���������������ڲ����ٶ����ݣ�ADXL345��ַ��Χ0x32~0x37
//
//*********************************************************
void multiple_read_smbus(uchar SlaveAddress, uchar REG_Address, uchar *buf)
{   
	uchar i;
    mc_smbus_start();                          //��ʼ�ź�
    mc_smbus_sendbyte(SlaveAddress + WRITE);   //�����豸��ַ+д�ź�
    mc_smbus_sendbyte(REG_Address);            //���ʹ洢��Ԫ��ַ����REG_Address��ʼ	
    mc_smbus_start();                          //��ʼ�ź�
    mc_smbus_sendbyte(SlaveAddress + READ);    //�����豸��ַ+���ź�
	 for (i=0; i<6; i++)                       //������ȡ6����ַ���ݣ��洢��BUF
    {
        buf[i] = mc_smbus_recvbyte();          //BUF[0]�洢REG_Address��ַ�е�����
        if (i == 5)
        {
           mc_smbus_sendack(1);                //���һ��������Ҫ��NOACK
        }
        else
        {
          mc_smbus_sendack(0);                //��ӦACK
       }
   }
    mc_smbus_stop();                          //ֹͣ�ź�
    delay_ms(15);
}

/*********************************************
* name:	 		float_to_ascii
* description:	�ֱ�ȡ��ÿһλ�������ڴ�����������������ASCII��ʽ
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

