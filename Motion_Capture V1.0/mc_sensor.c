/* ���ڿ��ƴ����������ĺ��� */

#include <stdio.h>
#include <intrins.h>
#include <c8051f340.h>
#include "mc_sensor.h"

/*********************************************
* name:	 		mc_init_sensor
* description:	��ʼ��������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_sensor(void)
{
	mc_init_ADXL345(); 		//�����߼��ٶȴ�����
	mc_init_ITG3205();		//���������ǣ����ٶȴ�������
	mc_init_HMC5883L();		//����������̣��ų���������

	return;
}

/*********************************************
* name:	 		mc_init_ADXL345
* description:	��ʼ�������߼��ٶȴ�����
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ADXL345(void)
{
	
	//single_write_smbus(0xA6, 0x31, 0x0B);   //������Χ,����16g��13λģʽ
	/* ���ò���Ƶ��Ϊ 1600Hz ��1.6 sample per ms */
   	//single_write_smbus(0xA6, 0x2C, 0x0E);   //�����趨Ϊ1600Hz �ο�pdf13ҳ  /* Table 7��Table 8�������� */
   	//single_write_smbus(0xA6, 0x2D, 0x08);   //ѡ���Դģʽ   �ο�pdf24ҳ
   	//single_write_smbus(0xA6, 0x2E, 0x80);   //ʹ�� DATA_READY �ж�
   	//single_write_smbus(0xA6, 0x1E, 0x00);   //X ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
   	//single_write_smbus(0xA6, 0x1F, 0x00);   //Y ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
   	//single_write_smbus(0xA6, 0x20, 0x05);   //Z ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
	
	single_write_smbus(0xA6, 0x31, 0x0B);   //������Χ,����16g��13λģʽ
   	single_write_smbus(0xA6, 0x2C, 0x08);   //�����趨Ϊ12.5 �ο�pdf13ҳ
   	single_write_smbus(0xA6, 0x2D, 0x08);   //ѡ���Դģʽ   �ο�pdf24ҳ
   	single_write_smbus(0xA6, 0x2E, 0x80);   //ʹ�� DATA_READY �ж�
   	single_write_smbus(0xA6, 0x1E, 0x00);   //X ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
   	single_write_smbus(0xA6, 0x1F, 0x00);   //Y ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
   	single_write_smbus(0xA6, 0x20, 0x05);   //Z ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ

	return;
}

/*********************************************
* name:	 		mc_init_ITG3205
* description:	��ʼ�����������ǣ��Ƕȴ�������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_ITG3205(void)
{
 	single_write_smbus(0xD0, 0x3E, 0x80);   // 0x80
    /* ���ò���Ƶ��Ϊ F = 1kHz / (0 + 1) = 1000Hz, or 1ms per sample ��ͨ�˲�����Ϊ 5Hz*/
	//single_write_smbus(0xD0, 0x15, 0x07);	//SMPLRT_DIV��0-255�� 
	single_write_smbus(0xD0, 0x15, 0x00);	//SMPLRT_DIV��0-255�� 
	single_write_smbus(0xD0, 0x16, 0x1E);   //DLPF_FS
	single_write_smbus(0xD0, 0x17, 0x00);	//G_INIT_CFG
	single_write_smbus(0xD0, 0x3E, 0x00);   //

	return;
}

/*********************************************
* name:	 		mc_init_HMC5883L
* description:	��ʼ������������̣��ų���������
* input:		NONE
* return:		NONE
*********************************************/
void mc_init_HMC5883L(void)
{
    /*����Ϊ�������ģʽ������б�Ҫ�����ݿ��Դ���������Ĵ������¶�ȡ ���Ƶ��Ϊ 30Hz*/
	single_write_smbus(0x3C, 0x00, 0x14);   //
	single_write_smbus(0x3C, 0x02, 0x00);   //

	return;
}