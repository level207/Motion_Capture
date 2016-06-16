#include <stdio.h>
#include "test.h"

uchar ge,shi,bai,qian,wan,shiwan;           //��ʾ����
uchar data_write[18] = 0;
int dis_data = 0;
uchar dis[4];                         //��ʾ����

void a_conversion(long temp_data)  
{     
    shiwan=temp_data/100000+0x30 ;
    temp_data=temp_data%100000;   //ȡ������ 
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //ȡ������
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //ȡ������
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //ȡ������
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //ȡ������
    ge=temp_data+0x30; 	
}

//��ʾx��
void a_display_x()
{   
    float temp;
    dis_data=(data_write[1]<<8)+data_write[0];  //�ϳ�����   
	DisplayOneChar('X');   //��0�У���0�� ��ʾX
    DisplayOneChar(':'); 

	if(dis_data<0){
	dis_data=-dis_data;
    DisplayOneChar('-');      //��ʾ��������λ
	}
	else DisplayOneChar('+'); //��ʾ�ո�

    temp=(float)dis_data*3.9;  //�������ݺ���ʾ,�鿼ADXL345�������ŵ�4ҳ
    printf("%5.4f ", temp/1000);
	
	/*
	a_conversion(temp);          //ת������ʾ��Ҫ������
    DisplayOneChar(qian); 
	DisplayOneChar('.'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi); 
	DisplayOneChar(' ');
	*/

	return;
}

//***********************************************************************
//��ʾy��
void a_display_y()
{     
    float temp;
    dis_data=(data_write[3]<<8)+data_write[2];  //�ϳ����� 
	DisplayOneChar('Y');   //��1�У���0�� ��ʾy
    DisplayOneChar(':'); 
	  
	if(dis_data<0){
	dis_data=-dis_data;
    DisplayOneChar('-');      //��ʾ��������λ
	}
	else DisplayOneChar('+'); //��ʾ�ո�

    temp=(float)dis_data*3.9;  //�������ݺ���ʾ,�鿼ADXL345�������ŵ�4ҳ
	printf("%5.4f ", temp/1000);
	/*    
	a_conversion(temp);          //ת������ʾ��Ҫ������

    DisplayOneChar(qian); 
	DisplayOneChar('.'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi);  
	DisplayOneChar(' ');
	*/
	return;
}

//***********************************************************************
//��ʾz��
void a_display_z()
{     
	float temp;
    dis_data=(data_write[5]<<8)+data_write[4];    //�ϳ����� 
	DisplayOneChar('Z');  //��0�У���10�� ��ʾZ
    DisplayOneChar(':'); 	
	  
	if(dis_data<0){
	dis_data=-dis_data;
    DisplayOneChar('-');       //��ʾ������λ
	}
	else DisplayOneChar('+');  //��ʾ�ո�

    temp=(float)dis_data*3.9;  //�������ݺ���ʾ,�鿼ADXL345�������ŵ�4ҳ
	printf("%5.4f ", temp/1000);

	/*
	a_conversion(temp);          //ת������ʾ��Ҫ������
    DisplayOneChar(qian); 
	DisplayOneChar('.'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi); 
	DisplayOneChar(' ');
	*/
	return;
}

void i_conversion(uchar *s,int temp_data)
{
	if(temp_data<0){
	temp_data=-temp_data;
    *s='-';
	}
	else *s='+';
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //ȡ������
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //ȡ������
    *++s =temp_data+0x30; 	
}

//***********************************************************************
//��ʾx��
void i_display_x()
{  
    dis_data=(data_write[0]<<8) | data_write[1];   //�ϳ�����   
    dis_data/=14.375;              //�����Ӧ ��/��
    i_conversion(dis, dis_data);   //ת��������ʾ
	DisplayOneChar('X');   //��0�У���0�� ��ʾX
    DisplayOneChar(':');
	printf("%d ", dis_data);
    //DisplayListChar(dis,4);    //��ʼ�У��У���ʾ���飬��ʾ����
	//DisplayOneChar(' ');

	return;
}

//***********************************************************************
//��ʾy��
void i_display_y()
{    
    dis_data=(data_write[2]<<8) | data_write[3];   //�ϳ�����   
    dis_data/=14.375;              //�����Ӧ ��/��
    i_conversion(dis, dis_data);     //ת��������ʾ
	DisplayOneChar('Y');   //��0�У���0�� ��ʾX
    DisplayOneChar(':');
	printf("%d ", dis_data);
    //DisplayListChar(dis,4);    //��ʼ�У��У���ʾ���飬��ʾλ��
	//DisplayOneChar(' ');

	return;

}

//***********************************************************************
//��ʾz��
void i_display_z()
{ 
    dis_data=(data_write[4]<<8) | data_write[5];     //�ϳ�����   
    dis_data/=14.375;                //�����Ӧ ��/��
    i_conversion(dis, dis_data);       //ת��������ʾ
	DisplayOneChar('Z');   //��0�У���0�� ��ʾX
    DisplayOneChar(':');
	printf("%d ", dis_data);
    //DisplayListChar(dis,4);     //��ʼ�У��У���ʾ���飬��ʾλ��
	//DisplayOneChar(' ');

	return;
}

/***********************************/
void DisplayListChar(uchar *DData,L)
{
	uchar ListLength=0; 
	while(L--)             
	{                       
		DisplayOneChar(DData[ListLength]);
		ListLength++;  
	}
}

void h_conversion(uint temp_data)  
{  
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //ȡ������
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //ȡ������
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //ȡ������
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //ȡ������
    ge=temp_data+0x30; 	
}

void h_display_dierection()
{
	int x, y, z;
	double angle = 0;

    x=data_write[0] << 8 | data_write[1]; //Combine MSB and LSB of X Data output register
    z=data_write[2] << 8 | data_write[3]; //Combine MSB and LSB of Z Data output register
    y=data_write[4] << 8 | data_write[5]; //Combine MSB and LSB of Y Data output register

    angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
    angle*=10;
    h_conversion(angle);       //�������ݺ���ʾ
	DisplayOneChar('A'); 
    DisplayOneChar(':'); 
    DisplayOneChar(qian); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi); 
    DisplayOneChar('.'); 
	DisplayOneChar(ge);
	DisplayOneChar(' '); 

	return;
}

void h_display_x()
{

    dis_data=data_write[0] << 8 | data_write[1]; //Combine MSB and LSB of X Data output register

    h_conversion(dis_data);       //�������ݺ���ʾ
	DisplayOneChar('X'); 
    DisplayOneChar(':'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi); 
	DisplayOneChar(ge);
	DisplayOneChar(' '); 

	return;
}

void h_display_y()
{

    dis_data=data_write[2] << 8 | data_write[3]; //Combine MSB and LSB of X Data output register

    h_conversion(dis_data);       //�������ݺ���ʾ
	DisplayOneChar('Y'); 
    DisplayOneChar(':'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi); 
	DisplayOneChar(ge);
	DisplayOneChar(' '); 

	return;
}

void h_display_z()
{

    dis_data=data_write[4] << 8 | data_write[5]; //Combine MSB and LSB of X Data output register

    h_conversion(dis_data);       //�������ݺ���ʾ
	DisplayOneChar('Z'); 
    DisplayOneChar(':'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi); 
	DisplayOneChar(ge);
	DisplayOneChar(' '); 

	return;
}

/*********************************************
* name:	 		DisplayOneChar
* description:	�Ӵ������һ��ASCIIֵ
* input:		NONE
* return:		NONE
*********************************************/
void DisplayOneChar(uchar out)
{
	SBUF0 = out;
	delay_5ms();
	delay_5ms(); 

	return;
}