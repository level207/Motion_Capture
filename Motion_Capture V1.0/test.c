#include <stdio.h>
#include "test.h"

uchar ge,shi,bai,qian,wan,shiwan;           //显示变量
uchar data_write[18] = 0;
int dis_data = 0;
uchar dis[4];                         //显示数组

void a_conversion(long temp_data)  
{     
    shiwan=temp_data/100000+0x30 ;
    temp_data=temp_data%100000;   //取余运算 
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //取余运算
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //取余运算
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //取余运算
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //取余运算
    ge=temp_data+0x30; 	
}

//显示x轴
void a_display_x()
{   
    float temp;
    dis_data=(data_write[1]<<8)+data_write[0];  //合成数据   
	DisplayOneChar('X');   //第0行，第0列 显示X
    DisplayOneChar(':'); 

	if(dis_data<0){
	dis_data=-dis_data;
    DisplayOneChar('-');      //显示正负符号位
	}
	else DisplayOneChar('+'); //显示空格

    temp=(float)dis_data*3.9;  //计算数据和显示,查考ADXL345快速入门第4页
    printf("%5.4f ", temp/1000);
	
	/*
	a_conversion(temp);          //转换出显示需要的数据
    DisplayOneChar(qian); 
	DisplayOneChar('.'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi); 
	DisplayOneChar(' ');
	*/

	return;
}

//***********************************************************************
//显示y轴
void a_display_y()
{     
    float temp;
    dis_data=(data_write[3]<<8)+data_write[2];  //合成数据 
	DisplayOneChar('Y');   //第1行，第0列 显示y
    DisplayOneChar(':'); 
	  
	if(dis_data<0){
	dis_data=-dis_data;
    DisplayOneChar('-');      //显示正负符号位
	}
	else DisplayOneChar('+'); //显示空格

    temp=(float)dis_data*3.9;  //计算数据和显示,查考ADXL345快速入门第4页
	printf("%5.4f ", temp/1000);
	/*    
	a_conversion(temp);          //转换出显示需要的数据

    DisplayOneChar(qian); 
	DisplayOneChar('.'); 
    DisplayOneChar(bai); 
    DisplayOneChar(shi);  
	DisplayOneChar(' ');
	*/
	return;
}

//***********************************************************************
//显示z轴
void a_display_z()
{     
	float temp;
    dis_data=(data_write[5]<<8)+data_write[4];    //合成数据 
	DisplayOneChar('Z');  //第0行，第10列 显示Z
    DisplayOneChar(':'); 	
	  
	if(dis_data<0){
	dis_data=-dis_data;
    DisplayOneChar('-');       //显示负符号位
	}
	else DisplayOneChar('+');  //显示空格

    temp=(float)dis_data*3.9;  //计算数据和显示,查考ADXL345快速入门第4页
	printf("%5.4f ", temp/1000);

	/*
	a_conversion(temp);          //转换出显示需要的数据
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
    temp_data=temp_data%100;     //取余运算
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //取余运算
    *++s =temp_data+0x30; 	
}

//***********************************************************************
//显示x轴
void i_display_x()
{  
    dis_data=(data_write[0]<<8) | data_write[1];   //合成数据   
    dis_data/=14.375;              //计算对应 度/秒
    i_conversion(dis, dis_data);   //转换数据显示
	DisplayOneChar('X');   //第0行，第0列 显示X
    DisplayOneChar(':');
	printf("%d ", dis_data);
    //DisplayListChar(dis,4);    //启始列，行，显示数组，显示长度
	//DisplayOneChar(' ');

	return;
}

//***********************************************************************
//显示y轴
void i_display_y()
{    
    dis_data=(data_write[2]<<8) | data_write[3];   //合成数据   
    dis_data/=14.375;              //计算对应 度/秒
    i_conversion(dis, dis_data);     //转换数据显示
	DisplayOneChar('Y');   //第0行，第0列 显示X
    DisplayOneChar(':');
	printf("%d ", dis_data);
    //DisplayListChar(dis,4);    //启始列，行，显示数组，显示位数
	//DisplayOneChar(' ');

	return;

}

//***********************************************************************
//显示z轴
void i_display_z()
{ 
    dis_data=(data_write[4]<<8) | data_write[5];     //合成数据   
    dis_data/=14.375;                //计算对应 度/秒
    i_conversion(dis, dis_data);       //转换数据显示
	DisplayOneChar('Z');   //第0行，第0列 显示X
    DisplayOneChar(':');
	printf("%d ", dis_data);
    //DisplayListChar(dis,4);     //启始列，行，显示数组，显示位数
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
    temp_data=temp_data%10000;   //取余运算
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //取余运算
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //取余运算
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //取余运算
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
    h_conversion(angle);       //计算数据和显示
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

    h_conversion(dis_data);       //计算数据和显示
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

    h_conversion(dis_data);       //计算数据和显示
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

    h_conversion(dis_data);       //计算数据和显示
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
* description:	从串口输出一个ASCII值
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