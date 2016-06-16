/* �����㷨��ʵ�� */
#include <stdio.h>
#include <intrins.h>
#include <math.h>
#include "mc_common_define.h"
#include "mc_sensor.h"
#include "mc_algorithm.h"

float q0 = 0, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

/*******************************************************************************
* Function Name  : init_quaternion
* Description    : �����ʼ����Ԫ��q0 q1 q2 q3.	��ʼ��ֻ��Ҫ���ٶȼƺʹ����Ƶ�ֵ�Ϳ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void)
{ 
	uchar data_write[6];
    signed short int accel[3], mag[3];
    float init_Yaw, init_Pitch, init_Roll;
	float init_ax, init_ay, init_az, init_mx, init_my, init_mz;
    int i = 0;

  	multiple_read_smbus(0xA6, 0x32, data_write); //�����ٶȼƵ�ֵ
	accel[0]=(signed short int)((data_write[1]<<8) | data_write[0]);
	accel[1]=(signed short int)((data_write[3]<<8) | data_write[2]);
	accel[2]=(signed short int)((data_write[5]<<8) | data_write[4]);
	  	    
	//��λת�����������ٶȵĵ�λg��m/s2
	init_ax=(float)(accel[0] * 3.9 / 1000);	   
	init_ay=(float)(accel[1] * 3.9 / 1000);
    init_az=(float)(accel[2] * 3.9 / 1000);
	//printf("ax=%f,   ay=%f,   az=%f", init_ax, init_ay, init_az);

    //����x y���У׼��δ��z�����У׼���ο�MEMSense��У׼���� 
    multiple_read_smbus(0x3C, 0x03, data_write); //���������̵�ֵ
	mag[0]=(signed short int)((data_write[0]<<8) | data_write[1]);
    mag[1]=(signed short int)((data_write[2]<<8) | data_write[3]);
    mag[2]=(signed short int)((data_write[4]<<8) | data_write[5]);
    init_mx =(float)mag[0];						
    init_my =(float)mag[1];
    init_mz =(float)mag[2];

	//������x��Ϊǰ������
    init_Roll  = atan2(init_ay, init_az);
    init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
    init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                        init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);

    q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
    q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��roll
    q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��pitch
    q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z 	 ��z����ת��Yaw
	
	//printf("��ʼ����Ԫ����Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw*57.295780, init_Pitch*57.295780, init_Roll*57.295780);
    //������y��Ϊǰ������    
	//init_Roll = -atan2(init_ax, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.3ת��Ϊ�Ƕ�
	//init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
	//init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
	//                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���
	//if(init_Yaw < 0){init_Yaw = init_Yaw + 2*3.141593;}
	//if(init_Yaw > 360){init_Yaw = init_Yaw - 2*3.141593;}				            
	//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������ZXY˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
	//q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
	//q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
	//q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
	//q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw    
    
	return;
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float  ay, float az, float mx, float my, float mz) 
{
    float norm;
    float hx, hy, hz, bz, bx; //, by
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
   //xdata float Pitch, Roll, Yaw;

/*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
   //auxiliary variables to reduce number of repeated operations��
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
          
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣����Ҷ���byָ������������by=ĳֵ��bx=0
������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
��Ϊbx=0�����Ծͼ򻯳�(by*by)  = ((hx*hx) + (hy*hy))�������by�������޸�by��bxָ����Զ����ĸ���ָ������*/
 
   //���ǵĺ�ԭ���������𣬶����bxΪǰ������Ϊ����
   bx = sqrt((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
��Ϊbx=0�����������漰��bx�Ĳ��ֶ���ʡ���ˡ�ͬ��by=0�����������漰��by�Ĳ���Ҳ���Ա�ʡ�ԣ�������Լ������Ǹ���ָ���йء�
������������vxyz�����㣬��Ϊ����g��az=1��ax=ay=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
����Կ���������ʽ��wxyz�Ĺ�ʽ����by����ay��0������bz����az��1�����ͱ����vxyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/

	 //���Ƕ����bxָ������
   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

   //wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   //wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   //wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
         
//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
   //��������ϵ��,����������º��������ֵ
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //�ܹؼ���һ�仰��ԭ�㷨û��
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //���Բ������ڵ�һ��
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise����Ԫ�������㷨
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
      
   // normalise quaternion
   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 * norm;       //w
   q1 = q1 * norm;       //x
   q2 = q2 * norm;       //y
   q3 = q3 * norm;       //z

   //����Ԫ������ŷ���ǣ�ע�����Ǹ�D3D����ϵ�ã����Ժ���������ϵ�ǲ�ͬ�Ĺ�ʽ��
   //Roll  = arctan2(2(wz + xy), 1 - 2(xx + zz))*57.3;
   //Pitch = arcsin(2(wx - yz))*57.3;
   //Yaw   = arctan2(2(wy + xz), 1- 2(xx + yy))*57.3;
   //1=q0*q0+q1*q1+q2*q2+q3*q3;
   
    Roll  = atan2(2 * (q0 * q3 + q1 *q2), 1 - 2 *(q1 * q1 + q3 * q3)) * 57.3; //roll �����ǣ���y��ת��
    Pitch = asin(2 * (q0 * q1 - q2 *q3)) * 57.3; // pitch �����ǣ���x��ת��
    Yaw   = atan2(2 * (q0 * q2 + q1 * q3),  1 - 2 * (q1 * q1 + q2 *q2)) * 57.3; //yaw ƫ���ǣ���z��ת��
	  
   	//printf("Yaw=%f, Pitch=%f, Roll=%f\n", Yaw, Pitch, Roll);
	printf("Yaw=%f, Pitch=%f, Roll=%f\n", Yaw*57.3, Pitch*57.3, Roll*57.3);

   return;
}


/*******************************************************************************
���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}