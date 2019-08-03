/*
 * *******************************************
 * Copyright (c) 2019 valon Shen 
 * E-mail: valonshen@foxmail.com
 * All right reserved
 * *******************************************
 */
#include "IMU.h"
#include "stdint.h"
#include "math.h"

#define filter_num 20



//���Ư���趨
#define APX (-1000)
//y    -241
#define APY (300)
#define APZ (-724)
#define GPX (-21)
#define GPY (3)
#define GPZ (15)

Int_xyz_t Acc_buf[ filter_num ], Acc_buf[ filter_num ], Acc_buf[ filter_num ],Acc_avg;
Float_xyz_t temp,Gyro_ready;
extern Int_xyz_t Acc_data,Gyro_data;
uint8_t Imu_DataReady;
//extern uint t,count[3];      

  void Read_6050()
{
  
 	static uint8_t filter_cnt = 0;
  	temp.x=0;temp.y=0;temp.z=0;
	
	uint8_t i;

//	MPU6050_Read();

   /*     
         if(count[2]>=count[1])
           t=count[2]-count[1];
         else
           t=65536+count[2]-count[1];
  
         
         t/=1000000.0;*/
	Acc_buf[ filter_cnt ].x = Acc_data.x;			//�������ݡ��Ƚ��ȳ�
	Acc_buf[ filter_cnt ].y = Acc_data.y;
	Acc_buf[ filter_cnt ].z = Acc_data.z;

	for(i = 0; i<filter_num; i++)
	{
		temp.x += Acc_buf[i].x;
		temp.y += Acc_buf[i].y;
		temp.z += Acc_buf[i].z;
	}
	Acc_avg.x =(int)( temp.x / filter_num )-APX;			//ȡ��ֵ
	Acc_avg.y =(int)( temp.y / filter_num )-APY;
	Acc_avg.z =(int)( temp.z / filter_num )-APZ;
	filter_cnt ++;
	if(filter_cnt == filter_num)				//ȡ��20ʱ����
		filter_cnt = 0;
        Imu_DataReady = 1;


	Gyro_ready.x=(Gyro_data.x-GPX)*0.061035;
        Gyro_ready.y=(Gyro_data.y-GPY)*0.061035;
	Gyro_ready.z=(Gyro_data.z-GPZ)*0.061035;
}

void Get_Attitude()
{
    if(Imu_DataReady)
    {
          Imu_DataReady=0;
          IMU_Updata(Gyro_ready.x*0.017453,Gyro_ready.y*0.017453,Gyro_ready.z*0.017453, Acc_avg.x, Acc_avg.y, Acc_avg.z,0);
    }
} 

/**************************************************************************************	

                               �� �� �� ��
***************************************************************************************/
#define Kp 20.0f
#define Ki 0.008f
#define halfT 0.0025f

float  q0 = 1, q1 = 0, q2 = 0, q3  = 0;                         //������Ԫ����Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;


void IMU_Updata(float gx, float gy, float gz, float ax, float ay, float az,Attitude_t* Q)
{

    float norm;
    //float hx, hy, hz, bx, bz;
    float vx, vy, vz;// wx, wy, wz;
    float ex, ey, ez;

    //�Ȱ���Щ��Ҫ�õ���ֵŪ��
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
//    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
//    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az==0) return;
          
    norm = sqrt(ax*ax + ay*ay + az*az);//acc���ݹ�һ��
    ax = ax /norm;
    ay = ay / norm;
    az = az / norm;

    //  �����������������/��Ǩ
    vx = 2*(q1q3 - q0q2);	//��Ԫ����xyz�ı�ʾ
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    ex = (ay*vz - az*vy) ;   //�������������õ���־������
    ey = (az*vx - ax*vz) ;
    ez = (ax*vy - ay*vx) ;

    exInt = exInt + ex * Ki; //�������л���
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //�����PI�󲹳��������ǣ����������Ư��
    gx = gx + Kp*ex + exInt; 
    gy = gy + Kp*ey + eyInt;
    //�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ������������Լ�
    gz = gz + Kp*ez + ezInt; 

    //��Ԫ����΢�ַ���
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;


    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    // yaw Q_angle.z
    Q->yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; 
    // pitch  Q_angle.y
    Q->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.32784; 
    // roll Q_angle.x
    Q->roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.32484; 

}