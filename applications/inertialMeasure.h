/*
 * *******************************************
 * Copyright (c) 2019 , valon Shen 
 * E-mail: valonshen@foxmail.com
 * All right reserved
 * *******************************************
 */
#ifndef _INERTIALMEASURE_H_
#define _INERTIALMEASURE_H_

//宏定义
#define MATH_PI 3.1415926

//自定义结构体
typedef struct 
{
    float x,y,z;
}triaxialFloat_t;

typedef struct 
{
    float rollRadian,pitchRadian,yawRadian;
    float rollDegree,pitchDegree,yawDepree;
}attitudeAngle_t;




//函数声明
void calculateAttitudeAngle(triaxialFloat_t *acce, triaxialFloat_t *gyro, triaxialFloat_t *magnet, float dt , attitudeAngle_t *attitude);

#endif