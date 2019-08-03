/*
 * *******************************************
 * Copyright (c) 2019 valon Shen 
 * E-mail: valonshen@foxmail.com
 * All right reserved
 * *******************************************
 */
#ifndef _COMMUNICATE_H_
#define _COMMUNICATE_H_


#include "rtthread.h"
#include "sensor_thread.h"
#include "IMU.h"

extern rt_mq_t communicateQ;

typedef struct{
    Attitude_t  Q_angle;
}Communicate_Data_t;



void communicate_enter(void *para);













#endif
