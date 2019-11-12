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
#include "inertialMeasure.h"

extern rt_mq_t communicateQ;

typedef struct{
    uint8_t msgType;
    attitudeAngle_t  attitude;
}Communicate_Data_t;

enum{
    COMM_TYPR_ATTITUDE = 0,
};


void communicate_enter(void *para);













#endif
