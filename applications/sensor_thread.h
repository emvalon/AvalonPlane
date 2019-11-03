#ifndef __SENSOR_THREAD_H__
#define __SENSOR_THREAD_H__

#include "sensor.h"

struct sensor_readData{
    uint8_t         type;
    rt_hwtimerval_t time;
    union data
    {
        struct mpu6050
        {
            struct sensor_3_axis    dataAccel;
            struct sensor_3_axis    dataGyro;
        };
        
    };   
};

typedef struct sensor_readData sensor_readData_t;




void read_sensor_enter(void *para);
void process_sensor_enter(void *para);
    

#endif
