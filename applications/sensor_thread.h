#ifndef __SENSOR_THREAD_H__
#define __SENSOR_THREAD_H__

#include "sensor.h"

struct mpu6050_data{
    uint8_t     type;
    uint16_t    time;
    struct sensor_3_axis    dataAccel;
    struct sensor_3_axis    dataGyro;
};

typedef struct mpu6050_data mpu6050_data_t;




void read_sensor_enter(void *para);
void process_sensor_enter(void *para);
    

#endif
