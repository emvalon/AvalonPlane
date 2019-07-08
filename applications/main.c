/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "sensor_thread.h"
#include "sensor_inven_mpu6xxx.h"
#include "ulog.h"

// read sensor thread
#define READ_SENSOR_STACK_SIZE      800
#define READ_SENSOR_PRO             5
#define READ_SENSOR_SLICE           2
// process sensor data thread
#define PROCESS_SENSOR_STACK_SIZE   800
#define PROCESS_SENSOR_PRO          6
#define PROCESS_SENSOR_SLICE        2



int main(void)
{
    
    rt_thread_t readSensorThread;
    rt_thread_t processSensorThread;
    /* user app entry */
   // ulog_global_filter_lvl_set(LOG_LVL_INFO);
    readSensorThread    = rt_thread_create(   
                            "readSen",      read_sensor_enter,  
                            RT_NULL,            READ_SENSOR_STACK_SIZE, 
                            READ_SENSOR_PRO,    READ_SENSOR_SLICE   );
    
    processSensorThread = rt_thread_create(   
                            "process",   process_sensor_enter,  
                            RT_NULL,            PROCESS_SENSOR_STACK_SIZE, 
                            PROCESS_SENSOR_PRO, PROCESS_SENSOR_SLICE   );
    
    rt_thread_startup(processSensorThread);
    rt_thread_startup(readSensorThread);
    
    return 0;
    
}


/* *****************************************
        ����mpu6050�豸
******************************************** */
int add_device_mpu6050()
{
    struct rt_sensor_config config;
    
    memset(&config,0,sizeof(config));
    config.intf.dev_name    = "i2c1";
    config.irq_pin.pin      = GET_PIN(C,13);  
    config.irq_pin.mode     = PIN_MODE_INPUT_PULLDOWN;   
    
    rt_hw_mpu6xxx_init("mpu6050",&config);
    return 0;
}
INIT_DEVICE_EXPORT(add_device_mpu6050);