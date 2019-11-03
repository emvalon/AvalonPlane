/*
 * *******************************************
 * Copyright (c) 2019 , valon Shen 
 * E-mail: valonshen@foxmail.com
 * All right reserved
 * *******************************************
 */
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <string.h>
#include "sensor.h"
#include "sensor_thread.h"
#include "communicate.h"
#include "ulog.h"
#include "IMU.h"

#define SENSOR_TIMER        "timer2"

#define SENSOR_MQ_SIZE      60u
#define MPU_INT_PIN         2u
#define MPU_READY_FLAG      1ul

#define DATA_TYPE_MPU6050    1


static rt_mq_t      sensorMQ;
static rt_event_t   sensorEvent;
static rt_device_t  acc_mpu6050,gyro_mpu6050,sensor_timer;



static int create_sensor_queue(void)
{
    //create sensor mailbox
    sensorMQ = rt_mq_create("sensorMQ",sizeof(sensor_readData_t),SENSOR_MQ_SIZE,RT_IPC_FLAG_FIFO);      
    if(sensorMQ == RT_NULL){
        LOG_E("sensor mb create fault\r\n");
        while(1);
    }
    //create sensor event
    sensorEvent = rt_event_create("sensorE",0);
    if(sensorEvent == RT_NULL){
        LOG_E("sensor event create fault\r\n");
        while(1);
    }
    return 0;
}
INIT_APP_EXPORT(create_sensor_queue);



/***********************************
 read sensor thread enter
************************************/
static rt_err_t mpu6050_rx_callBack(rt_device_t dev, rt_size_t size)
{
    if(rt_event_send(sensorEvent,MPU_READY_FLAG) != RT_EOK){
        LOG_E("sensor data ready set fault\r\n");
    }
    return 0;
}

static void sensorTimerInit(void)
{
    uint32_t freq;
    rt_hwtimer_mode_t mode;
    rt_hwtimerval_t time;

    sensor_timer = rt_device_find(SENSOR_TIMER);
    if(sensor_timer == RT_NULL){
        LOG_W("sensor_timer not find\r\n");
        return;
    }
    //设置计时定时器
    if(rt_device_open(sensor_timer,RT_DEVICE_OFLAG_RDWR) != RT_EOK){
        LOG_W("sensor_timer open failure\r\n");
        return;
    }
    freq = 1000000;
    if(RT_EOK != rt_device_control(sensor_timer,HWTIMER_CTRL_FREQ_SET,&freq)){
        LOG_W("sensor_timer set frequency failure\r\n");
        return;
    }
    mode = HWTIMER_MODE_PERIOD;
    if(RT_EOK != rt_device_control(sensor_timer,HWTIMER_CTRL_MODE_SET,&mode)){
        LOG_W("sensor_timer set mode failure\r\n");
        return;
    }
    time.sec = 0;
    time.usec= 6500;
    if(sizeof(time) != rt_device_write(sensor_timer,0,&time,sizeof(time))){
        LOG_W("sensor_timer write failure\r\n");
        return;
    }
}

static void mpu6050Init(void)
{
    acc_mpu6050 = rt_device_find("acce_mpu");
    if(acc_mpu6050 == RT_NULL){
        LOG_W("acc_mpu6050 not find\r\n");
        return;
    }
    gyro_mpu6050 = rt_device_find("gyro_mpu");
    if(gyro_mpu6050 == RT_NULL){
        LOG_W("gyro_mpu6050 not find\r\n");
        return;
    }
    //设置mpu接收回调函数
    rt_device_set_rx_indicate(acc_mpu6050,mpu6050_rx_callBack);
    //设置低通滤波
    rt_device_control(acc_mpu6050, RT_SENSOR_CTRL_SET_DLPF , (void*)4);
    rt_device_control(acc_mpu6050, RT_SENSOR_CTRL_SET_ODR , (void*)200);
    
  
    rt_device_open(acc_mpu6050, RT_DEVICE_FLAG_INT_RX );
    //rt_device_open(gyro_mpu6050, RT_DEVICE_FLAG_INT_RX );
}

void read_sensor_enter(void *para)
{
    rt_device_t  acc_mpu6050,gyro_mpu6050;
    uint32_t flag;
    sensor_readData_t  readData;
    struct rt_sensor_data dataTemp;
    rt_err_t    res;
    
    //初始化定时器，用于获取精确的系统时间
    sensorTimerInit();
    //初始化MPU
    mpu6050Init();

    while(1){
        res = rt_event_recv( sensorEvent,           MPU_READY_FLAG, 
                             RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,
                             RT_WAITING_FOREVER,    &flag       );
        if(res != RT_EOK){
            LOG_W("recv sensor ready flag fault!");
        }
        else{
            rt_memset(&readData,0,sizeof(readData));
            //获取当前时间
            if(rt_device_read(sensor_timer,0,&readData.time,sizeof(readData.time)) != sizeof(readData.time) ){
                LOG_E("hwtimer read error!");
                rt_thread_mdelay(10);
                continue;
            }
            rt_memset(&dataTemp,0,sizeof(dataTemp));
            if(flag & MPU_READY_FLAG){    
                readData.type         = DATA_TYPE_MPU6050;
                //读取加速度计数据
                rt_device_read(acc_mpu6050,0,(void*)&dataTemp,sizeof(dataTemp));
                readData.dataAccel.x  = dataTemp.data.acce.x;
                readData.dataAccel.y  = dataTemp.data.acce.y;
                readData.dataAccel.z  = dataTemp.data.acce.z;
                rt_memset(&dataTemp,0,sizeof(dataTemp));
                //读取陀螺仪数据
                rt_device_read(gyro_mpu6050,0,(void*)&dataTemp,sizeof(dataTemp));
                readData.dataGyro.x  = dataTemp.data.gyro.x;
                readData.dataGyro.y  = dataTemp.data.gyro.y;
                readData.dataGyro.z  = dataTemp.data.gyro.z;
                //send data mailQ
                res = rt_mq_send(sensorMQ,&readData,sizeof(readData));
                if(res != RT_EOK){
                    LOG_W("sensor send fault:%d\r\n",res );  
                    rt_thread_mdelay(10);
                }
            }
        }
        flag = 0;
    }
}


/***********************************
 process sensor data thread enter
************************************/
void process_sensor_enter(void *para)
{
    rt_err_t    res; 
    sensor_readData_t  data;
    Communicate_Data_t  commData;
    uint32_t sec=0;
    uint32_t cnt=0;
 
    while(1){ 
        res = rt_mq_recv(sensorMQ,&data,sizeof(data),RT_WAITING_FOREVER);
        if(res == RT_EOK){            
            switch ( data.type )
            {
            case DATA_TYPE_MPU6050:
                cnt++;
                IMU_Updata(data.dataGyro.x*0.017453/1000.0,data.dataGyro.y*0.017453/1000.0,data.dataGyro.z*0.017453/1000.0,
                            data.dataAccel.x,data.dataAccel.y,data.dataAccel.z,&commData.Q_angle);
                if(data.time.sec > sec){ 
//                  LOG_D("x:%d y%d z%d",data.dataAccel.x,data.dataAccel.y,data.dataAccel.z);
                    if(RT_EOK != rt_mq_send(communicateQ,&commData,sizeof(commData)) ){
                        LOG_W("send to commu fault");
                    }
                    LOG_D("RECV data cnt=%d",cnt);
                    sec = data.time.sec;
                    cnt = 0;
                }
//                tick = rt_tick_get();
//                if( tick - preTick >=1000 ){
//                    LOG_D("read cnt:%d\r\n",cnt);
//                    preTick = tick;
//                    cnt=0;
//                }
                break;
            
            default:
                break;
            
            
            }
        }
        else{
            LOG_E("process recv MQ error:%d",res);
        }
        
    }
}




















