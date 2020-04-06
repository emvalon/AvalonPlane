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
#include "filter.h"
#include "inertialMeasure.h"


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
        LOG_W("sensor data ready set fault\r\n");
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
        LOG_E("sensor_timer not find\r\n");
        return;
    }
    //设置计时定时器
    if(rt_device_open(sensor_timer,RT_DEVICE_OFLAG_RDWR) != RT_EOK){
        LOG_E("sensor_timer open failure\r\n");
        return;
    }
    freq = 1000000;
    if(RT_EOK != rt_device_control(sensor_timer,HWTIMER_CTRL_FREQ_SET,&freq)){
        LOG_E("sensor_timer set frequency failure\r\n");
        return;
    }
    mode = HWTIMER_MODE_PERIOD;
    if(RT_EOK != rt_device_control(sensor_timer,HWTIMER_CTRL_MODE_SET,&mode)){
        LOG_E("sensor_timer set mode failure\r\n");
        return;
    }
    time.sec = 0;
    time.usec= 6500;
    if(sizeof(time) != rt_device_write(sensor_timer,0,&time,sizeof(time))){
        LOG_E("sensor_timer write failure\r\n");
        return;
    }
}

static void mpu6050Init(void)
{
    rt_err_t res;
    acc_mpu6050 = rt_device_find("acce_mpu");
    if(acc_mpu6050 == RT_NULL){
        LOG_E("acc_mpu6050 not find\r\n");
        return;
    }
    gyro_mpu6050 = rt_device_find("gyro_mpu");
    if(gyro_mpu6050 == RT_NULL){
        LOG_E("gyro_mpu6050 not find\r\n");
        return;
    }
    //设置mpu接收回调函数
    rt_device_set_rx_indicate(acc_mpu6050,mpu6050_rx_callBack);
    //设置低通滤波
    rt_device_control(acc_mpu6050, RT_SENSOR_CTRL_SET_DLPF , (void*)4);
    rt_device_control(acc_mpu6050, RT_SENSOR_CTRL_SET_ODR , (void*)100);
    
    res = rt_device_open(acc_mpu6050, RT_DEVICE_FLAG_INT_RX );
    if(RT_EOK != res  ){
        LOG_E("acce open error :%d",res);
    }
    res = rt_device_open(gyro_mpu6050, RT_DEVICE_FLAG_INT_RX );
    if(RT_EOK != res  ){
        LOG_E("gyro open error :%d",res);
    }
}

void read_sensor_enter(void *para)
{
    uint32_t flag;
    sensor_readData_t  readData;
    struct rt_sensor_data dataTemp;
    rt_err_t    res;
    
    //初始化定时器，用于获取精确的系统时间
    sensorTimerInit();
    //初始化MPU
    mpu6050Init();
    
   
    rt_thread_mdelay(1000);
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
                }
            }
        }
        flag = 0;
    }
}


/***********************************
 process sensor data thread enter
************************************/
#define so3_comp_params_Kp 1.0f
#define so3_comp_params_Ki  0.05f
void process_sensor_enter(void *para)
{
    rt_err_t                res; 
    triaxialFloat_t         acce,gyro,magnet;
    attitudeAngle_t         attitude;
    sensor_readData_t       recvData;
    rt_hwtimerval_t         preTime;
    Communicate_Data_t      commData;
    uint32_t    cnt=0;
    float       dt;
 
    //初始化low frequency filter
    filterInit();

    rt_memset(&preTime,0,sizeof(preTime));
    rt_memset(&magnet,0,sizeof(magnet));

    while(1){ 
        res = rt_mq_recv(sensorMQ,&recvData,sizeof(recvData),RT_WAITING_FOREVER);
        if(res == RT_EOK){            
            switch ( recvData.type )
            {
            case DATA_TYPE_MPU6050:
                cnt++;
                if(recvData.time.usec > preTime.usec){
                    dt = (recvData.time.usec - preTime.usec);
                }
                else{
                    dt = 1000000 - preTime.usec + recvData.time.usec;
                }
                //转为秒为单位
                dt /= 1000000.0;
                //低通滤波进行滤波
               acce.x = LPF2pApply_1(recvData.dataAccel.x);
               acce.y = LPF2pApply_2(recvData.dataAccel.y);
               acce.z = LPF2pApply_3(recvData.dataAccel.z);
               gyro.x = LPF2pApply_4(recvData.dataGyro.x*0.017453/1000.0);
               gyro.y = LPF2pApply_5(recvData.dataGyro.y*0.017453/1000.0);
               gyro.z = LPF2pApply_6(recvData.dataGyro.z*0.017453/1000.0);
//                
                // acce.x = (float)(recvData.dataAccel.x);
                // acce.y = (float)(recvData.dataAccel.y);
                // acce.z = (float)(recvData.dataAccel.z);

                // gyro.x = (recvData.dataGyro.x*0.017453/1000.0);
                // gyro.y = (recvData.dataGyro.y*0.017453/1000.0);
                // gyro.z = (recvData.dataGyro.z*0.017453/1000.0);

               calculateAttitudeAngle(&acce,&gyro,&magnet,dt,&attitude);
               //if(recvData.time.sec > preTime.sec){ 
                if(cnt >= 20){
                    commData.msgType  = COMM_TYPR_ATTITUDE;
                    commData.attitude = attitude;
                    if(RT_EOK != rt_mq_send(communicateQ,&commData,sizeof(commData)) ){
                        LOG_W("send to commu fault");
                    }
                    // LOG_D("RECV data cnt=%d  dt=%d",cnt,(int)(dt*10000));
                    cnt = 0;
                }
                break;
            
            default:
                break;
            
            
            }
            preTime = recvData.time;
        }
        else{
            LOG_E("process recv MQ error:%d",res);
        }
        
    }
}




















