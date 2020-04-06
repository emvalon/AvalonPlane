/*
 * *******************************************
 * Copyright (c) 2019 valon Shen 
 * E-mail: valonshen@foxmail.com
 * All right reserved
 * *******************************************
 */

#include "communicate.h"
#include "ulog.h"
#include "cJSON.h"
#include <string.h>

#define COMMU_Q_MAX             30



#define BYTE3(d)        ((d>>24)&0xffu)
#define BYTE2(d)        ((d>>16)&0xffu)
#define BYTE1(d)        ((d>>8)&0xffu)
#define BYTE0(d)        (d&0xffu)

rt_mq_t communicateQ;
static int create_communicate_queue(void)
{
    //create mailqueue
    communicateQ = rt_mq_create("commu_Q",sizeof(Communicate_Data_t),COMMU_Q_MAX,RT_IPC_FLAG_FIFO);
    if(communicateQ == RT_NULL){
        LOG_E("commuQ create fault\r\n");
        while(1);
    }  
    return 0;
}
INIT_APP_EXPORT(create_communicate_queue);

uint8_t data_to_send[100];
void Data_Send_Status(attitudeAngle_t *angle)
{
  
	uint8_t     i,_cnt  =0;
    uint8_t     sum     = 0;
    uint16_t    _temp   = 0;
    uint32_t    _temp2  = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (uint16_t)(angle->rollDegree *100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (uint16_t)(angle->pitchDegree *100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (uint16_t)(angle->yawDepree *100);
	//_temp = (int)(Mag_Heading*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0xff;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp2 = 0xffffffff;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	//lock
	data_to_send[_cnt++]=0xA0;
	
	data_to_send[3] = _cnt-4;
	
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
//
//	LOG_RAW(data_to_send); 
        for( i=0;i<_cnt;i++){
            rt_kprintf("%c",data_to_send[i]);
        }
}


cJSON* buildAttitudeJSON(attitudeAngle_t* att)
{
    cJSON *root;
    root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root,"Roll",att->rollDegree);
    cJSON_AddNumberToObject(root,"Pitch",att->pitchDegree);
    cJSON_AddNumberToObject(root,"Yaw",att->yawDepree);
    return root;
}

uint8_t printForm = 0;
void communicate_enter(void *para)
{
    rt_err_t res;
    Communicate_Data_t recvData;
    cJSON *root;
    char* str;
    while(1){
        res = rt_mq_recv(communicateQ,&recvData,sizeof(recvData),RT_WAITING_FOREVER);
        if(res == RT_EOK){
          switch (recvData.msgType)
          {
          case COMM_TYPR_ATTITUDE:
            if(printForm){
                Data_Send_Status(&recvData.attitude);
            }
            else{
                LOG_D(str = cJSON_Print(root = buildAttitudeJSON(&recvData.attitude)));
                cJSON_Delete(root);
                rt_free(str);
            }
            break;
          default:
            break;
          }
        } 
        else{
            LOG_E("commu recv MQ error:%d",res);
        }
    }
    
}

void showAttitude(int argc,char** arg)
{
    if(argc != 2 ){
        rt_kprintf("usage:\n\tshowAttitude <json/hex>\n");
        return;
    }
    if( strcmp(arg[1],"json") == 0 ){
        printForm = 0;
    }
    else if( strcmp(arg[1],"hex") == 0 ){
        printForm = 1;
    }
    else{
        rt_kprintf("usage:\n\tshowAttitude <json/hex>\n");
    }
}

MSH_CMD_EXPORT(showAttitude,"print attitude by Json");










