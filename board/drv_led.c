/*
 * Copyright (c) 2006-2018, valon shen(valonshen@foxmail.com)
 *
 * Change Logs:
*/

#include "drv_led.h"
#include <board.h>


static rt_timer_t ledBlinkTimer;
static led_control_t gLed_ctlPara;


/********************************************
led单次闪烁处理
*********************************************/
static void led_blink_process(void)
{
    if( gLed_ctlPara.step ){                   
        LED1_OFF,LED2_OFF,LED3_OFF,LED4_OFF;
        rt_timer_control(ledBlinkTimer,RT_TIMER_CTRL_SET_TIME, (void*)(&gLed_ctlPara.ctl.led_blink.offTime));
        gLed_ctlPara.step = 0;
    }
    else{
        LED1_ON,LED2_ON,LED3_ON,LED4_ON;
        rt_timer_control(ledBlinkTimer,RT_TIMER_CTRL_SET_TIME, (void*)(&gLed_ctlPara.ctl.led_blink.onTime));
        gLed_ctlPara.step = 1;
    }
}
/********************************************
led双次闪烁处理
*********************************************/
static void led_doubleBlink_process(void)
{
    switch( gLed_ctlPara.step ){
        case 0:
            LED1_ON,LED2_ON,LED3_ON,LED4_ON;
            rt_timer_control(   ledBlinkTimer,  RT_TIMER_CTRL_SET_TIME, 
                                (void*)(&gLed_ctlPara.ctl.led_doubleBlink.PreOnTime));
            gLed_ctlPara.step++;
            break;
        case 1:
            LED1_OFF,LED2_OFF,LED3_OFF,LED4_OFF;
            rt_timer_control(   ledBlinkTimer,  RT_TIMER_CTRL_SET_TIME, 
                                (void*)(&gLed_ctlPara.ctl.led_doubleBlink.PreOffTime));
            gLed_ctlPara.step++;
            break;
        case 2:
            LED1_ON,LED2_ON,LED3_ON,LED4_ON;
            rt_timer_control(   ledBlinkTimer,  RT_TIMER_CTRL_SET_TIME, 
                                (void*)(&gLed_ctlPara.ctl.led_doubleBlink.onTime));
            gLed_ctlPara.step++;
            break;
        default:
            LED1_OFF,LED2_OFF,LED3_OFF,LED4_OFF;
            rt_timer_control(   ledBlinkTimer,  RT_TIMER_CTRL_SET_TIME, 
                                (void*)(&gLed_ctlPara.ctl.led_doubleBlink.offTime));
            gLed_ctlPara.step = 0;
            break;
    }
}

void led_blink_timeout_callBack(void* para)
{
    switch(gLed_ctlPara.mode){
        case LED_STOP:
            LED1_OFF,LED2_OFF,LED3_OFF,LED4_OFF;
            return;
        case LED_BLINK:          
            led_blink_process();
            break;
        case LED_DOUBLE_BLINK:
            led_doubleBlink_process();
            break;
    }
    rt_timer_start(ledBlinkTimer);
}

static int LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB2 PB6 PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    return 0;
}

rt_err_t  ops_led_control(rt_device_t dev, int cmd, void *args)
{
    const rt_tick_t start_time = 0;
    
    if(cmd != RT_DEVICE_CTRL_CONFIG){
        return 1;
    }
    rt_timer_stop(ledBlinkTimer);
    rt_memcpy(&gLed_ctlPara,args,sizeof(gLed_ctlPara));
    rt_timer_control(ledBlinkTimer,RT_TIMER_CTRL_SET_TIME,(void*)&start_time);
    rt_timer_start(ledBlinkTimer);
    return 0;
}
/****************************************************
        create and register led device
*****************************************************/
int device_led_init(void)
{
    static rt_device_t  led_device;
    
    LED_GPIO_Init();
    led_device = rt_device_create(RT_Device_Class_Miscellaneous,0);
    led_device->control = ops_led_control;
    rt_device_register(led_device,"led_dev",RT_DEVICE_FLAG_WRONLY);
    
    
    ledBlinkTimer = rt_timer_create(    "LED_Soft",    led_blink_timeout_callBack,  
                                        RT_NULL,       100,
                                        RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    return 0;
}
INIT_DEVICE_EXPORT(device_led_init);

/****************************************************
        export led ctl shell cmd
*****************************************************/

void light_led(int argc,char** arg)
{
    rt_device_t led_dev;
    led_control_t led_ctlPara;
    
    if(argc != 2){
        rt_kprintf("input right mode\r\n");
        return;   
    }  
    led_dev = rt_device_find("led_dev");
    if(led_dev == RT_NULL){
        rt_kprintf("no find device:led_dev\r\n");
        return;
    }  
    switch(arg[1][0]){
        case '0':
            led_ctlPara.mode = LED_STOP;
            break;
        case '1':
            led_ctlPara.mode = LED_BLINK;
            led_ctlPara.step = 0;
            led_ctlPara.ctl.led_blink.offTime = 400;
            led_ctlPara.ctl.led_blink.onTime  = 100;
            break;
        case '2':
            led_ctlPara.mode = LED_DOUBLE_BLINK;
            led_ctlPara.step = 0;
            led_ctlPara.ctl.led_doubleBlink.PreOnTime   = 50;
            led_ctlPara.ctl.led_doubleBlink.PreOffTime  = 200;
            led_ctlPara.ctl.led_doubleBlink.onTime      = 50;
            led_ctlPara.ctl.led_doubleBlink.offTime     = 800;
            break;
        default:
            rt_kprintf("input right mode\r\n");
            return;
    } 
    rt_device_control(led_dev,RT_DEVICE_CTRL_CONFIG,&led_ctlPara);
}
MSH_CMD_EXPORT(light_led,"light up led with different mode");












