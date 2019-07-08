#ifndef __LED_DRIVER_H__
#define __LED_DRIVER_H__
#include <rtthread.h>



#define LED1_TORGER     GPIOA->ODR ^= GPIO_PIN_4
#define LED2_TORGER     GPIOB->ODR ^= GPIO_PIN_7
#define LED3_TORGER     GPIOB->ODR ^= GPIO_PIN_6
#define LED4_TORGER     GPIOB->ODR ^= GPIO_PIN_2

#define LED1_ON         GPIOA->BSRR = GPIO_PIN_4<<16
#define LED2_ON         GPIOB->BSRR = GPIO_PIN_7<<16
#define LED3_ON         GPIOB->BSRR = GPIO_PIN_6<<16
#define LED4_ON         GPIOB->BSRR = GPIO_PIN_2<<16

#define LED1_OFF        GPIOA->BSRR = GPIO_PIN_4
#define LED2_OFF        GPIOB->BSRR = GPIO_PIN_7
#define LED3_OFF        GPIOB->BSRR = GPIO_PIN_6
#define LED4_OFF        GPIOB->BSRR = GPIO_PIN_2




enum led_control_mode
{
    LED_STOP = 0,
    LED_BLINK ,
    LED_DOUBLE_BLINK,
};

struct led_blink_ctl
{
    rt_tick_t onTime;
    rt_tick_t offTime;
};
struct led_doubleBlink_ctl
{  
    rt_tick_t PreOnTime;
    rt_tick_t PreOffTime;
    rt_tick_t onTime;
    rt_tick_t offTime;
};

typedef struct
{
    uint8_t mode;
    uint8_t step;
    union 
    {
        struct led_blink_ctl led_blink;
        struct led_doubleBlink_ctl led_doubleBlink;          
    }ctl;
}led_control_t;



void led_blink_timeout_callBack(void* para);





#endif
