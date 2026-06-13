#include "encoder.h"
#include "tim.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 编码器初始化 (启动 TIM4) */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/* 读取编码器增量 (正=顺时针, 负=逆时针), 读取后清零 */
int32_t Encoder_ReadDelta(void)
{
    static int32_t last_count = 0;
    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t delta = current - last_count;
    
    /* 处理计数回绕 (65536 周期) */
    if (delta > 32768)  delta -= 65536;
    if (delta < -32768) delta += 65536;
    
    last_count = current;
    return delta;
}

/* 读取编码器按键 (PA11, 按下返回 true, 带消抖) */
bool Encoder_ButtonPressed(void)
{
    static uint32_t last_press_time = 0;
    
    /* 读取 PA11 电平 (假设按下为低电平) */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
        uint32_t now = HAL_GetTick();
        /* 消抖: 距离上次按键至少 300ms */
        if (now - last_press_time > 300) {
            last_press_time = now;
            return true;
        }
    }
    return false;
}