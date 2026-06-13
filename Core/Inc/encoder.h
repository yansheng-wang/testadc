#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* 编码器初始化 (启动 TIM4) */
void Encoder_Init(void);

/* 读取编码器增量 (正=顺时针, 负=逆时针), 读取后清零 */
int32_t Encoder_ReadDelta(void);

/* 读取编码器按键 (PA11, 按下返回 true, 使用消抖) */
bool Encoder_ButtonPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */