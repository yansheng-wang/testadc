#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ADC_DisplayOnLCD(void);

/* 屏幕驱动测试函数 */
void LCD_Test_FillColors(void);     /* 红绿蓝白黑全屏切换 */
void LCD_Test_CornerBlocks(void);   /* 四角色块 */
void LCD_Test_Cross(void);          /* 十字交叉线 + 文字 */
void LCD_Test_Text(void);           /* 字体颜色测试 */
void LCD_Test_Gradient(void);       /* 7 色渐变彩条 */
void LCD_Test_DiagRect(void);       /* 对角线 + 矩形边框 */

#ifdef __cplusplus
}
#endif

#endif