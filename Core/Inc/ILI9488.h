#ifndef ILI9488_H
#define ILI9488_H

#include "main.h"
#include "spi.h"
#include "gpio.h"
#include <stdint.h>

#define CS_Pin GPIO_PIN_6 // Modify CS pin here
#define CS_GPIO_Port GPIOB // Modify CS port here
#define DC_Pin GPIO_PIN_5 // Modify DC pin here
#define DC_GPIO_Port GPIOB // Modify DC port here
#define RST_Pin GPIO_PIN_4 // Modify RST pin here
#define RST_GPIO_Port GPIOD // Modify RST port here
  
#define LCD_WIDTH 320  // Modify to your screen width
#define LCD_HEIGHT 480

/* ★★★ H7 400MHz 下 GPIO 翻转有几十ns延迟，必须用 DSB 屏障撑开时序 ★★★ */
#define CS_LOW()    do { HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); __DSB(); __NOP(); __NOP(); } while(0)
#define CS_HIGH()   do { HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);   __DSB(); __NOP(); __NOP(); } while(0)
#define DC_LOW()    do { HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);  __DSB(); __NOP(); __NOP(); } while(0)
#define DC_HIGH()   do { HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);    __DSB(); __NOP(); __NOP(); } while(0)
#define RST_LOW()   do { HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET); __DSB(); __NOP(); __NOP(); } while(0)
#define RST_HIGH()  do { HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);   __DSB(); __NOP(); __NOP(); } while(0)

#define LCD_WHITE       0xFFFF
#define LCD_BLACK       0x0000    
#define LCD_BLUE        0x001F  
#define LCD_BRED        0XF81F
#define LCD_GRED        0XFFE0
#define LCD_GBLUE       0X07FF
#define LCD_RED         0xF800
#define LCD_MAGENTA     0xF81F
#define LCD_GREEN       0x07E0
#define LCD_CYAN        0x7FFF
#define LCD_YELLOW      0xFFE0
#define LCD_BROWN       0XBC40
#define LCD_BRRED       0XFC07
#define LCD_GRAY        0X8430

typedef enum {
    ILI9488_DMA_IDLE = 0,
    ILI9488_DMA_BUSY,
    ILI9488_DMA_ERROR
} ILI9488_DMA_State;

typedef enum {
    ILI9488_ROTATION_0   = 0,    // Normal orientation
    ILI9488_ROTATION_90  = 1,    // Clockwise 90 degrees
    ILI9488_ROTATION_180 = 2,    // 180 degrees
    ILI9488_ROTATION_270 = 3     // Clockwise 270 degrees
} ILI9488_Rotation;

typedef void (*ILI9488_DMATxCpltCallback)(void);
void LCD_WriteCommand(uint8_t command);
void LCD_WriteData(uint8_t data);
void LCD_WriteData16(uint16_t data);
void LCD_WriteDataBuffer16(const uint16_t* buffer, uint32_t len);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void LCD_WaitDMAComplete(void);
ILI9488_DMA_State ILI9488_GetDMAState(void);
void ILI9488_SetDMAState(ILI9488_DMA_State state);
void ILI9488_SetDMACallback(ILI9488_DMATxCpltCallback callback);
void LCD_Reset(void);
void LCD_Init(void);
void LCD_SetRotation(ILI9488_Rotation rotation);
void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawImageRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t *colors);
uint16_t LCD_GetWidth(void);
uint16_t LCD_GetHeight(void);
void LCD_FillColor(uint16_t color);
void LCD_InitDMA(void);
void LCD_SPI_Init(void);
void SPI1_Reg_Transmit_N(uint8_t *data, uint32_t count);
uint16_t LCD_ReadID(void);
uint16_t LCD_ReadID_FullDuplex(void);

/* ── 6×8 字符绘制 ── */
void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg);
void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg);

/* ── FPS 显示函数 ── */
/**
 * @brief 在屏幕顶部显示 FPS（填充大小写颜色块并测量帧率）
 * @note  会独占约 1 秒进行测量，然后更新顶部 FPS 框。
 *        调用此函数的间隔可控制更新频率。
 */
void LCD_ShowFPS(void);

#endif
