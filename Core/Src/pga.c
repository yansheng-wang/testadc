#include "pga.h"
#include "main.h"

void PGA_SET(float value)
{
    if (value == 0.125f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_RESET);
    }
    else if (value == 0.25f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_RESET);
    }
    else if (value == 0.5f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_RESET);
    }
    else if (value == 1.0f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_RESET);
    }
    else if (value == 2.0f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_SET);
    }
    else if (value == 4.0f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_SET);
    }
    else if (value == 8.0f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_SET);
    }
    else if (value == 16.0f) {
        HAL_GPIO_WritePin(PGA1_0_GPIO_Port, PGA1_0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_1_GPIO_Port, PGA1_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PGA1_2_GPIO_Port, PGA1_2_Pin, GPIO_PIN_SET);
    }
}