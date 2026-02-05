/*
 * printf.c
 *
 *  Created on: Jan 12, 2026
 *      Author: quyen
 */


#include "printf.h"

#include "printf.h"
#include "cmsis_os2.h"

extern osMutexId_t myMutex01Handle;

void myPrintf(UART_HandleTypeDef *huart, const char *format, ...)
{
    char buff[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buff, sizeof(buff), format, args);
    va_end(args);
    if (len < 0) return;
    if (len >= (int)sizeof(buff)) len = sizeof(buff) - 1;

    if (osKernelGetState() == osKernelRunning)
        osMutexAcquire(myMutex01Handle, osWaitForever);

    HAL_UART_Transmit(huart, (uint8_t*)buff, len, 100);

    if (osKernelGetState() == osKernelRunning)
        osMutexRelease(myMutex01Handle);
}
