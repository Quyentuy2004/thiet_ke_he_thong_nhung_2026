/*
 * adc_vdda.c
 *
 *  Created on: Feb 3, 2026
 *      Author: quyen
 */


#include "stm32f4xx_hal.h"

/* ===================== TÙY CHỌN / HẰNG SỐ ===================== */
#define ADC_MAX_12BIT      4095.0f
#define VDDA_CALIB         3.3f

/*
  VREFINT_CAL: giá trị calibration do ST lưu trong System Memory (đo ở VDDA=3.3V).
  Địa chỉ có thể khác giữa các dòng F4.
  - Nếu dự án của bạn đã có macro VREFINT_CAL_ADDR thì dùng nó.
  - Nếu không có, bạn phải chỉnh địa chỉ đúng theo chip (tra datasheet/RM).
*/
#ifndef VREFINT_CAL_ADDR
  #define VREFINT_CAL_ADDR ((uint16_t*)0x1FFF7A2A)   // phổ biến trên nhiều STM32F4, nhưng KHÔNG chắc cho mọi chip
#endif

#define VREFINT_CAL        (*((uint16_t*)VREFINT_CAL_ADDR))

/* ===================== HÀM NỘI BỘ ===================== */

// Chờ ngắn để kênh ADC ổn định sau khi đổi channel
static void adc_short_delay(void)
{
  // ~ vài us đến vài chục us (tùy clock); viết đơn giản cho HAL
  for (volatile int i = 0; i < 200; i++) { __NOP(); }
}

// Đọc ADC 1 lần (polling) -> raw
static uint16_t ADC_ReadRaw_Blocking(ADC_HandleTypeDef *hadc, uint32_t timeout_ms)
{
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, timeout_ms);
  uint16_t raw = (uint16_t)HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  return raw;
}

// Đổi channel cho ADC (HAL)
static HAL_StatusTypeDef ADC_SetChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t sampleTime)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel      = channel;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = sampleTime;

  // Một số STM32F4 có thêm Offset, nhưng HAL thường để mặc định 0
  return HAL_ADC_ConfigChannel(hadc, &sConfig);
}

/* ===================== API CHÍNH ===================== */

/*
  1) Bật VREFINT (cần gọi 1 lần sau khi init ADC)
*/
void ADC_EnableVrefint(ADC_HandleTypeDef *hadc)
{
  // STM32F411 chỉ có ADC1
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
  }

  // Bật đường VREFINT / Temp sensor
  ADC->CCR |= ADC_CCR_TSVREFE;

  // Bật ADC (macro cần HANDLE)
  __HAL_ADC_ENABLE(hadc);

  // Delay ngắn cho ổn định
  for (volatile int i = 0; i < 10000; i++) { __NOP(); }
}



/*
  2) Đọc VDDA thực tế dựa trên VREFINT
  - hadc: ADC handle
  - samples: số mẫu trung bình
  - sampleTime: sampling time cho VREFINT (nên để lớn, ví dụ ADC_SAMPLETIME_144CYCLES hoặc 480)
*/
float ADC_MeasureVDDA_FromVrefint(ADC_HandleTypeDef *hadc, uint16_t samples, uint32_t sampleTime)
{
  if (samples == 0) samples = 1;

  // Config kênh VREFINT
  if (ADC_SetChannel(hadc, ADC_CHANNEL_VREFINT, sampleTime) != HAL_OK) {
    return 0.0f;
  }

  adc_short_delay();

  // Bỏ 1 lần đọc đầu (sau khi đổi channel thường không ổn định)
  (void)ADC_ReadRaw_Blocking(hadc, 10);

  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += ADC_ReadRaw_Blocking(hadc, 10);
  }

  float vref_raw = (float)sum / (float)samples;

  if (vref_raw <= 0.0f) return 0.0f;

  // Công thức ST:
  // VDDA = 3.3V * VREFINT_CAL / VREFINT_DATA
  float vdda = VDDA_CALIB * ((float)VREFINT_CAL / vref_raw);
  return vdda;
}

/*
  3) Đọc điện áp kênh ngoài (Volts) với VDDA đã đo
  - channel: ví dụ ADC_CHANNEL_0, ADC_CHANNEL_1, ...
  - vdda: lấy từ ADC_MeasureVDDA_FromVrefint()
*/
float ADC_ReadChannelVoltage_WithVDDA(ADC_HandleTypeDef *hadc,
                                      uint32_t channel,
                                      float vdda,
                                      uint16_t samples,
                                      uint32_t sampleTime)
{
  if (samples == 0) samples = 1;
  if (vdda <= 0.0f) return 0.0f;

  if (ADC_SetChannel(hadc, channel, sampleTime) != HAL_OK) {
    return 0.0f;
  }

  adc_short_delay();

  // bỏ 1 mẫu đầu
  (void)ADC_ReadRaw_Blocking(hadc, 10);

  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += ADC_ReadRaw_Blocking(hadc, 10);
  }

  float raw_avg = (float)sum / (float)samples;
  float v = (raw_avg / ADC_MAX_12BIT) * vdda;
  return v;
}

/*
  4) Tiện ích: đọc raw kênh ngoài (nếu cần)
*/
uint16_t ADC_ReadChannelRaw_Avg(ADC_HandleTypeDef *hadc,
                                uint32_t channel,
                                uint16_t samples,
                                uint32_t sampleTime)
{
  if (samples == 0) samples = 1;

  if (ADC_SetChannel(hadc, channel, sampleTime) != HAL_OK) {
    return 0;
  }

  adc_short_delay();
  (void)ADC_ReadRaw_Blocking(hadc, 10);

  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += ADC_ReadRaw_Blocking(hadc, 10);
  }

  return (uint16_t)(sum / samples);
}
