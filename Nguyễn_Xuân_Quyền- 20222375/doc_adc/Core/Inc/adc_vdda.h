/*
 * adc_vdda.h
 *
 *  Created on: Feb 3, 2026
 *      Author: quyen
 */

#ifndef INC_ADC_VDDA_H_
#define INC_ADC_VDDA_H_
#pragma once
#include "stm32f4xx_hal.h"

void  ADC_EnableVrefint(void);

float ADC_MeasureVDDA_FromVrefint(ADC_HandleTypeDef *hadc,
                                  uint16_t samples,
                                  uint32_t sampleTime);

float ADC_ReadChannelVoltage_WithVDDA(ADC_HandleTypeDef *hadc,
                                      uint32_t channel,
                                      float vdda,
                                      uint16_t samples,
                                      uint32_t sampleTime);

uint16_t ADC_ReadChannelRaw_Avg(ADC_HandleTypeDef *hadc,
                                uint32_t channel,
                                uint16_t samples,
                                uint32_t sampleTime);


#endif /* INC_ADC_VDDA_H_ */
