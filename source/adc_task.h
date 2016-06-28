/*
 * adc_task.h
 *
 */

#ifndef SOURCE_ADC_TASK_H_
#define SOURCE_ADC_TASK_H_
#include "board.h"

#ifdef BOARD_USES_ADC
TaskHandle_t adc_task_handle;
TaskHandle_t tsc_task_handle;
void adc_task(void *pvParameters);
void tsc_task(void *pvParameters);
int adc_registers(uint8_t *rx_buf, uint8_t *tx_buf);
int tsc_registers(uint8_t *rx_buf, uint8_t *tx_buf);
#endif

#endif /* SOURCE_ADC_TASK_H_ */
