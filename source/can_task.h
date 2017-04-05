/*
 * can_task.h
 */

#ifndef SOURCE_CAN_TASK_H_
#define SOURCE_CAN_TASK_H_

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t can_task_handle;
void can_test_task(void *pvParameters);
int can0_registers(uint8_t *rx_buf, uint8_t *tx_buf);
int can1_registers(uint8_t *rx_buf, uint8_t *tx_buf);

#endif /* SOURCE_CAN_TASK_H_ */
