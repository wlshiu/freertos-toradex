/*
 * can_task.h
 */

#ifndef SOURCE_CAN_TASK_H_
#define SOURCE_CAN_TASK_H_

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_dspi.h"

TaskHandle_t can0_task_handle;
TaskHandle_t can1_task_handle;
void can0_task(void *pvParameters);
void can1_task(void *pvParameters);
int canx_registers(dspi_transfer_t *spi_transfer, int id);
void can_calculate_available_data(uint8_t id);

#endif /* SOURCE_CAN_TASK_H_ */
