/*
 * can_task.h
 */

#ifndef SOURCE_CAN_TASK_H_
#define SOURCE_CAN_TASK_H_

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_dspi.h"

TaskHandle_t can_task_handle;
void can_task(void *pvParameters);
int can0_registers(dspi_transfer_t *spi_transfer);
int can1_registers(dspi_transfer_t *spi_transfer);

#endif /* SOURCE_CAN_TASK_H_ */
