/*
 * com_task.h
 */

#ifndef COM_TASK_H_
#define COM_TASK_H_

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "errno.h"

typedef struct _callback_message_t
{
	status_t async_status;
	SemaphoreHandle_t sem;
} callback_message_t;

TaskHandle_t spi_task_handle;
void generate_irq(uint8_t irq);
void clear_irq_flag(uint8_t irq);
void spi_task(void *pvParameters);

#define BIT(nr)                 (1UL << (nr))


/* Commands and registers used in SPI communication */

/* Commands*/
#define APALIS_TK1_K20_READ_INST		0x0F
#define APALIS_TK1_K20_WRITE_INST		0xF0
#define APALIS_TK1_K20_BULK_WRITE_INST		0x3C
#define APALIS_TK1_K20_BULK_READ_INST		0xC3

#define APALIS_TK1_K20_MAX_BULK			(64)

/* General registers*/
#define APALIS_TK1_K20_STAREG			0x00 /* General status register RO */
#define APALIS_TK1_K20_REVREG			0x01 /* FW revision register RO*/
#define APALIS_TK1_K20_IRQREG			0x02 /* IRQ status RW(write of 1 will reset the bit) */
#define APALIS_TK1_K20_CTRREG			0x03 /* General control register RW */
#define APALIS_TK1_K20_MSQREG			0x04 /* IRQ mask register RW */

/* CAN Registers */
#define APALIS_TK1_K20_CANREG			0x10 /* CAN control & status register RW */
#define APALIS_TK1_K20_CAN_BAUD_REG		0x11 /* CAN Baud set register RW */
#define APALIS_TK1_K20_CAN_IN_BUF_CNT		0x12 /* CAN IN BUF Received Data Count RO */
#define APALIS_TK1_K20_CAN_IN_BUF		0x13 /* CAN IN BUF RO */
#define APALIS_TK1_K20_CAN_OUT_BUF_CNT		0x14 /* CAN OUT BUF  Data Count WO, must be written before bulk write to APALIS_TK1_K20_CAN0_OUT_BUF_CNT */
#define APALIS_TK1_K20_CAN_OUT_FIF0		0x15 /* CAN OUT BUF WO */

#define APALIS_TK1_K20_CAN_DEV_OFFSET(x)	(x ? 0:0x10)

/* ADC Registers */
#define APALIS_TK1_K20_ADCREG			0x30 /* ADC control & status register RW */
#define APALIS_TK1_K20_ADC_CH0L			0x31 /* ADC Channel 0 LSB RO */
#define APALIS_TK1_K20_ADC_CH0H			0x32 /* ADC Channel 0 MSB RO */
#define APALIS_TK1_K20_ADC_CH1L			0x33 /* ADC Channel 1 LSB RO */
#define APALIS_TK1_K20_ADC_CH1H			0x34 /* ADC Channel 1 MSB RO */
#define APALIS_TK1_K20_ADC_CH2L			0x35 /* ADC Channel 2 LSB RO */
#define APALIS_TK1_K20_ADC_CH2H			0x36 /* ADC Channel 2 MSB RO */
#define APALIS_TK1_K20_ADC_CH3L			0x37 /* ADC Channel 3 LSB RO */
#define APALIS_TK1_K20_ADC_CH3H			0x38 /* ADC Channel 3 MSB RO */
/* Bulk read of LSB register can be use to read entire 16-bit in one command */

/* TSC Register */
#define APALIS_TK1_K20_TSCREG			0x40 /* TSC control & status register RW */
#define APALIS_TK1_K20_TSC_XML			0x41 /* TSC X- data LSB RO */
#define APALIS_TK1_K20_TSC_XMH			0x42 /* TSC X- data MSB RO */
#define APALIS_TK1_K20_TSC_XPL			0x43 /* TSC X+ data LSB RO */
#define APALIS_TK1_K20_TSC_XPH			0x44 /* TSC X+ data MSB RO */
#define APALIS_TK1_K20_TSC_YML			0x45 /* TSC Y- data LSB RO */
#define APALIS_TK1_K20_TSC_YMH			0x46 /* TSC Y- data MSB RO */
#define APALIS_TK1_K20_TSC_YPL			0x47 /* TSC Y+ data LSB RO */
#define APALIS_TK1_K20_TSC_YPH			0x48 /* TSC Y+ data MSB RO */
/* Bulk read of LSB register can be use to read entire 16-bit in one command */
#define APALIS_TK1_K20_TSC_ENA			BIT(0)
#define APALIS_TK1_K20_TSC_ENA_MASK		0xFE

/* GPIO Registers */
#define APALIS_TK1_K20_GPIOREG			0x50 /* GPIO control & status register RW */
#define APALIS_TK1_K20_GPIO_NO			0x51 /* currently configured GPIO RW */
#define APALIS_TK1_K20_GPIO_STA			0x52 /* Status register for the APALIS_TK1_K20_GPIO_NO GPIO RW */
/* MSB | 0 ... 0 | VALUE | Output-1 / Input-0 | LSB  */
#define APALIS_TK1_K20_GPIO_STA_OE		BIT(0)
#define APALIS_TK1_K20_GPIO_STA_VAL		BIT(1)

/* Interrupt flags */
#define APALIS_TK1_K20_GEN_IRQ			0
#define APALIS_TK1_K20_CAN0_IRQ			1
#define APALIS_TK1_K20_CAN1_IRQ			2
#define APALIS_TK1_K20_ADC_IRQ			3
#define APALIS_TK1_K20_TSC_IRQ			4
#define APALIS_TK1_K20_GPIO_IRQ			5

#define APALIS_TK1_K20_FW_VER			0x05

#define FW_MINOR (APALIS_TK1_K20_FW_VER & 0x0F)
#define FW_MAJOR ((APALIS_TK1_K20_FW_VER & 0xF0) >> 8)

#define TK1_K20_SENTINEL			0x55
#define TK1_K20_INVAL				0xAA

#define ADC0_CHANNEL_CNT	4

#define TSC0_CHANNEL_CNT	4

/* Structure with general registers */
struct register_struct {
	uint8_t  status;
	uint8_t  control;
	uint8_t  irq_mask;
	uint8_t  irq;
	uint8_t  gpio_no;
	uint16_t adc[ADC0_CHANNEL_CNT];
	uint16_t tsc_xm;
	uint16_t tsc_xp;
	uint16_t tsc_ym;
	uint16_t tsc_yp;
} gen_regs;

#endif /* COM_TASK_H_ */
