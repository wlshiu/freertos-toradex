
#include "com_task.h"
#include "gpio_ext.h"
#include "adc_task.h"

/* Put FW version at known address in binary. Make it 32-bit to have room for the future*/
const uint32_t __attribute__((section(".FwVersion"))) fw_version = APALIS_TK1_K20_FW_VER;

#define MAX_TRANSFER_SIZE 32U
static dspi_slave_handle_t spi_handle;
static uint8_t slaveRxData[MAX_TRANSFER_SIZE] = {0U};
static uint8_t slaveTxData[MAX_TRANSFER_SIZE] = {0U};

void generate_irq(uint8_t irq) {
	gen_regs.irq = gen_regs.irq | BIT(irq);
	/* Toggle INT1 pin */
	GPIO_TogglePinsOutput(GPIOA, 1u << 16u);
}

void clear_irq_flag(uint8_t irq) {
	gen_regs.irq = ~irq & gen_regs.irq;
}

uint8_t get_control_reg()
{
	return gen_regs.control;

}

void set_control_reg(uint8_t value)
{
	gen_regs.control = value;
}

uint8_t get_status_reg()
{
	return gen_regs.status;

}

void set_status_reg(uint8_t value)
{
	gen_regs.status = value;
}

uint8_t get_mask_reg()
{
	return gen_regs.irq_mask;

}

void set_mask_reg(uint8_t value)
{
	gen_regs.irq_mask = value;
}

uint8_t get_irq_reg()
{
	return gen_regs.irq;

}

void set_irq_reg(uint8_t value)
{
	/* Clear IRQ flag on 1 */
	clear_irq_flag(value);

}

int inline general_registers(uint8_t *rx_buf, uint8_t * tx_buf) {

	if (rx_buf[0] == APALIS_TK1_K20_READ_INST) {
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_STAREG:
			tx_buf[0] = get_status_reg();
			return 1;
		case APALIS_TK1_K20_REVREG:
			tx_buf[0] = APALIS_TK1_K20_FW_VER;
			return 1;
		case APALIS_TK1_K20_IRQREG:
			tx_buf[0] = get_irq_reg();
			return 1;
		case APALIS_TK1_K20_CTRREG:
			tx_buf[0] = get_control_reg();
			return 1;
		case APALIS_TK1_K20_MSQREG:
			tx_buf[0] = get_mask_reg();
			return 1;
		default:
			return -ENOENT;
		}
	} else if (rx_buf[0] == APALIS_TK1_K20_WRITE_INST) {
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_STAREG:
			set_status_reg(rx_buf[2]);
			return 0;
		case APALIS_TK1_K20_REVREG:
			return -ENOENT;
		case APALIS_TK1_K20_IRQREG:
			set_irq_reg(rx_buf[2]);
			return 0;
		case APALIS_TK1_K20_CTRREG:
			set_control_reg(rx_buf[2]);
			return 0;
		case APALIS_TK1_K20_MSQREG:
			set_mask_reg(rx_buf[2]);
			return 0;
		default:
			return -ENOENT;
		}
	}


	return -ENOENT;
}

static void SPI_callback(SPI_Type *base, dspi_slave_handle_t *handle, status_t status, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

	if (status == kStatus_Success)
	{
		xSemaphoreGiveFromISR(cb->sem, &reschedule);
	}

	if (status == kStatus_DSPI_Error)
	{
		__NOP();
	}
	portYIELD_FROM_ISR(reschedule);
}

static void SPI_init() {
	dspi_slave_config_t slaveConfig;
	/* Slave config */
	slaveConfig.whichCtar = kDSPI_Ctar0;
	slaveConfig.ctarConfig.bitsPerFrame = 8;
	slaveConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	slaveConfig.ctarConfig.cpha = kDSPI_ClockPhaseSecondEdge;
	slaveConfig.enableContinuousSCK = true;
	slaveConfig.enableRxFifoOverWrite = false;
	slaveConfig.enableModifiedTimingFormat = false;
	slaveConfig.samplePoint = kDSPI_SckToSin0Clock;

	DSPI_SlaveInit(SPI2, &slaveConfig);
	DSPI_SlaveTransferCreateHandle(SPI2, &spi_handle, SPI_callback, spi_handle.userData);

	/* Set dspi slave interrupt priority higher. */
	NVIC_SetPriority(SPI2_IRQn, 5U);
	GPIO_ClearPinsOutput(GPIOA, 1u << 29u); /* INT2 active */
	PRINTF("SPI init done \r\n");

}

void spi_task(void *pvParameters) {
	callback_message_t cb_msg;
	dspi_transfer_t slaveXfer;
	int ret;

	cb_msg.sem = xSemaphoreCreateBinary();
	spi_handle.userData = &cb_msg;
	SPI_init();
	GPIO_SetPinsOutput(GPIOA, 1u << 29u); /* INT2 idle */

	while(1){
		slaveXfer.txData = slaveTxData;
		slaveXfer.rxData = slaveRxData;
		slaveXfer.dataSize = 3;
		slaveXfer.configFlags = kDSPI_SlaveCtar0;
		/* Wait for instructions from SoC */
		DSPI_SlaveTransferNonBlocking(SPI2, &spi_handle, &slaveXfer);
		xSemaphoreTake(cb_msg.sem, portMAX_DELAY);
		if (slaveRxData[1] <= 0x05) {
			ret = general_registers(slaveRxData, &slaveTxData[1]);
#ifdef BOARD_USES_ADC
		} else if ((slaveRxData[1] >= APALIS_TK1_K20_ADCREG) && (slaveRxData[1] <= APALIS_TK1_K20_ADC_CH3H)) {
			ret = adc_registers(slaveRxData, &slaveTxData[1]);

		} else if ((slaveRxData[1] >= APALIS_TK1_K20_TSCREG) && (slaveRxData[1] <= APALIS_TK1_K20_TSC_YPH)) {
			ret = tsc_registers(slaveRxData, &slaveTxData[1]);
#endif
		} else if ((slaveRxData[1] >= APALIS_TK1_K20_GPIOREG) && (slaveRxData[1] <= APALIS_TK1_K20_GPIO_STA)) {
			ret = gpio_registers(slaveRxData, &slaveTxData[1]);

		} else {
			/* Register not defined */
			ret = -EINVAL;
		}

		if (ret < 0) {
			slaveTxData[0] = TK1_K20_INVAL;
			slaveTxData[1] = TK1_K20_INVAL;
		} else {
			slaveTxData[0] = TK1_K20_SENTINEL;
		}

		if (slaveRxData[0] == APALIS_TK1_K20_READ_INST || slaveRxData[0] == APALIS_TK1_K20_BULK_READ_INST)
		{
			slaveXfer.dataSize = (ret >= 0) ? (ret + 1):2; /* Extra byte is for sentinel */
			DSPI_SlaveTransferNonBlocking(SPI2, &spi_handle, &slaveXfer);
			xSemaphoreTake(cb_msg.sem, portMAX_DELAY);
		}
	}
}
