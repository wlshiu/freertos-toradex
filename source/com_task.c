
#include "com_task.h"
#include "can_task.h"
#include "gpio_ext.h"
#include "adc_task.h"

/* Put FW version at known address in binary. Make it 32-bit to have room for the future*/
const uint32_t __attribute__((section(".FwVersion"), used)) fw_version = APALIS_TK1_K20_FW_VER;

static dspi_slave_handle_t spi_handle;
static uint8_t slaveRxData[APALIS_TK1_K20_MAX_BULK + APALIS_TK1_K20_HEADER] = {0U};
static uint8_t slaveTxData[APALIS_TK1_K20_MAX_BULK + APALIS_TK1_K20_HEADER] = {0U};

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

inline int general_registers(dspi_transfer_t * spi_transfer)
{
	uint8_t *rx_buf = spi_transfer->rxData;
	uint8_t *tx_buf = &spi_transfer->txData[1];

	if (rx_buf[0] == APALIS_TK1_K20_READ_INST) {
		switch (rx_buf[2]) {
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
			return 1;
		case APALIS_TK1_K20_REVREG:
			return -ENOENT;
		case APALIS_TK1_K20_IRQREG:
			set_irq_reg(rx_buf[2]);
			return 1;
		case APALIS_TK1_K20_CTRREG:
			set_control_reg(rx_buf[2]);
			return 1;
		case APALIS_TK1_K20_MSQREG:
			set_mask_reg(rx_buf[2]);
			return 1;
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

static dspi_slave_config_t spi2_slaveConfig;

static void SPI_init() {
	gpio_pin_config_t gpio_out_config = {
			kGPIO_DigitalOutput, 0,
	};
	GPIO_PinInit(GPIOD, 11u, &gpio_out_config);
	/* Slave config */
	spi2_slaveConfig.whichCtar = kDSPI_Ctar0;
	spi2_slaveConfig.ctarConfig.bitsPerFrame = 8;
	spi2_slaveConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	spi2_slaveConfig.ctarConfig.cpha = kDSPI_ClockPhaseSecondEdge;
	spi2_slaveConfig.enableContinuousSCK = false;
	spi2_slaveConfig.enableRxFifoOverWrite = true;
	spi2_slaveConfig.enableModifiedTimingFormat = false;
	spi2_slaveConfig.samplePoint = kDSPI_SckToSin0Clock;
	PRINTF("SPI init \r\n");
	DSPI_SlaveInit(SPI2, &spi2_slaveConfig);
	DSPI_SlaveTransferCreateHandle(SPI2, &spi_handle, SPI_callback, spi_handle.userData);

	/* Set dspi slave interrupt priority higher. */
	NVIC_SetPriority(SPI2_IRQn, 5U);
	GPIO_ClearPinsOutput(GPIOA, 1u << 29u); /* INT2 active */
	PRINTF("SPI init done \r\n");

}

void spi_task(void *pvParameters) {
	callback_message_t cb_msg;
	dspi_transfer_t slaveXfer;
	int ret, retry_size = 0;
	uint8_t req_register = 0xFF;

	cb_msg.sem = xSemaphoreCreateBinary();
	spi_handle.userData = &cb_msg;
	SPI_init();
	GPIO_SetPinsOutput(GPIOA, 1u << 29u); /* INT2 idle */

	while(1){
		slaveXfer.txData = NULL;/* We're not expecting any MISO traffic */
		slaveXfer.rxData = slaveRxData;
		slaveXfer.dataSize = 3;
		slaveXfer.configFlags = kDSPI_SlaveCtar0;
		/* Wait for instructions from SoC */
		DSPI_SlaveTransferNonBlocking(SPI2, &spi_handle, &slaveXfer);
		xSemaphoreTake(cb_msg.sem, portMAX_DELAY);
		GPIO_ClearPinsOutput(GPIOD, 1u << 11u);
		slaveXfer.txData = slaveTxData;
		slaveXfer.rxData = slaveRxData;

		if (slaveRxData[0] == APALIS_TK1_K20_WRITE_INST)
			req_register = slaveRxData[1];
		else
			req_register = slaveRxData[2];

		if (req_register <= 0x05) {
			ret = general_registers(&slaveXfer);
		} else if ((req_register >= APALIS_TK1_K20_CANREG + APALIS_TK1_K20_CAN_DEV_OFFSET(0))
			&& (req_register <= APALIS_TK1_K20_CAN_OUT_BUF_END + APALIS_TK1_K20_CAN_DEV_OFFSET(0))) {
			ret = canx_registers(&slaveXfer, 0);

		} else if ((req_register >= APALIS_TK1_K20_CANREG + APALIS_TK1_K20_CAN_DEV_OFFSET(1))
			&& (req_register <= APALIS_TK1_K20_CAN_OUT_BUF_END + APALIS_TK1_K20_CAN_DEV_OFFSET(1))) {
			ret = canx_registers(&slaveXfer, 1);
#ifdef BOARD_USES_ADC
		} else if ((req_register >= APALIS_TK1_K20_ADCREG) && (req_register <= APALIS_TK1_K20_ADC_CH3H)) {
			ret = adc_registers(&slaveXfer);

		} else if ((req_register >= APALIS_TK1_K20_TSCREG) && (req_register <= APALIS_TK1_K20_TSC_YPH)) {
			ret = tsc_registers(&slaveXfer);
#endif
		} else if ((req_register >= APALIS_TK1_K20_GPIOREG) && (req_register <= APALIS_TK1_K20_GPIO_STA)) {
			ret = gpio_registers(&slaveXfer);

		} else if (req_register == APALIS_TK1_K20_RET_REQ) {
			/* something wrong with the SPI peripheral, try resetting it */
			DSPI_StopTransfer(SPI2);
			DSPI_Enable(SPI2, false);
			DSPI_SlaveInit(SPI2, &spi2_slaveConfig);
			ret = retry_size - 1;
		} else {
			/* Register not defined */
			ret = -EINVAL;
		}

		if (ret <= 0) {
			slaveTxData[0] = TK1_K20_INVAL;
			slaveTxData[1] = TK1_K20_INVAL;
		} else {
			slaveTxData[0] = TK1_K20_SENTINEL;
		}

		if (slaveRxData[0] == APALIS_TK1_K20_READ_INST || slaveRxData[0] == APALIS_TK1_K20_BULK_READ_INST)
		{
			slaveXfer.dataSize = (ret > 0) ? (ret + 1):2; /* Extra byte is for sentinel*/
			retry_size = slaveXfer.dataSize;
			slaveXfer.rxData = NULL; /* We're not expecting any MOSI traffic */
			DSPI_SlaveTransferNonBlocking(SPI2, &spi_handle, &slaveXfer);
			GPIO_SetPinsOutput(GPIOD, 1u << 11u);
			xSemaphoreTake(cb_msg.sem, portMAX_DELAY);
		}
	}
}
