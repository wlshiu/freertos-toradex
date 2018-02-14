
#include "can_task.h"
#include "com_task.h"
#include "fsl_flexcan.h"

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

#define TX_MESSAGE_BUFFER_NUM0 (9)

#define CAN_FRAME_MAX_LEN	8
#define CAN_HEADER_MAX_LEN	5
#define CAN_TRANSFER_BUF_LEN	(CAN_HEADER_MAX_LEN + CAN_FRAME_MAX_LEN)

#define CAN_CTRLMODE_NORMAL		0x00	/* normal mode */
#define CAN_CTRLMODE_LOOPBACK		0x01	/* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY		0x02 	/* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES		0x04	/* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT		0x08	/* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING	0x10	/* Bus-error reporting */

#define CANCTRL_MODMASK	0x03
#define CANCTRL_INTMASK	0x38
#define CANCTRL_INTEN	BIT(2)
#define CANINTF_RX	BIT(3)
#define CANINTF_TX	BIT(4)
#define CANINTF_ERR	BIT(5)

#define EFLG_EWARN	0x01
#define EFLG_RXWAR	0x02
#define EFLG_TXWAR	0x04
#define EFLG_RXEP	0x08
#define EFLG_TXEP	0x10
#define EFLG_TXBO	0x20
#define EFLG_RXOVR	0x40

static flexcan_handle_t flexcanHandle[2];
static volatile bool txComplete[2] = {false, false};
static volatile bool rxComplete[2] = {false, false};

struct can_task {
	uint8_t id;
	flexcan_frame_t rx_frame;
};

struct can_registers {
	uint8_t can_status_reg;
	uint8_t can_err_reg;
	uint8_t can_mode;
	uint8_t recived_frames_cnt;
	uint8_t rx_buf_top;
	uint8_t rx_buf_bottom;
};

static flexcan_frame_t tx_frame[2];
static uint8_t data_buffer[2][APALIS_TK1_CAN_RX_BUF_SIZE][CAN_TRANSFER_BUF_LEN];

static struct can_registers can_regs[2];
uint8_t resume_can;

void generate_can_irq(uint8_t id) {
	if (id == 0)
		GPIO_TogglePinsOutput(GPIOB, 1u << 8u);
	else
		GPIO_TogglePinsOutput(GPIOE, 1u << 26u);
}

static void flexcan_callback0(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

	switch (status)
	{
	case kStatus_FLEXCAN_RxFifoIdle:
		cb->async_status = pdTRUE;
		xSemaphoreGiveFromISR(cb->sem, &reschedule);
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM0 == result)
		{
			/* write status and gen irq */
			registers[APALIS_TK1_K20_CANREG] |= CANINTF_TX;
			generate_can_irq(0);
		}
		break;

	default:
		break;
	}
	portYIELD_FROM_ISR(reschedule);
}

static void flexcan_callback1(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

	switch (status)
	{
	case kStatus_FLEXCAN_RxFifoIdle:
		cb->async_status = pdTRUE;
		xSemaphoreGiveFromISR(cb->sem, &reschedule);
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM0 == result)
		{
			/* write status and gen irq */
			registers[APALIS_TK1_K20_CANREG + APALIS_TK1_K20_CAN_OFFSET] |= CANINTF_TX;
			generate_can_irq(1);
		}
		break;

	default:
		break;
	}
	portYIELD_FROM_ISR(reschedule);
}

static void CAN0_Init()
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_fifo_config_t fifoConfig;
	uint32_t fifoFilter = 0xFFFFFFFF;

	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	flexcanConfig.baudRate = 1000000U; /* set default to 1Mbit/s*/

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN0, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle[0], flexcan_callback0, flexcanHandle[0].userData);

	/* Set Rx Mask to don't care on all bits. */
	FLEXCAN_SetRxMbGlobalMask(CAN0, FLEXCAN_RX_MB_EXT_MASK(0x00, 0, 0));
	FLEXCAN_SetRxFifoGlobalMask(CAN0, FLEXCAN_RX_MB_EXT_MASK(0x00, 0, 0));

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM0, true);

	fifoConfig.idFilterNum = 1;
	fifoConfig.idFilterTable = &fifoFilter;
	fifoConfig.idFilterType = kFLEXCAN_RxFifoFilterTypeC;
	fifoConfig.priority = kFLEXCAN_RxFifoPrioHigh;
	FLEXCAN_SetRxFifoConfig(CAN0, &fifoConfig, true);

	PRINTF("CAN0 init done \r\n");
}

static void CAN1_Init()
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_fifo_config_t fifoConfig;
	uint32_t fifoFilter = 0xFFFFFFFF;

	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	flexcanConfig.baudRate = 1000000U; /* set default to 1Mbit/s */

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN1, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN1, &flexcanHandle[1], flexcan_callback1, flexcanHandle[1].userData);

	/* Set Rx Mask to don't care on all bits. */
	FLEXCAN_SetRxMbGlobalMask(CAN1, FLEXCAN_RX_MB_EXT_MASK(0x00, 0, 0));
	FLEXCAN_SetRxFifoGlobalMask(CAN1, FLEXCAN_RX_MB_EXT_MASK(0x00, 0, 0));

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN1, TX_MESSAGE_BUFFER_NUM0, true);

	fifoConfig.idFilterNum = 1;
	fifoConfig.idFilterTable = &fifoFilter;
	fifoConfig.idFilterType = kFLEXCAN_RxFifoFilterTypeC;
	fifoConfig.priority = kFLEXCAN_RxFifoPrioHigh;
	FLEXCAN_SetRxFifoConfig(CAN1, &fifoConfig, true);

	PRINTF("CAN1 init done \r\n");
}
uint8_t available_data[2];

void can_calculate_available_data(uint8_t id) {
	if ( can_regs[id].rx_buf_bottom <= can_regs[id].rx_buf_top)
		available_data[id] = can_regs[id].rx_buf_top - can_regs[id].rx_buf_bottom;
	else
		available_data[id] = APALIS_TK1_CAN_RX_BUF_SIZE - can_regs[id].rx_buf_bottom;

	available_data[id] = (available_data[id] > APALIS_TK1_MAX_CAN_DMA_XREF) ? APALIS_TK1_MAX_CAN_DMA_XREF:available_data[id];
	registers[APALIS_TK1_K20_CAN_IN_BUF_CNT + APALIS_TK1_K20_CAN_DEV_OFFSET(id)] = available_data[id];
	if (can_regs[id].rx_buf_bottom == can_regs[id].rx_buf_top)
		registers[APALIS_TK1_K20_CANREG + APALIS_TK1_K20_CAN_DEV_OFFSET(id)] &= ~CANINTF_RX;
}

void frame_to_buffer(flexcan_frame_t *frame, uint8_t id) {
	uint8_t top_frame = can_regs[id].rx_buf_top;
	if (frame->format ==  kFLEXCAN_FrameFormatExtend) {
		data_buffer[id][top_frame][0] = frame->id & 0xFF;
		data_buffer[id][top_frame][1] = (frame->id >> 8 ) & 0xFF;
		data_buffer[id][top_frame][2] = (frame->id >> 16 ) & 0xFF;
		data_buffer[id][top_frame][3] = (frame->id >> 24 ) & 0x1F;
		data_buffer[id][top_frame][3] |= CAN_EFF_FLAG >> 24;
	} else {
		data_buffer[id][top_frame][0] = (frame->id >> 18) & 0xFF;
		data_buffer[id][top_frame][1] = ((frame->id >> 18) >> 8 ) & 0x7F;
		data_buffer[id][top_frame][2] = 0x00;
		data_buffer[id][top_frame][3] = 0x00;
	}
	data_buffer[id][top_frame][3] |= frame->type ?  (CAN_RTR_FLAG >> 24):0x00;
	data_buffer[id][top_frame][4] = frame->length;

	switch (frame->length) {
	case 8:data_buffer[id][top_frame][5 + 7] = frame->dataByte7;
	case 7:data_buffer[id][top_frame][5 + 6] = frame->dataByte6;
	case 6:data_buffer[id][top_frame][5 + 5] = frame->dataByte5;
	case 5:data_buffer[id][top_frame][5 + 4] = frame->dataByte4;
	case 4:data_buffer[id][top_frame][5 + 3] = frame->dataByte3;
	case 3:data_buffer[id][top_frame][5 + 2] = frame->dataByte2;
	case 2:data_buffer[id][top_frame][5 + 1] = frame->dataByte1;
	case 1:data_buffer[id][top_frame][5] = frame->dataByte0;
	}
	can_regs[id].rx_buf_top++;
	can_regs[id].rx_buf_top %= APALIS_TK1_CAN_RX_BUF_SIZE;
	can_calculate_available_data(id);
}

void can0_task(void *pvParameters) {
	flexcan_frame_t  rxFrame;
	flexcan_fifo_transfer_t rxXfer;
	callback_message_t cb_msg;
	cb_msg.sem = xSemaphoreCreateBinary();
	cb_msg.async_status = pdFALSE;
	flexcanHandle[0].userData = (void *) &cb_msg;
	CAN0_Init();
	rxXfer.frame = &rxFrame;

	while(1)
	{
		FLEXCAN_TransferReceiveFifoNonBlocking(CAN0, &flexcanHandle[0], &rxXfer);
		if (xSemaphoreTake(cb_msg.sem, portMAX_DELAY) == pdTRUE) {
			if (cb_msg.async_status == pdTRUE) {
				cb_msg.async_status = pdFALSE;
				frame_to_buffer(&rxFrame, 0);
				can_regs[0].recived_frames_cnt++;
				registers[APALIS_TK1_K20_CANREG] |= CANINTF_RX;
				generate_can_irq(0);
			}

			if (can_regs[0].recived_frames_cnt >= APALIS_TK1_CAN_RX_BUF_SIZE)
				vTaskSuspend(NULL);
		}

	}
	vSemaphoreDelete(cb_msg.sem);
}

void can1_task(void *pvParameters) {
	flexcan_frame_t  rxFrame;
	flexcan_fifo_transfer_t rxXfer;
	callback_message_t cb_msg;

	cb_msg.sem = xSemaphoreCreateBinary();
	flexcanHandle[1].userData = (void *) &cb_msg;
	CAN1_Init();
	rxXfer.frame = &rxFrame;

	while(1)
	{
		FLEXCAN_TransferReceiveFifoNonBlocking(CAN1, &flexcanHandle[1], &rxXfer);
		if (xSemaphoreTake(cb_msg.sem, portMAX_DELAY) == pdTRUE) {
			if (cb_msg.async_status == pdTRUE) {
				cb_msg.async_status = pdFALSE;
				frame_to_buffer(&rxFrame, 1);
				can_regs[1].recived_frames_cnt++;
				registers[APALIS_TK1_K20_CANREG + APALIS_TK1_K20_CAN_OFFSET] |= CANINTF_RX;
				generate_can_irq(1);
			}

			if (can_regs[1].recived_frames_cnt >= APALIS_TK1_CAN_RX_BUF_SIZE) {
				vTaskSuspend(NULL);
			}
		}
	}
	vSemaphoreDelete(cb_msg.sem);

}

static void can_change_mode(int id, uint8_t new_mode)
{
	CAN_Type *base = id ? CAN1:CAN0;

	can_regs[id].can_mode = new_mode;

	switch (new_mode){
	case CAN_CTRLMODE_LOOPBACK:
		base->CTRL1 = base->CTRL1 | CAN_CTRL1_LPB_MASK;
		break;
	case CAN_CTRLMODE_NORMAL:
		base->CTRL1 = base->CTRL1 & ~CAN_CTRL1_LPB_MASK;
	}

}

uint8_t set_canreg (int id, uint8_t value)
{
	registers[APALIS_TK1_K20_CANREG + APALIS_TK1_K20_CAN_DEV_OFFSET(id)] = value;
	if ( can_regs[id].can_mode != (value & CANCTRL_MODMASK) )
		can_change_mode(id, (value & CANCTRL_MODMASK));
	return 1;
}

uint8_t clr_canreg (int id, uint8_t mask)
{
	mask &= (CANINTF_TX | CANINTF_ERR);
	registers[APALIS_TK1_K20_CANREG + APALIS_TK1_K20_CAN_DEV_OFFSET(id)] &= ~mask;
	return 1;
}

uint8_t set_canerr (int id, uint8_t value)
{
	return 1;
}

uint8_t set_canbadreg (int id, uint8_t value)
{

	FLEXCAN_SetBitRate(id ? CAN1:CAN0, CLOCK_GetFreq(kCLOCK_BusClk), value * APALIS_TK1_CAN_CLK_UNIT);
	return 1;
}

#if 0
static flexcan_timing_config_t timingConfig;
#endif

uint8_t set_canbittimig (int id, uint16_t value, int16_t mask)
{
#if 0
	if(mask & 0xFF) {
		timingConfig.phaseSeg1 = 0;
		timingConfig.phaseSeg2 = 0;
		timingConfig.rJumpwidth = 0;
	}
	if (mask & 0xFF00) {
		timingConfig.propSeg = 0;
		timingConfig.preDivider = 0;

	}
#endif
	return 1;
}

uint8_t can0_transmit(){
	flexcan_mb_transfer_t txXfer;

	txXfer.frame = &tx_frame[0];
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM0;

	FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle[0], &txXfer);
	return 0;
}

uint8_t can1_transmit(){
	flexcan_mb_transfer_t txXfer;

	txXfer.frame = &tx_frame[1];
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM0;

	FLEXCAN_TransferSendNonBlocking(CAN1, &flexcanHandle[1], &txXfer);
	return 0;
}

uint8_t can_sendframe(uint8_t id, uint8_t *data, uint8_t len)
{
	tx_frame[id].length = data[4];
	tx_frame[id].id = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];

	tx_frame[id].format = (data[3] << 24 & CAN_EFF_FLAG) ?
				kFLEXCAN_FrameFormatExtend:kFLEXCAN_FrameFormatStandard;
	tx_frame[id].type = (data[3] << 24 & CAN_RTR_FLAG) ?
				kFLEXCAN_FrameTypeRemote:kFLEXCAN_FrameTypeData;

	if (tx_frame[id].format == kFLEXCAN_FrameFormatExtend)
		tx_frame[id].id = FLEXCAN_ID_EXT(tx_frame[id].id);
	else
		tx_frame[id].id = FLEXCAN_ID_STD(tx_frame[id].id);

	tx_frame[id].dataByte0 = data[5];
	tx_frame[id].dataByte1 = data[5 + 1];
	tx_frame[id].dataByte2 = data[5 + 2];
	tx_frame[id].dataByte3 = data[5 + 3];
	tx_frame[id].dataByte4 = data[5 + 4];
	tx_frame[id].dataByte5 = data[5 + 5];
	tx_frame[id].dataByte6 = data[5 + 6];
	tx_frame[id].dataByte7 = data[5 + 7];
	if (id == 0)
		can0_transmit();
	else if (id == 1)
		can1_transmit();
	return 0;
}

uint16_t can_readframe(uint8_t id, dspi_transfer_t *spi_transfer)
{
	uint8_t rx_size;
	spi_transfer->txData = data_buffer[id][can_regs[id].rx_buf_bottom];
	rx_size = spi_transfer->rxData[2] / CAN_TRANSFER_BUF_LEN; /* size in frames, not bytes */
	if (rx_size > available_data[id])
		rx_size = available_data[id];
	can_regs[id].rx_buf_bottom = can_regs[id].rx_buf_bottom + rx_size;
	can_regs[id].rx_buf_bottom %= APALIS_TK1_CAN_RX_BUF_SIZE;

	resume_can = id + 1;

	can_regs[id].recived_frames_cnt -= rx_size;
	can_calculate_available_data(id);

	return rx_size * CAN_TRANSFER_BUF_LEN;
}

int canx_registers(dspi_transfer_t *spi_transfer, int id)
{
	uint8_t *rx_buf = spi_transfer->rxData;
	if (rx_buf[0] == APALIS_TK1_K20_WRITE_INST) {
		rx_buf[1] -= APALIS_TK1_K20_CAN_DEV_OFFSET(id);
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_CANREG:
			return set_canreg(id, rx_buf[2]);
		case APALIS_TK1_K20_CANREG_CLR:
			return clr_canreg(id, rx_buf[2]);
		case APALIS_TK1_K20_CANERR:
			return set_canerr(id, rx_buf[2]);
		case APALIS_TK1_K20_CAN_BAUD_REG:
			return set_canbadreg(id, rx_buf[2]);
		case APALIS_TK1_K20_CAN_BIT_1:
			return set_canbittimig(id, rx_buf[2], 0x00FF);
		case APALIS_TK1_K20_CAN_BIT_2:
			return set_canbittimig(id, (rx_buf[2] << 8), 0xFF00);
		default:
			return -EIO;
		}

	} else if (rx_buf[0] == APALIS_TK1_K20_BULK_READ_INST) {
		rx_buf[1] -= APALIS_TK1_K20_CAN_DEV_OFFSET(id);
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_CAN_IN_BUF:
			return can_readframe(id, spi_transfer);
		default:
			return -EIO;
		}
	} else if (rx_buf[0] == APALIS_TK1_K20_BULK_WRITE_INST) {
		rx_buf[1] -= APALIS_TK1_K20_CAN_DEV_OFFSET(id);
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_CAN_OUT_BUF:
			can_sendframe(id, &rx_buf[3], rx_buf[2]);
			break;
		default:
			return -EIO;
		}
	}
	return -EIO;
}

