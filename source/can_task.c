
#include "can_task.h"
#include "com_task.h"
#include "fsl_flexcan.h"

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

#define RX_MESSAGE_BUFFER_NUM0 (0)
#define TX_MESSAGE_BUFFER_NUM0 (1)

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
};

static flexcan_frame_t tx_frame[2];
static flexcan_frame_t rx_buffer[2][APALIS_TK1_CAN_RX_BUF_SIZE];

static struct can_registers can_regs[2];

static void flexcan_callback0(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

	switch (status)
	{
	case kStatus_FLEXCAN_RxIdle:
		if (RX_MESSAGE_BUFFER_NUM0 == result)
		{
			cb->async_status = pdTRUE;
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
		}
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM0 == result)
		{
			/* write status and gen irq */
			can_regs[0].can_status_reg |= CANINTF_TX;
			generate_irq(APALIS_TK1_K20_CAN0_IRQ);
			cb->async_status = pdFALSE;
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
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
	case kStatus_FLEXCAN_RxIdle:
		if (RX_MESSAGE_BUFFER_NUM0 == result)
		{
			cb->async_status = pdTRUE;
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
		}
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM0 == result)
		{
			/* write status and gen irq */
			can_regs[1].can_status_reg |= CANINTF_TX;
			generate_irq(APALIS_TK1_K20_CAN1_IRQ);
			cb->async_status = pdFALSE;
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
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
	flexcan_rx_mb_config_t mbConfig;

	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	flexcanConfig.baudRate = 1000000U; /* set default to 1Mbit/s*/
	flexcanConfig.maxMbNum = 2; /* Use only 2 mailboxes, 1 RX 1TX */

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN0, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle[0], flexcan_callback0, flexcanHandle[0].userData);

	/* Set Rx Mask to don't care on all bits. */
	FLEXCAN_SetRxMbGlobalMask(CAN0, FLEXCAN_RX_MB_EXT_MASK(0x00, 0, 0));

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatExtend;
	mbConfig.type = kFLEXCAN_FrameTypeData;
	mbConfig.id = FLEXCAN_ID_EXT(0x1FFFFFFF);
	FLEXCAN_SetRxMbConfig(CAN0, RX_MESSAGE_BUFFER_NUM0, &mbConfig, true);

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM0, true);
	PRINTF("CAN0 init done \r\n");
}

static void CAN1_Init()
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	flexcanConfig.baudRate = 1000000U; /* set default to 1Mbit/s */
	flexcanConfig.maxMbNum = 2; /* Use only 2 mailboxes, 1 RX 1TX */

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN1, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN1, &flexcanHandle[1], flexcan_callback1, flexcanHandle[1].userData);

	/* Set Rx Mask to don't care on all bits. */
	FLEXCAN_SetRxMbGlobalMask(CAN1, FLEXCAN_RX_MB_EXT_MASK(0x00, 0, 0));

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatExtend;
	mbConfig.type = kFLEXCAN_FrameTypeData;
	mbConfig.id = FLEXCAN_ID_EXT(0x1FFFFFFF);
	FLEXCAN_SetRxMbConfig(CAN1, RX_MESSAGE_BUFFER_NUM0, &mbConfig, true);

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN1, TX_MESSAGE_BUFFER_NUM0, true);
	PRINTF("CAN1 init done \r\n");
}

void can0_task(void *pvParameters) {
	flexcan_frame_t  rxFrame;
	flexcan_mb_transfer_t rxXfer;
	flexcan_rx_mb_config_t mbConfig;
	callback_message_t cb_msg;
	status_t ret;


	cb_msg.sem = xSemaphoreCreateBinary();
	flexcanHandle[0].userData = (void *) &cb_msg;
	CAN0_Init();
	mbConfig.format = kFLEXCAN_FrameFormatExtend;
	mbConfig.type = kFLEXCAN_FrameTypeData;
	mbConfig.id = FLEXCAN_ID_EXT(0x1FFFFFFF);

	while(1)
	{
		rxXfer.frame = &rxFrame;
		rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM0;

		FLEXCAN_SetRxMbConfig(CAN0, RX_MESSAGE_BUFFER_NUM0, &mbConfig, true);
		ret = FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle[0], &rxXfer);
		if (ret == kStatus_Success){
			if (xSemaphoreTake(cb_msg.sem, portMAX_DELAY) == pdTRUE)
			{
				if (cb_msg.async_status == pdTRUE)
				{
					if (can_regs[0].recived_frames_cnt < APALIS_TK1_CAN_RX_BUF_SIZE){

						memcpy(&rx_buffer[0][can_regs[0].recived_frames_cnt],
								rxXfer.frame, sizeof(flexcan_frame_t));
						can_regs[0].recived_frames_cnt++;
						can_regs[0].can_status_reg |= CANINTF_RX;
						generate_irq(APALIS_TK1_K20_CAN0_IRQ);
					}
					if (can_regs[0].recived_frames_cnt >= APALIS_TK1_CAN_RX_BUF_SIZE)
						vTaskSuspend(NULL);
				}
			}
		}else {
			FLEXCAN_TransferAbortReceive(CAN0, &flexcanHandle[0], RX_MESSAGE_BUFFER_NUM0);
			FLEXCAN_TransferAbortSend(CAN0, &flexcanHandle[0], TX_MESSAGE_BUFFER_NUM0);
		}
	}
	vSemaphoreDelete(cb_msg.sem);

}

void can1_task(void *pvParameters) {
	flexcan_frame_t  rxFrame;
	flexcan_mb_transfer_t rxXfer;
	flexcan_rx_mb_config_t mbConfig;
	callback_message_t cb_msg;
	status_t ret;

	cb_msg.sem = xSemaphoreCreateBinary();
	flexcanHandle[1].userData = (void *) &cb_msg;
	CAN1_Init();

	while(1)
	{
		rxXfer.frame = &rxFrame;
		rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM0;

		FLEXCAN_SetRxMbConfig(CAN1, RX_MESSAGE_BUFFER_NUM0, &mbConfig, true);
		ret = FLEXCAN_TransferReceiveNonBlocking(CAN1, &flexcanHandle[1], &rxXfer);
		if (ret == kStatus_Success){
		if (xSemaphoreTake(cb_msg.sem, portMAX_DELAY) == pdTRUE)
		{
			if (cb_msg.async_status == pdTRUE)
			{
				if (can_regs[1].recived_frames_cnt < APALIS_TK1_CAN_RX_BUF_SIZE){

					memcpy(&rx_buffer[1][can_regs[1].recived_frames_cnt],
							rxXfer.frame, sizeof(flexcan_frame_t));
					can_regs[1].recived_frames_cnt++;
					can_regs[1].can_status_reg |= CANINTF_RX;
					generate_irq(APALIS_TK1_K20_CAN1_IRQ);
				}
				if (can_regs[1].recived_frames_cnt >= APALIS_TK1_CAN_RX_BUF_SIZE)
					vTaskSuspend(NULL);
			}
		}
		}else {
			FLEXCAN_TransferAbortReceive(CAN1, &flexcanHandle[1], RX_MESSAGE_BUFFER_NUM0);
			FLEXCAN_TransferAbortSend(CAN1, &flexcanHandle[1], TX_MESSAGE_BUFFER_NUM0);
		}
	}
	vSemaphoreDelete(cb_msg.sem);
}

uint8_t get_canreg (int id)
{
	return can_regs[id].can_status_reg;
}

uint8_t get_canerr (int id)
{
	uint8_t temp;
	temp = can_regs[id].can_err_reg;
	can_regs[id].can_err_reg = 0;
	return temp;
}

uint8_t get_canbadreg (int id)
{
	return 0;
}

uint16_t get_canbittimig (int id)
{
	return 0;
}

uint8_t get_can_rxcnt(int id)
{
	return can_regs[id].recived_frames_cnt;
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
	can_regs[id].can_status_reg = value;
	if ( can_regs[id].can_mode != (value & CANCTRL_MODMASK) )
		can_change_mode(id, (value & CANCTRL_MODMASK));
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

static int can0_inframe;

uint8_t can0_transmit(){
	flexcan_mb_transfer_t txXfer;

	txXfer.frame = &tx_frame[0];
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM0;
	can0_inframe = 0;

	FLEXCAN_TransferAbortReceive(CAN0, &flexcanHandle[0], RX_MESSAGE_BUFFER_NUM0);
	FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle[0], &txXfer);
	return 0;
}

uint8_t can0_sendframe_single_rw(uint8_t address, uint8_t data)
{

	if (address == 0){
		tx_frame[0].length = data;
		can0_inframe = 1;
	}

	if ((address > 0) && (address < 5) && can0_inframe){
		tx_frame[0].id = data << (8 * (address - 1));
	}

	if ((address > 4) && can0_inframe){
		switch (address){
		case 5: tx_frame[0].dataByte0 = data;
			break;
		case 6: tx_frame[0].dataByte1 = data;
			break;
		case 7: tx_frame[0].dataByte2 = data;
			break;
		case 8: tx_frame[0].dataByte3 = data;
			break;
		case 9: tx_frame[0].dataByte4 = data;
			break;
		case 10: tx_frame[0].dataByte5 = data;
			break;
		case 11: tx_frame[0].dataByte6 = data;
			break;
		case 12: tx_frame[0].dataByte7 = data;
			break;
		}
		if ((address - 4) == tx_frame[0].length)
			return can0_transmit();
	}
	return 0;
}

uint8_t can1_sendframe(uint8_t *data, uint8_t len)
{
	return 1;
}

static int can1_inframe;

uint8_t can1_transmit(){
	flexcan_mb_transfer_t txXfer;

	txXfer.frame = &tx_frame[1];
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM0;
	can1_inframe = 0;

	FLEXCAN_TransferAbortReceive(CAN1, &flexcanHandle[1], RX_MESSAGE_BUFFER_NUM0);
	FLEXCAN_TransferSendNonBlocking(CAN1, &flexcanHandle[1], &txXfer);
	return 0;
}

uint8_t can1_sendframe_single_rw(uint8_t address, uint8_t data)
{

	if (address == 0){
		tx_frame[1].length = data;
		can1_inframe = 1;
	}

	if ((address > 0) && (address < 5) && can1_inframe){
		tx_frame[1].id = data << (8 * (address - 1));
	}

	if ((address > 4) && can1_inframe){
		switch (address){
		case 5: tx_frame[1].dataByte0 = data;
			break;
		case 6: tx_frame[1].dataByte1 = data;
			break;
		case 7: tx_frame[1].dataByte2 = data;
			break;
		case 8: tx_frame[1].dataByte3 = data;
			break;
		case 9: tx_frame[1].dataByte4 = data;
			break;
		case 10: tx_frame[1].dataByte5 = data;
			break;
		case 11: tx_frame[1].dataByte6 = data;
			break;
		case 12: tx_frame[1].dataByte7 = data;
			break;
		}
		if ((address - 4) == tx_frame[1].length)
			return can1_transmit();
	}
	return 0;
}

uint8_t can_sendframe(uint8_t id, uint8_t *data, uint8_t len)
{
	tx_frame[id].length = data[0];
	tx_frame[id].id = (data[4] << 24) + (data[3] << 16) + (data[2] << 8) + data[1];

	tx_frame[id].format = (data[4] << 24 & CAN_EFF_FLAG) ?
				kFLEXCAN_FrameFormatExtend:kFLEXCAN_FrameFormatStandard;
	tx_frame[id].type = (data[4] << 24 & CAN_RTR_FLAG) ?
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

uint8_t can_readframe(uint8_t id, uint8_t *data)
{
	data[0] = rx_buffer[id][0].length ;
	if (rx_buffer[id][0].format ==  kFLEXCAN_FrameFormatExtend) {
		data[1] = rx_buffer[id][0].id & 0xFF;
		data[2] = (rx_buffer[id][0].id >> 8 ) & 0xFF;
		data[3] = (rx_buffer[id][0].id >> 16 ) & 0xFF;
		data[4] = (rx_buffer[id][0].id >> 24 ) & 0x1F;
		data[4] |= CAN_EFF_FLAG >> 24;
	} else {
		data[1] = (rx_buffer[id][0].id >> 18) & 0xFF;
		data[2] = ((rx_buffer[id][0].id >> 18) >> 8 ) & 0x7F;
		data[3] = 0x00;
		data[4] = 0x00;
	}
	data[4] |= rx_buffer[id][0].type ?  (CAN_RTR_FLAG >> 24):0x00;

	data[5] = rx_buffer[id][0].dataByte0;
	data[5 + 1] = rx_buffer[id][0].dataByte1;
	data[5 + 2] = rx_buffer[id][0].dataByte2;
	data[5 + 3] = rx_buffer[id][0].dataByte3;
	data[5 + 4] = rx_buffer[id][0].dataByte4;
	data[5 + 5] = rx_buffer[id][0].dataByte5;
	data[5 + 6] = rx_buffer[id][0].dataByte6;
	data[5 + 7] = rx_buffer[id][0].dataByte7;

	can_regs[id].recived_frames_cnt = 0;
	can_regs[id].can_status_reg &= ~CANINTF_RX;
	if (id == 1)
		vTaskResume(can1_task_handle);
	else
		vTaskResume(can0_task_handle);

	return CAN_TRANSFER_BUF_LEN;
}

uint8_t can_readframe_single_rw(int id, uint8_t address)
{

	if (address == 0){
		return rx_buffer[id][0].length;
	}

	if ((address > 0) && (address < 5) && can1_inframe){
		return  rx_buffer[id][0].id >> (8 * (address - 1));
	}

	if ((address > 4) && can1_inframe){
		switch (address){
		case 5: return rx_buffer[id][0].dataByte0;
		case 6: return rx_buffer[id][0].dataByte1;
		case 7: return rx_buffer[id][0].dataByte2;
		case 8: return rx_buffer[id][0].dataByte3;
		case 9: return rx_buffer[id][0].dataByte4;
		case 10: return rx_buffer[id][0].dataByte5;
		case 11: return rx_buffer[id][0].dataByte6;
		case 12:
			vTaskResume(id ? can1_task:can0_task);
			can_regs[id].recived_frames_cnt = 0;
			return rx_buffer[id][0].dataByte7;
		}
		if ((address - 4) == tx_frame[1].length)
			return can1_transmit();
	}
	return 0;
}

int canx_registers(dspi_transfer_t *spi_transfer, int id)
{
	uint8_t *rx_buf = spi_transfer->rxData;
	uint8_t *tx_buf = &spi_transfer->txData[1];

	if (rx_buf[0] == APALIS_TK1_K20_READ_INST) {
		rx_buf[2] -= APALIS_TK1_K20_CAN_DEV_OFFSET(id);
		switch (rx_buf[2]) {
		case APALIS_TK1_K20_CANREG:
			tx_buf[0] = get_canreg(id);
			break;
		case APALIS_TK1_K20_CANERR:
			tx_buf[0] = get_canerr(id);
			break;
		case APALIS_TK1_K20_CAN_BAUD_REG:
			tx_buf[0] = get_canbadreg(id);
			break;
		case APALIS_TK1_K20_CAN_BIT_1:
			tx_buf[0] = get_canbittimig(id) && 0xFF;
			break;
		case APALIS_TK1_K20_CAN_BIT_2:
			tx_buf[0] = (get_canbittimig(id) >> 8) && 0xFF;
			break;
		case APALIS_TK1_K20_CAN_IN_BUF_CNT:
			tx_buf[0] = get_can_rxcnt(id);
			break;
		default:
			if((rx_buf[1] >=APALIS_TK1_K20_CAN_IN_BUF) &&
					(rx_buf[1] <= APALIS_TK1_K20_CAN_IN_BUF_END)) {
				tx_buf[0] = can_readframe_single_rw(id, rx_buf[2]);
			} else
				return -EIO;
		}
		return 1;
	} else if (rx_buf[0] == APALIS_TK1_K20_WRITE_INST) {
		rx_buf[1] -= APALIS_TK1_K20_CAN_DEV_OFFSET(id);
		switch (rx_buf[1]) {
		case APALIS_TK1_K20_CANREG:
			return set_canreg(id, rx_buf[2]);
		case APALIS_TK1_K20_CANERR:
			return set_canerr(id, rx_buf[2]);
		case APALIS_TK1_K20_CAN_BAUD_REG:
			return set_canbadreg(id, rx_buf[2]);
		case APALIS_TK1_K20_CAN_BIT_1:
			return set_canbittimig(id, rx_buf[2], 0x00FF);
		case APALIS_TK1_K20_CAN_BIT_2:
			return set_canbittimig(id, (rx_buf[2] << 8), 0xFF00);
		default:
			if((rx_buf[1] >=APALIS_TK1_K20_CAN_OUT_BUF) &&
					(rx_buf[1] <= APALIS_TK1_K20_CAN_OUT_BUF_END) &&
					(id == 0)) {
				return can0_sendframe_single_rw(rx_buf[1] - APALIS_TK1_K20_CAN_OUT_BUF, rx_buf[2]);
			} else
				return -EIO;
		}

	} else if (rx_buf[0] == APALIS_TK1_K20_BULK_READ_INST) {
		rx_buf[2] -= APALIS_TK1_K20_CAN_DEV_OFFSET(id);
		switch (rx_buf[2]) {
		case APALIS_TK1_K20_CAN_IN_BUF:
			return can_readframe(id, tx_buf);
		default:
			return -EIO;
		}
	} else if (rx_buf[0] == APALIS_TK1_K20_BULK_WRITE_INST) {
		rx_buf[2] -= APALIS_TK1_K20_CAN_DEV_OFFSET(id);
		switch (rx_buf[2]) {
		case APALIS_TK1_K20_CAN_OUT_BUF:
			can_sendframe(id, &rx_buf[3], rx_buf[1]);
			break;
		case APALIS_TK1_K20_CAN_BIT_1:
		default:
			return -EIO;
		}
	}
	return -EIO;
}

