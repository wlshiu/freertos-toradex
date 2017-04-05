
#include "can_task.h"
#include "com_task.h"
#include "fsl_flexcan.h"

#define RX_MESSAGE_BUFFER_NUM0 (9)
#define TX_MESSAGE_BUFFER_NUM0 (8)
#define RX_MESSAGE_BUFFER_NUM1 (7)
#define TX_MESSAGE_BUFFER_NUM1 (6)

static flexcan_handle_t flexcanHandle[2];
static uint32_t txIdentifier[2];
static uint32_t rxIdentifier[2];
static volatile bool txComplete[2] = {false, false};
static volatile bool rxComplete[2] = {false, false};

static void flexcan_callback0(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	callback_message_t * cb = (callback_message_t*) userData;
	BaseType_t reschedule = pdFALSE;

	switch (status)
	{
	case kStatus_FLEXCAN_RxIdle:
		if (RX_MESSAGE_BUFFER_NUM0 == result)
		{
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
		}
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM0 == result)
		{
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
		if (RX_MESSAGE_BUFFER_NUM1 == result)
		{
			xSemaphoreGiveFromISR(cb->sem, &reschedule);
		}
		break;

	case kStatus_FLEXCAN_TxIdle:
		if (TX_MESSAGE_BUFFER_NUM1 == result)
		{
		}
		break;

	default:
		break;
	}
	portYIELD_FROM_ISR(reschedule);
}

static void CAN_Init()
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	txIdentifier[0] = 0x321;
	rxIdentifier[0] = 0x123;
	txIdentifier[1] = 0x123;
	rxIdentifier[1] = 0x321;

	/* Get FlexCAN module default Configuration. */
	/*
	 * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
	 * flexcanConfig.baudRate = 125000U;
	 * flexcanConfig.maxMbNum = 16;
	 * flexcanConfig.enableLoopBack = false;
	 * flexcanConfig.enableSelfWakeup = false;
	 * flexcanConfig.enableIndividMask = false;
	 * flexcanConfig.enableDoze = false;
	 */
	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN0, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle[0], flexcan_callback0, flexcanHandle[0].userData);

	/* Set Rx Masking mechanism. */
	FLEXCAN_SetRxMbGlobalMask(CAN0, FLEXCAN_RX_MB_STD_MASK(rxIdentifier[0], 0, 0));

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
	mbConfig.type = kFLEXCAN_FrameTypeData;
	mbConfig.id = FLEXCAN_ID_STD(rxIdentifier[0]);
	FLEXCAN_SetRxMbConfig(CAN0, RX_MESSAGE_BUFFER_NUM0, &mbConfig, true);

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM0, true);

	/* Get FlexCAN module default Configuration. */
	/*
	 * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
	 * flexcanConfig.baudRate = 125000U;
	 * flexcanConfig.maxMbNum = 16;
	 * flexcanConfig.enableLoopBack = false;
	 * flexcanConfig.enableSelfWakeup = false;
	 * flexcanConfig.enableIndividMask = false;
	 * flexcanConfig.enableDoze = false;
	 */
	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	/* Init FlexCAN module. */
	flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	FLEXCAN_Init(CAN1, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(CAN1, &flexcanHandle[1], flexcan_callback1, flexcanHandle[1].userData);

	/* Set Rx Masking mechanism. */
	FLEXCAN_SetRxMbGlobalMask(CAN1, FLEXCAN_RX_MB_STD_MASK(rxIdentifier[1], 0, 0));

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
	mbConfig.type = kFLEXCAN_FrameTypeData;
	mbConfig.id = FLEXCAN_ID_STD(rxIdentifier[1]);
	FLEXCAN_SetRxMbConfig(CAN1, RX_MESSAGE_BUFFER_NUM1, &mbConfig, true);

	mbConfig.format = kFLEXCAN_FrameFormatExtend;
	mbConfig.type = kFLEXCAN_FrameTypeRemote;
	mbConfig.id = FLEXCAN_ID_STD(txIdentifier[1]);
	FLEXCAN_SetRxMbConfig(CAN1, RX_MESSAGE_BUFFER_NUM1, &mbConfig, true);


	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(CAN1, TX_MESSAGE_BUFFER_NUM1, true);
	PRINTF("CAN init done \r\n");
}


void can_test_task(void *pvParameters) {
	flexcan_frame_t txFrame, rxFrame;
	flexcan_mb_transfer_t txXfer, rxXfer;
	callback_message_t cb_msg[2];

	cb_msg[0].sem = xSemaphoreCreateBinary();
	cb_msg[1].sem = xSemaphoreCreateBinary();
	flexcanHandle[0].userData = (void *) &cb_msg[0];
	flexcanHandle[1].userData = (void *) &cb_msg[1];
	CAN_Init();

	vTaskSuspend(NULL);
	rxXfer.frame = &rxFrame;
	rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM1;
	FLEXCAN_TransferReceiveNonBlocking(CAN1, &flexcanHandle[1], &rxXfer);

	txFrame.format = kFLEXCAN_FrameFormatStandard;
	txFrame.type = kFLEXCAN_FrameTypeData;
	txFrame.id = FLEXCAN_ID_STD(0x321);
	txFrame.length = 8;
	txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
			CAN_WORD0_DATA_BYTE_3(0x44);
	txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
			CAN_WORD1_DATA_BYTE_7(0x88);
	txXfer.frame = &txFrame;
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM0;

	PRINTF("\r\nCAN0->CAN1 ");

	FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle[0], &txXfer);
	if (xSemaphoreTake(cb_msg[1].sem, 1000u) == pdFALSE)
	{
		PRINTF("FAIL!\r\n");
		FLEXCAN_TransferAbortSend(CAN0, &flexcanHandle[0], txXfer.mbIdx);
		FLEXCAN_TransferAbortReceive(CAN1, &flexcanHandle[1], rxXfer.mbIdx);
	}
	else
	{
		PRINTF("SUCCESS!\r\n");
	}

	rxXfer.frame = &rxFrame;
	rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM0;
	FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle[0], &rxXfer);

	txFrame.format = kFLEXCAN_FrameFormatStandard;
	txFrame.type = kFLEXCAN_FrameTypeData;
	txFrame.id = FLEXCAN_ID_STD(0x123);
	txFrame.length = 8;
	txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
			CAN_WORD0_DATA_BYTE_3(0x44);
	txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
			CAN_WORD1_DATA_BYTE_7(0x88);
	txXfer.frame = &txFrame;
	txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM1;

	PRINTF("CAN1->CAN0 ");

	FLEXCAN_TransferSendNonBlocking(CAN1, &flexcanHandle[1], &txXfer);
	if (xSemaphoreTake(cb_msg[0].sem, 1000u ) == pdFALSE)
	{
		PRINTF("FAIL!\r\n");
		FLEXCAN_TransferAbortSend(CAN1, &flexcanHandle[0], txXfer.mbIdx);
		FLEXCAN_TransferAbortReceive(CAN0, &flexcanHandle[1], rxXfer.mbIdx);
	}
	else
	{
		PRINTF("SUCCESS!\r\n");
	}

	vSemaphoreDelete(cb_msg[0].sem);
	vSemaphoreDelete(cb_msg[1].sem);

	vTaskResume(spi_task_handle);
	vTaskDelete(NULL);
}

int can0_registers(uint8_t *rx_buf, uint8_t *tx_buf){
	return -EIO;
}

int can1_registers(uint8_t *rx_buf, uint8_t *tx_buf){
	return -EIO;
}
