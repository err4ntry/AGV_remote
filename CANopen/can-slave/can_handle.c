#include "canfestival.h"
#include "can.h"
#include "can_handle.h"

// extern CO_Data movens_1_Data;
CAN_TxHeaderTypeDef can_tx_header;
CAN_RxHeaderTypeDef can_rx_header;
// last_control_cmd _last_cmd={0x00,0x00,0x00,0x00};//��ʼ���������״̬

void InitNodes(CO_Data *d, UNS32 id)
{
	/****************************** INITIALISATION SLAVE
	 *******************************/
	// if (strcmp(slaveBoard.baudrate, "none")) {
	/*TestSlave.C ����רҵ�������ɵĶ����ֵ�*/
	/*����ӳ���Ӧ��ϵ
	SDO(rx)��602 SDO(tx): 582
	TPDO: 182 282 382 482
	RPDO: �ӽڵ�û�ж����ֵ��Ӧ���� PDO*/
	unsigned char deviceid = 0;
	setNodeId(d, id); // set node id @bruce
	// deviceid=*(master_node_Data.bDeviceNodeId);

	// printf(" 11 deviceid is 0x[%x]\r\n",deviceid);
	/* init */
	setState(d, Initialisation); // boot-up �ýڵ� ���� Pre-optional ״̬
	// deviceid=*(master_node_Data.bDeviceNodeId);
	setState(d, Operational);

	masterSendNMTstateChange(d, 0, NMT_Start_Node);

	// printf(" 22 deviceid is 0x[%x]\r\n",deviceid);
}

unsigned char canSend(CAN_PORT notused, Message *m)
{
	uint8_t txbuf[8];
	uint32_t i, TxMailbox;
	can_tx_header.StdId = m->cob_id;
	uint8_t status = 0;

	if (m->rtr == 0)
		can_tx_header.RTR = CAN_RTR_DATA;
	else
		can_tx_header.RTR = CAN_RTR_REMOTE;

	can_tx_header.IDE = CAN_ID_STD;
	can_tx_header.DLC = m->len;
	can_tx_header.TransmitGlobalTime = DISABLE;
	for (i = 0; i < m->len; i++)
		txbuf[i] = m->data[i];

	while (0 == HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		;

	if (HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, txbuf, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		// DEBUG_PRINTF("error1\r\n");
		status = 1;
	}
	return status;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	uint8_t i = 0;
	Message RxMSG;

	uint8_t rxdata[8];
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, rxdata);

	//	printf("id:%#x\r\n",can_rx_header.StdId);
	//  printf("len:%d\r\n",can_rx_header.DLC);

	if (can_rx_header.RTR == CAN_RTR_REMOTE)
	{
		RxMSG.rtr = 1;
	}
	else
	{
		RxMSG.rtr = 0;
	}
	RxMSG.cob_id = can_rx_header.StdId;
	RxMSG.len = can_rx_header.DLC;

	for (i = 0; i < 8; i++)
	{
		RxMSG.data[i] = rxdata[i];
		// printf("rxdata[%d]: %d\r\n",i,rxdata[i]);
	}

	canDispatch(&master_node_Data, &(RxMSG)); // canfestival �������

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void _SetMode(uint16_t mode)
{
	switch (mode)
	{
	case MOTOR_ENABLE:
		motor_MotorEnable();
		break;

	case MOTOR_DISABLE:
		motor_MotorDisable();
		break;

	case MOTOR_RESET:
		motor_Reset();
		break;

	case MOTOR_UPSTOP:
		motor_UpStop();
		break;

	case MOTOR_TORQUE_MODE:
		motor_TorqueMode();
		break;

	case MOTOR_SPEED_MODE:
		motor_SpeedMode();
		break;

	case MOTOR_POSITION_MODE:
		motor_PositionMode();
		break;

	case MOTOR_ERROR_HANDLE:
		motor_ErrorHandle();
		break;

	case MOTOR_ENCODER_ALIGN:
		motor_EncoderAlign();
		break;

	default:
		errorHandle(NO_SUCH_MODE);
		break;
	}
}

void errorHandle(uint16_t error)
{
	printf("Error : no such mode ! error code = %i \n", error);
}

void SelectSlaveDeviceId(uint32_t id)
{
	//	master_node_obj1800_COB_ID_used_by_PDO = 0x200 + id;
	//	master_node_obj1400_COB_ID_used_by_PDO = 0x180 + id;
	//	master_node_obj1401_COB_ID_used_by_PDO = 0x280 + id;
}

void motor_Reset() { printf("cmd_Reset\n"); }
void motor_MotorEnable() { printf("cmd_MotorEnable\n"); }
void motor_MotorDisable() { printf("cmd_MotorDisable\n"); }
void motor_TorqueMode() { printf("cmd_TorqueMode\n"); }
void motor_UpStop() { printf("cmd_UpStop\n"); }
void motor_SpeedMode() { printf("cmd_SpeedMode\n"); }
void motor_PositionMode() { printf("cmd_PositionMode\n"); }
void motor_ErrorHandle() { printf("cmd_ErrorHandle\n"); }
void motor_EncoderAlign() { printf("cmd_EncoderCorrect\n"); }

uint16_t _GetTorque()
{
	printf("_GetTorque\n");
	return 0;
}
uint16_t _GetSpeed()
{
	printf("_GetSpeed\n");
	return 0;
}
uint16_t _GetPosition()
{
	printf("_GetPosition\n");
	return 0;
}

void _SetTorque(uint16_t torque) { printf("cmd_SetTorque\n"); }
void _SetSpeed(uint16_t speed) { printf("cmd_SetSpeed\n"); }
void _SetPosition(uint16_t position) { printf("cmd_SetPosition\n"); }
