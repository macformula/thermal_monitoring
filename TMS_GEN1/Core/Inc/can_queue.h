/*
 *  can_queue.h
 *
 *  Created on: May 1st, 2022
 *  Author: Samuel Parent
 */
#define NUM_CAN_MESSAGES 4

typedef struct {
	CAN_HandleTypeDef 	CANHandle;
	CAN_TxHeaderTypeDef TxHeader;
	int8_t 				TxData[8];
	uint32_t 			TxMailbox;
}TxPackage;

TxPackage				TxQueue[NUM_CAN_MESSAGES+1];
uint8_t					qFront = 0;
int8_t					qEnd = 0;

uint8_t isFull(void);
uint8_t CAN_Enqueue(TxPackage newMsg);
uint8_t isEmpty(void);
uint8_t CAN_Dequeue(void);
void Tx_Header_Init(void);
void Tx_Package_Init(void);

uint8_t isFull()
{
    if (((qEnd+1)%(NUM_CAN_MESSAGES+1)) == qFront)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t CAN_Enqueue(TxPackage newMsg)
{
    if(isFull())
    {
        return 0;
    }
    else
    {
        TxQueue[qEnd] = newMsg;
        qEnd = (qEnd+1)%(NUM_CAN_MESSAGES+1);
        return 1;
    }
}

uint8_t isEmpty(void)
{
    return qFront == qEnd;
}

uint8_t CAN_Dequeue(void)
{
    if (isEmpty())
    {
        return 0;
    }
    else
    {
    	HAL_CAN_AddTxMessage(&TxQueue[qFront].CANHandle, &TxQueue[qFront].TxHeader, &TxQueue[qFront].TxData, &TxMailbox);
        qFront = (qFront+1)%(NUM_CAN_MESSAGES+1);
        return 1;
    }
}

void Tx_Package_Init(void)
{
	TxBMS.TxHeader = Tx_BMS_broadcast;
	TxBMS.CANHandle = hcan1;
	TxAddrClaim.TxHeader = Tx_address_claim;
	TxAddrClaim.CANHandle = hcan1;
	TxGeneral.TxHeader = Tx_gen_broadcast;
	TxGeneral.CANHandle = hcan1;
	TxLegacy.TxHeader = Tx_legacy_broadcast;
	TxLegacy.CANHandle = hcan1;
}

void Tx_Header_Init(void)
{
	  //Themistor Module -> BMS Broadcast
	  Tx_BMS_broadcast.StdId = 0x00;
	  Tx_BMS_broadcast.ExtId = 0x1839F380;
	  Tx_BMS_broadcast.RTR = CAN_RTR_DATA;
	  Tx_BMS_broadcast.IDE = CAN_ID_EXT;
	  Tx_BMS_broadcast.DLC = 8;
	  Tx_BMS_broadcast.TransmitGlobalTime = DISABLE;
//
//	  //Thermistor General Broadcast
//	  Tx_gen_broadcast.StdId = 0x00;
//	  Tx_gen_broadcast.ExtId = 0x1838F380;
//	  Tx_gen_broadcast.RTR = CAN_RTR_DATA;
//	  Tx_gen_broadcast.IDE = CAN_ID_EXT;
//	  Tx_gen_broadcast.DLC = 8;
//	  Tx_gen_broadcast.TransmitGlobalTime = DISABLE;
//
//	  //J1939 Adress Claim Broadcast
//	  Tx_address_claim.StdId = 0x00;
//	  Tx_address_claim.ExtId = 0x18EEFF80;
//	  Tx_address_claim.RTR = CAN_RTR_DATA;
//	  Tx_address_claim.IDE = CAN_ID_EXT;
//	  Tx_address_claim.DLC = 8;
//	  Tx_address_claim.TransmitGlobalTime = DISABLE;
//
//	  //Legacy Broadcast Message - RESERVED
//	  Tx_legacy_broadcast.StdId = 0x80;
//	  Tx_legacy_broadcast.ExtId = 0x00;
//	  Tx_legacy_broadcast.RTR = CAN_RTR_DATA;
//	  Tx_legacy_broadcast.IDE = CAN_ID_STD;
//	  Tx_legacy_broadcast.DLC = 4;
//	  Tx_legacy_broadcast.TransmitGlobalTime = DISABLE;
}
