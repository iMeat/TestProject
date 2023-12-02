#include "packetHandler.h"
#include "ble_const.h"
#include "events.h"
#include "clock.h"
//#include "serial_port.h"
//define section
#define DEBUGGING
#define PACKET_START							0
#define PACKET_NUMBER_SIZE				2//bytes
#define PACKET_QUANTITY_SIZE			2//bytes
#define PACKET_TIMESTAMP_SIZE			4//bytes
#define PACKET_INFO_SIZE					(PACKET_NUMBER_SIZE + PACKET_QUANTITY_SIZE + PACKET_TIMESTAMP_SIZE)
//#define PACKET_NUMBER_POSITION		(PACKET_START)//0 to 1
//#define PACKET_QUANTITY_POSITION	(PACKET_START + PACKET_NUMBER_SIZE)//2 to 3
//#define PACKET_TIMESTAMP_POSITION	(PACKET_QUANTITY_POSITION + PACKET_TIMESTAMP_SIZE)//4 to 7
//#define PACKET_PAYLOAD_POSITION		(PACKET_TIMESTAMP_POSITION + PACKET_TIMESTAMP_SIZE)//8
#define PACKET_NUMBER_POSITION		(0)//0 to 1
#define PACKET_QUANTITY_POSITION	(2)//2 to 3
#define PACKET_TIMESTAMP_POSITION	(4)//4 to 7
#define PACKET_PAYLOAD_POSITION		(8)//8




cltState_t cltState;
srvState_t srvState;

txData_t txData;
rxData_t rxData;
packetHandlerSettings_t packetHandlerSettings; //server or client settigs
static uint16_t charachteristicHandle = 0;

statistic_t packetHandlerTxStatistic[STATISTIC_ARRAY_SIZE];
//statistic_t packetHandlerRxStatistic[STATISTIC_ARRAY_SIZE];

statisticData_t packetHandlerRxStatistic;


//local function section
void PacketHandlerPackDataPacket(uint8_t *payload, uint32_t time);
void PacketHandlerNotificationSrvFSM(void);
void PacketHandlerNotificationCltFSM(void);




//==============================================================================
void PacketHandlerInit(uint16_t connHandle,
											 uint16_t charHandle,
											 cltOrServer_t cltOrSrv,
											 notificationType_t notificationType,
											 uint8_t *rxBuffer)
{
	packetHandlerSettings.connectionHandle = connHandle;
	packetHandlerSettings.characteristicHandle = charHandle;
	packetHandlerSettings.cltOrSrv = cltOrSrv;//set client or server
	packetHandlerSettings.notificationType = notificationType; //indication or notification
	rxData.rxDoneFlag = 0;
	packetHandlerSettings.cltReceptionDoneCb = NULL;
	packetHandlerSettings.srvTransmissionDoneCb = NULL;
	//state initialisation
	if(cltOrSrv == SERVERS)
	{
		srvState = SRV_IDLE;
	}else if((cltOrSrv == CLIENTS) & (notificationType == NOTIFICATION))
	{
		cltState = CLT_WAIT_NOTIFICATION;
	}else if ((cltOrSrv == CLIENTS) & (notificationType == INDICATION))
	{
		cltState = CLT_WAIT_INDICATION;
	}
}
//==============================================================================
uint8_t PacketHandlerSend(uint8_t *txDataBase,
													uint16_t dataSize,
													uint16_t payloadSize)
{
	txData.txPacketCounter = 0;
	txData.txDataBase = txDataBase;
	txData.txDataSize = dataSize;
	txData.txPayloadSize = payloadSize;
	if(payloadSize != 0)
	{
		//packetNr - 2 bytes, packet quantity - 2 bytes, timestamp - 4 bytes
		txData.txPacketSize = payloadSize + PACKET_INFO_SIZE;
		txData.txPacketQuantity = (dataSize / payloadSize) - 1 ;//because start from 0
		//check for last short packet
		if( dataSize % payloadSize != 0)
		{
			txData.txLastPacketSize = dataSize % (payloadSize + PACKET_INFO_SIZE);//!!!!!!
			txData.txPacketQuantity ++;	//+ 1 for last packet
		}else
		{
			txData.txLastPacketSize = 0;//last packet is not shorh
		}
		//new state for FSM
		srvState = SRV_START_NOTIFICATION;//go to the start notification	
	}
	
}
//==============================================================================
void PacketHandler(void)
{
	if(packetHandlerSettings.notificationType == NOTIFICATION)
	{
		//========== NOTIFICATION --- SERVER
		if(packetHandlerSettings.cltOrSrv == SERVERS)
		{
			PacketHandlerNotificationSrvFSM();
		}
		//========== NOTIFICATION --- CLIENT
		else
		{
			PacketHandlerNotificationCltFSM();
		}
	}
	else 
	{
		//========== INDICATION --- SERVER
		if(packetHandlerSettings.cltOrSrv == SERVERS)
		{
			
		}
		//========== INDICATION --- CLIENT
		else 
		{
			
		}
	}
}


//==============================================================================
/**/
void PacketHandlerPackDataPacket(uint8_t *payload, uint32_t time)
{
	uint8_t *packetPointer;
	//add packet number
	packetPointer = &txData.txPacketArray[PACKET_NUMBER_POSITION];
	uint8_t byteCounter0 = (uint8_t)(txData.txPacketCounter & 0xFF);
	uint8_t byteCounter1 = (uint8_t)(txData.txPacketCounter >> 8) & 0xFF;
	*packetPointer = byteCounter1;
	packetPointer++;
	*packetPointer = byteCounter0;	
	//packetPointer = &txData.txPacketArray[PACKET_NUMBER_POSITION + 1];	
	//add packet quantity
	packetPointer = &txData.txPacketArray[PACKET_QUANTITY_POSITION];
	uint8_t byteQuantity0 = (uint8_t)(txData.txPacketQuantity & 0xFF);
	uint8_t byteQuantity1 = (uint8_t)((txData.txPacketQuantity >> 8) & 0xFF);
	*packetPointer = byteQuantity1;
	packetPointer++;
	*packetPointer = byteQuantity0;
	//add packet timestamp
	uint8_t byte0 = (uint8_t)(time & 0xFF);
	uint8_t byte1 = (uint8_t)((time >> 8) & 0xFF);
	uint8_t byte2 = (uint8_t)((time >> 16) & 0xFF);
	uint8_t byte3 = (uint8_t)((time >> 24) & 0xFF);
	packetPointer = &txData.txPacketArray[PACKET_TIMESTAMP_POSITION];	
	*packetPointer = byte3;
	packetPointer++;
	*packetPointer = byte2;
	packetPointer++;
	*packetPointer = byte1;
	packetPointer++;
	*packetPointer = byte0;
	packetPointer++;
	//add payload
//	uint16_t payloadOffset = txData.txPayloadSize * txData.txPacketCounter;
	for(uint16_t i = 0; i < txData.txPayloadSize; i++)
	{
//		txData.txPacketArray[PACKET_PAYLOAD_POSITION + i] =  payload[payloadOffset + i];
		txData.txPacketArray[PACKET_PAYLOAD_POSITION + i] =  payload[i];		
	}
}
//==============================================================================
void PacketHandlerUnpackDataPacket(uint16_t packetLenght,
																	 uint8_t packetData[],
																	 uint8_t txDataBase[])
{
	//read packet number
		uint16_t packetNr = (uint16_t)(packetData[0] << 8 ) + 
		(uint16_t)packetData[1];
#ifdef DEBUGGING
		
#endif	//DEBUGGING

		//read packet quantity
		uint16_t packetQuantity = (uint16_t)(packetData[2] << 8) +
		(uint16_t)packetData[3];
		//calc data pointer offset
		uint16_t dataBufferOffset = packetNr * (packetLenght - PACKET_INFO_SIZE);
		//read payload	
		for(uint16_t i = 0 ; i < packetLenght - PACKET_INFO_SIZE; i++)		
		{
			rxData.rxDataBufferBase[dataBufferOffset + i] = packetData[PACKET_INFO_SIZE + i];
		}			
		//write statistic
		//write packet size
		packetHandlerRxStatistic.statistic[packetNr].packetSize = packetLenght;
		//write timestamp

		uint32_t time = (uint32_t)(packetData[PACKET_TIMESTAMP_POSITION] << 24) +
			(uint32_t)(packetData[PACKET_TIMESTAMP_POSITION + 1] << 16) +
			(uint32_t)(packetData[PACKET_TIMESTAMP_POSITION + 2] << 8) +
			(uint32_t)(packetData[PACKET_TIMESTAMP_POSITION + 3]);
		
		//write first packet time statistic
		if(packetNr == 0)
		{
			packetHandlerRxStatistic.firstPacketTime = time;
		}
		//write last packet time statistic
		if(packetNr == packetQuantity)
		{
			packetHandlerRxStatistic.lastPacketTime = time;
			packetHandlerRxStatistic.timeSpent =
			packetHandlerRxStatistic.lastPacketTime -
			packetHandlerRxStatistic.firstPacketTime;
		}
		
		packetHandlerRxStatistic.statistic[packetNr].packetTime = time;
#ifdef DEBUGGING

		printf("Unpack. pNr: %04d, pQ: %04d\r\n , pT: %010d\r\n",  
					 packetNr, 
					 packetQuantity, 
					 time);				
		
#endif //DEBUGGING		
		//check for last packet
		if(packetNr == packetQuantity)
		{
			rxData.rxDoneFlag = 1;
		}
}
//==============================================================================

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{
	if(srvState == SRV_LOCK_NOTIFICATION)
	{
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS5, LL_GPIO_OUTPUT_HIGH);
		srvState = SRV_START_NOTIFICATION;
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS5, LL_GPIO_OUTPUT_LOW);
//	printf("pool_avalable_event () SRV_START_NOTIFICATION\r\n");
	}
}
//==============================================================================
void aci_gatt_clt_indication_event(uint16_t Connection_Handle,
                                   uint16_t Attribute_Handle,
                                   uint16_t Attribute_Value_Length,
                                   uint8_t Attribute_Value[])
{
	if(cltState == CLT_WAIT_INDICATION)
	{
		PacketHandlerUnpackDataPacket(Attribute_Value_Length, Attribute_Value, rxData.rxDataBufferBase);
		if(rxData.rxDoneFlag == 1)
		{
			cltState = CLT_RX_DONE;
		}
		
		//cltState = CLT_RX_DONE;
	}
}
//==============================================================================
void aci_gatt_clt_notification_event(uint16_t Connection_Handle,
                                     uint16_t Attribute_Handle,
                                     uint16_t Attribute_Value_Length,
                                     uint8_t Attribute_Value[])
{
#ifdef DEBUGGING
//	printf("aci_gatt_clt_notification_event()\r\n");

#endif //DEBUGGING
	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH);
	if(cltState == CLT_WAIT_NOTIFICATION)
	{
		PacketHandlerUnpackDataPacket(Attribute_Value_Length, Attribute_Value, rxData.rxDataBufferBase);
		if(rxData.rxDoneFlag == 1)
		{
			cltState = CLT_RX_DONE;
		}
	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW);
	}
}

//==============================================================================
void PacketHandlerNotificationSrvFSM(void)
{
//			if(txData.txPacketCounter == 31)
//			{
//				__ASM("NOP");
//			}
			//==========
			if(srvState == SRV_IDLE)// no data to send
			{
				return;
			}
			//==========
			if(srvState == SRV_START_NOTIFICATION)//start notification for client
			{
				//all packets has been transmitted - go to SRV_TX_DONE
				if(txData.txPacketCounter > txData.txPacketQuantity)//last packet !!!!	
				{
					srvState = SRV_TX_DONE;
					return;
				}
				//calculate new data offset for sending
				uint8_t *txDataOffset = txData.txDataBase +
				txData.txPayloadSize * txData.txPacketCounter;//!!!!!!!!!!!!!!!!!!!!
//				printf("DtOffs: %04d\r\n", txDataOffset);//print dataoffset address
				//make the data packet with info data and payload
				uint32_t currentTime = Clock_Time();
				PacketHandlerPackDataPacket(txDataOffset , currentTime);				
				uint8_t status;
				//attempt to send data packet
				/*test for penultimate data packet*/
//				if(txData.txPacketCounter < txData.txPacketQuantity)
				uint8_t notLastPacket = (txData.txPacketCounter < txData.txPacketQuantity); 
				if((txData.txLastPacketSize == 0) || ((txData.txLastPacketSize != 0) && (notLastPacket)))	
				{
				//attempt to send data packet
					LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH);
					status = aci_gatt_srv_notify(packetHandlerSettings.connectionHandle,
																		 packetHandlerSettings.characteristicHandle + 1,
																		 NOTIFICATION,
																		 txData.txPacketSize, &txData.txPacketArray[0]);
					LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW);					
				}	
//				else if ((txData.txPacketCounter == txData.txPacketQuantity)
//								 &&(txData.txLastPacketSize != 0))
				else
				{
					//attempt to send lat short data packet
					LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH);					
					status = aci_gatt_srv_notify(packetHandlerSettings.connectionHandle,
																			 packetHandlerSettings.characteristicHandle + 1,
																		 	 NOTIFICATION,
																		 	 txData.txLastPacketSize, &txData.txPacketArray[0]);
					LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW);					
				}
//				printf("0x%02x\r\n",status);
				if(status == BLE_STATUS_SUCCESS)
				{
					packetHandlerTxStatistic[txData.txPacketCounter].packetTime = currentTime;
					//write packet size or last packet size
					if ((txData.txPacketCounter == txData.txPacketQuantity)
								 &&(txData.txLastPacketSize != 0))
					{
						packetHandlerTxStatistic[txData.txPacketCounter].packetSize = txData.txLastPacketSize - 1;
					}
					else
					{
						packetHandlerTxStatistic[txData.txPacketCounter].packetSize = txData.txPacketSize;
					}
					txData.txPacketCounter++;
//					printf("PacketCounter: %004d\r\n", txData.txPacketCounter);
				//stay in  SRV_START_NOTIFICATION state for new packet transmisson
					srvState = SRV_START_NOTIFICATION;
				}else if(status == BLE_STATUS_INSUFFICIENT_RESOURCES)
				{
					//lock notification until issue aci_gatt_tx_pool_avalable_event()
					srvState = SRV_LOCK_NOTIFICATION;
//					printf("SRV_LOCK_NOTIFICATION\r\n");
				}
				
			}
			//==========
			if(srvState == SRV_LOCK_NOTIFICATION)
			{
				//wait aci_gatt_tx_pool_avalable_event()
				return;
			}
			//==========
			if(srvState == SRV_TX_DONE)
			{
				txData.txPacketCounter = 0;
				// issue the done function
				if(packetHandlerSettings.srvTransmissionDoneCb != NULL)
				{
					packetHandlerSettings.srvTransmissionDoneCb();
				}
				srvState = SRV_IDLE;
			}
}
//==============================================================================
void PacketHandlerIndicationCltFSM(void)
{
	
}
//==============================================================================
void PacketHandlerIndicationSrvFSM(void)
{
	
}
//==============================================================================


void PacketHandlerNotificationCltFSM(void)
{
	if(cltState == CLT_WAIT_NOTIFICATION)
	{
		return;
	}
	if(cltState == CLT_RX_DONE)
	{
		rxData.rxDoneFlag = 0;
//		PacketHandlerRxDoneCb();
		if(packetHandlerSettings.cltReceptionDoneCb != NULL)
		{
			packetHandlerSettings.cltReceptionDoneCb();
		}
		cltState = CLT_WAIT_NOTIFICATION;
	}
}
//==============================================================================
void PacketHandlerTxDoneSetCallback(void (*callbackFunction)(void))
{
	packetHandlerSettings.srvTransmissionDoneCb = callbackFunction;
}
//==============================================================================
void PacketHandlerRxDoneSetCallback(void (*callbackFunction)(void))
{
	packetHandlerSettings.cltReceptionDoneCb = callbackFunction;
}
//==============================================================================
void PacketHandlerSetRxBuffer(uint8_t *rxBuffer)
{
	rxData.rxDataBufferBase = rxBuffer;
}
//==============================================================================