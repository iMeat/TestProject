#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <stdint.h>
#include <stdio.h>

#define MAX_PACKET_SIZE 1011
#define STATISTIC_ARRAY_SIZE	100


//typedef section ==============================================================
//method of clent subscribings to getting data from server
typedef enum {INDICATION = 0x02,
							NOTIFICATION = 0x01} notificationType_t;
//device role
typedef enum {CLIENTS = 0,
							SERVERS} cltOrServer_t;
//client states for FSM
typedef enum {CLT_WAIT_INDICATION = 0,	//for indication
							CLT_SEND_CONFIRMATION,		//for indication
							CLT_WAIT_NOTIFICATION,		//for notification
							CLT_RX_DONE								//for indication and confirmation
							} cltState_t;
//server states for FSM
typedef enum {SRV_IDLE = 0,
							SRV_START_INDICATION,			//for indication
							SRV_WAIT_CONFIRMATION,		//for indication
							SRV_TX_DONE,							//for indication and confirmation		
							SRV_START_NOTIFICATION,		//for notification
							SRV_LOCK_NOTIFICATION			//for notification
							} srvState_t;
//struct for server
typedef struct
{
	uint8_t		*txDataBase;//inlut value - USED
	uint16_t 	txDataSize;		//calc value - USED 
	uint16_t 	txPayloadSize; //input value - USED
	uint8_t		txPacketArray[MAX_PACKET_SIZE];// USED
	uint16_t 	txPacketQuantity; //calc value - USED
	uint16_t 	txPacketSize;			//calc value = payloadSize + packetNrSize + packetQuantity + timeStemp -USED
	uint16_t 	txLastPacketSize;// calc value - USED
	uint16_t	txPacketCounter;// work value - USED
} txData_t;

//struct for client
typedef struct
{
	uint8_t *rxDataBufferBase; //USED
	//	uint8_t rxBufferSize; //USED
	uint8_t rxDoneFlag;
} rxData_t;

typedef struct
{
	uint16_t connectionHandle;// USED
	uint16_t characteristicHandle;//USED
	cltOrServer_t cltOrSrv;// USED
	notificationType_t notificationType; //USED
	void (*cltReceptionDoneCb)(void);	//USED
	void (*srvTransmissionDoneCb)(void);//USED
}packetHandlerSettings_t;
//for statistic
typedef struct
	{
	uint32_t packetTime;
	uint16_t packetSize;
	}statistic_t;

typedef struct
	{
	statistic_t statistic[STATISTIC_ARRAY_SIZE];
	uint32_t firstPacketTime;
	uint32_t lastPacketTime;
	uint32_t timeSpent;
	}statisticData_t;

extern statistic_t packetHandlerTxStatistic[STATISTIC_ARRAY_SIZE];
extern statisticData_t packetHandlerRxStatistic;
	
//global variables =============================================================
extern cltState_t cltState;
extern srvState_t srvState;

//global function prototypes ===================================================
void PacketHandler(void);

void PacketHandlerInit(uint16_t connHandle,
											 uint16_t charHandle,
											 cltOrServer_t cltOrSrv,
											 notificationType_t notificationType,
											 uint8_t *rxBuffer);

//server --- send packet
uint8_t PacketHandlerSend(packetHandlerSettings_t *packetHandlerSettings,
													uint8_t *txDataBase,
													uint16_t dataSize,
													uint16_t payloadSize);

void PacketHandlerTxDoneSetCallback(void (*callbackFunction)(void));

void PacketHandlerRxDoneSetCallback(void (*callbackFunction)(void));
void PacketHandlerSetRxBuffer(uint8_t *rxBuffer);

//server --- add service data
//(packetNr - 2 bytes)(packetQuatnity - 2 byte )(packetSize - 2 byte)(packetTime - 4 bytes)(data - n bytes)
//uint8_t PacketHandlerPackDataPacket();


//client --- reception packet 
//uint8_t PacketHandlerReñeptionPacket();


#endif //PACKET_HANDLER_H