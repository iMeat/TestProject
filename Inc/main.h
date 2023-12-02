#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "com_port_handler.h"
//#include "console.h"

#define AUTOCONNECT_MODE
#define CONNECTED_DEV_LIST_SIZE		8

//LEDS
//BLUE - PA6
#define LED_BLUE_OFF() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH)
#define LED_BLUE_ON() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_LOW)
//GREEN - PB8
#define LED_GREEN_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_HIGH)	
#define LED_GREEN_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_LOW)	
//RED -  PB9
#define LED_RED_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_HIGH)	
#define LED_RED_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_LOW)


typedef enum{	SCANNING = 0,								//device makes scan to make the scan list
							STOP_SCANNING,
							WAITING_CONNECT,					//device wait num of device from scan list to be connected
							CONNECTION_ERROR,
							CONNECTING,
							CONNECTED,
							FEAT_EXCHANGE,
							SET_DATA_LENGHT,
							READ_PRIMARY_SERVICE,
							READ_CHARACTERISTICS,
							READ_DESCRIPTORS,
							READ_VALUES,
							GET_NOTIFICATION,
							READ_DATA,
							READ_MTU,
							DISCONNECTION,
							AUTOCONNECT,
							RECONNECT,
						} state_machine_t;

//Flags
extern uint8_t featureExchangedFlag;



typedef struct
{
	uint8_t			bankIndex;
	uint8_t 		linkStatus[8];
	uint16_t 		LinkConnectionHandle[8];
} listLinks_t;


typedef struct
{
	uint8_t 		order;
	uint8_t 		peerAddressType;
	uint8_t 		peerAddress[6];
	uint16_t 		connectionHandle;
	uint8_t			role;
	uint16_t		connInterval;
	uint16_t		connLatency;
	uint16_t		supervisionTimeout;
	uint8_t			masterClockAccuracy;
	
} connDev_t;

typedef struct
{
	connDev_t 	connDev[CONNECTED_DEV_LIST_SIZE];
	uint8_t 		nextElement;
} connDevList_t;

typedef struct
{
	uint8_t test;
} service_t;
	
typedef struct
{
	uint64_t allPackets;
	uint64_t allTime;
	uint64_t allBadCRCPackets;
	uint64_t allDataCounter;
	uint64_t allConnectionLost;
	
}allTimeStatistic_t;
//Global variables
extern	uint16_t packetCounter;
extern allTimeStatistic_t allTimeStatistic;

//functions defines
void applicationTick(void);
void bluetoothSetting(void);
void StartScanning(void);
void StopScanning(void);
//void MakeConnection(list_t scanList, uint16_t number);
void StateMachine(state_machine_t *current_state);
void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 uint8_t Data[]);
uint8_t strIsNum(uint8_t string[]);
uint16_t	strToUInt16(uint8_t string[]);
void GetLinksStatus(listLinks_t *linkList);
void GetPrimaryService(connDevList_t *connDevList, uint8_t order);
void GetCharactristics(void);
void GetDescriptors(void);
void DisconnectDevice(void);
void ReadValue(uint16_t UUID);
void GetNotification(void);
void GetMaxMTU(void);
void ReadDataFromBufferToBuffer(uint8_t *sourceBuffer,
																	 uint8_t *destinationBuffer,
																	 uint16_t Lenght);

void ConnDevListEntryDeliteByConnectionHandle(connDevList_t *list,
																							uint16_t connectionHandle);
void ConnDevListEntryDeliteByOrder(connDevList_t *list, uint8_t order);
/*
configurating---Scaning
scaning---Wait_connect
waiting_connect---scaning---connecting
connecting---connection_error---


*/




#endif //MAIN_H