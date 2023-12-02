#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>
#include "bluenrg_lp_stack.h"
#include "rf_driver_ll_usart.h"// for interrupts
#include "steval_idb011V1_config.h"
#include "com_port_handler.h"
#include "main.h"

#define SCAN_LIST_SIZE																		64//1024

//Define 
typedef struct
{
	uint8_t		order;
	uint8_t		addressType;
	uint8_t 	address[6];
	int8_t		rssi;
	uint8_t		primPHY;
	uint8_t		secPHY;
	uint8_t		deviceName[16];
} deviceInfo_t;

typedef struct
{
	deviceInfo_t	deviceInfo[SCAN_LIST_SIZE];
	uint8_t nextElement;
} list_t;


//==============================================================================	
//function definitions
/*Initialise scan list*/	
void ScanListInit(list_t *s);	
/*Clear scan list*/
void ScanListClear(list_t *s);
/* check two addresses*/
uint8_t AddressCheck(uint8_t deviceAddress[6], uint8_t dataAddress[6]);
//=============SCREENS
void StartScanningStatePrint(void);
void WaitingConnectStatePrint(void);
uint8_t ScanListUpdate(list_t *scanList, Extended_Advertising_Report_t *(data));
void LinksStatisPrint(listLinks_t *linksList);
void connDevListPrint(connDevList_t *connDevList);



/*Print scanList table to console*/
void ScanListPrint(list_t *scanList);

uint8_t ScanListUpdate(list_t *scanList, Extended_Advertising_Report_t *(data));

uint8_t ReadMessageFromRx(rxBuffer_t *buffer, uint8_t message[]);

void UartInit(void);

#endif //CONSOLE_H