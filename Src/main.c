
//My test
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ble_const.h"			//defines from lib
#include "bluenrg_lp_stack.h"	//main source file of bluetooth stack
#include "osal.h"				//operational system abstractive layer
#include "stack_config.h"	//ble stack configuration
//#include "startup_BlueNRG_LP.h"
#include "system_BlueNRG_LP.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"
#include "steval_idb011V1_config.h"
#include "clock.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "com_port_handler.h"
#include "nvm_db.h"
#include "bluenrg_lp_api.h"
#include "events.h"

#include "console.h"
#include "packetHandler.h"






//#include "custom_ble_stack_conf.h"

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

//==============================================================================
//scan defines
#define SCAN_INTERVAL_MS                160
#define SCAN_WINDOW_MS                  160
#define SYNC_TIMEOUT_MS                 1000
//#define CONN_INTERVAL_MIN   ((uint16_t)(6/1.25))       // 20 ms to 10
//#define CONN_INTERVAL_MAX   ((uint16_t)(6/1.25))       // 20 ms to 10
//#define CONN_INTERVAL_MIN   ((uint16_t)(7.5/1.25))       // 20 ms to 10
//#define CONN_INTERVAL_MAX   ((uint16_t)(7.5/1.25))       // 20 ms to 10
#define CONN_INTERVAL_MIN   ((uint16_t)(15.0/1.25))       // 15ms for 1M
#define CONN_INTERVAL_MAX   ((uint16_t)(15.0/1.25))       // 15ms for 1M


#define SUPERVISION_TIMEOUT ((uint16_t)(100/10))       // 1000 ms
//#define CE_LENGTH           ((uint16_t)(10/0.625))      // 20 ms to 10
//#define CE_LENGTH           ((uint16_t)(2/0.625))      // 20 ms to 10
#define CE_LENGTH           ((uint16_t)(200))      // 20 ms to 10
#define CE_LENGTH_MIN				((uint16_t)(200)) 
#define CE_LENGTH_MAX				((uint16_t)(200)) 
#define SLAVE_LATENCY				0
#define ACI_GAP_CONNECTION //connecton method
#define RX_BUFFER_SIZE			33000
//AUTOCONNECT_MODE defined in main.h
#define AUTOCONNECT_PHY_1M

//SPEED CONFIG
#define PHY_SPEED LE_1M_PHY_BIT
//#define PHY_SPEED LE_2M_PHY_BIT
//#define PHY_SPEED LE_CODED_PHY_BIT

//SPEED CODED OPTION
#define PHY_OPTION	0//no option (for LE_1M_PHY and LE_2M_PHY)
//#define PHY_OPTION  1// S=2 (only for LE_CODED_PHY_BIT)
//#define PHY_OPTION	2// S = 8 (only for LE_CODED_PHY_BIT)



//==============================================================================
	list_t 			scanList;
	listLinks_t 	linksStatusList;
	uint8_t command[50];
	extern rxBuffer_t comRxBuffer;
	uint8_t scanOn = 0;
#ifdef AUTOCONNECT_MODE
	state_machine_t state = AUTOCONNECT;
#else
	state_machine_t state = SCANNING;
#endif//AUTOCONNECT_MODE
	uint8_t firstEntryToState = 1;
	uint8_t rxMessage[32] = {0};
	uint8_t connectionNumber = 0;
	volatile uint32_t sysTick;
	connDevList_t	connDevList;
	ble_gatt_srv_def_t server;
	ble_gatt_chr_def_t characteristic;
	uint16_t readValueList[21] =
	{
	0x0001,//servise description
	0x0002,
	0x0003,
	0x0004,
	0x0005,
	0x0006,
	0x0007,
	0x0008,
	0x0009,
	0x000A,
	0x000B,
	0x000C,
	0x000D,
	0x000E,
	0x000F,
	0x0010,
	0x0011,
	0x0012,
	0x0013,
	0x0014,
	0x0015};
	uint8_t readDoneFlag;
	uint8_t featureExchangedFlag = 0;
	uint8_t testBuffer[1000];
	uint16_t packetCounter = 0;
	
	uint8_t phRxBuffer[33000];
	llc_conn_per_statistic_st statisticPerConnection;
	allTimeStatistic_t allTimeStatistic;
	
	//printBusy = 0;
	
//==============================================================================
//sysTick memory for programm timer

uint32_t sysTickMem;
void receptionDataDone(void);

//void ScanListUpdate(scanRecord_t *s, Extended_Advertising_Report_t extended_advertising_report[]);
//void ScanListPrint(scanRecord_t *s);



//==============================================================================
void main (void)
{
	allTimeStatistic.allBadCRCPackets = 0;
	allTimeStatistic.allPackets = 0;
	allTimeStatistic.allTime = 0;
	allTimeStatistic.allDataCounter = 0;
	uint8_t ret;
	BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
	//enable clocking Public Key Accelerator and Rundom Number Generator
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG); 	
  /* System initialization function */
if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
{
	/* Error during system clock configuration take appropriate action */
  while(1);
}  
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();  
  /* Init Clock */
  Clock_Init();
  /* Configure I/O communication channel */
	  BSP_COM_Init(comPortRead);
		BufferInit();

	//Initialize Virtual Timer (many sw timers based on one hw timer)
  //HS_STARTUP_TIME = 780uS, INITIAL_CALIBRATION = FALSE, CALIBRATION_INTERVAL =v0
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  //database initialisation
  BLEPLAT_Init();  
	 //initial PKA (Public Key Accelerator)
  if (PKAMGR_Init() == PKAMGR_ERROR)
  {
      while(1);
  }
  if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
      while(1);
  }
  /* Init the AES block */
  AESMGR_Init();
		
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
  }
	//Stack size information
	printf("TotalBuffersize: \t\t%06d\r\n",BLE_STACK_InitParams.TotalBufferSize);
	printf("NumAttrRecords: \t\t%06d\r\n",BLE_STACK_InitParams.NumAttrRecords);
	printf("MaxNumOfClientProcs: \t%06d\r\n",BLE_STACK_InitParams.MaxNumOfClientProcs);
	printf("NumOfLinks: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfLinks);
	printf("NumOfEATTChannels: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfEATTChannels);
	printf("NumBlockCount: \t\t%06d\r\n",BLE_STACK_InitParams.NumBlockCount);
	printf("ATT_MTU: \t\t\t%06d\r\n",BLE_STACK_InitParams.ATT_MTU);
	printf("MaxConnEventLength: \t%06d\r\n",BLE_STACK_InitParams.MaxConnEventLength);
	printf("SleepClockAccuracy: \t%06d\r\n",BLE_STACK_InitParams.SleepClockAccuracy);
	printf("NumOfAdvDataSet: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfAdvDataSet);
	printf("NumOfAuxScanSlots: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfAuxScanSlots);
	printf("WhiteListSizeLog2: \t\t%06d\r\n",BLE_STACK_InitParams.WhiteListSizeLog2);
	printf("L2CAP_MPS: \t\t%06d\r\n",BLE_STACK_InitParams.L2CAP_MPS);
	printf("L2CAP_NumChannels: \t\t%06d\r\n",BLE_STACK_InitParams.L2CAP_NumChannels);	
	printf("NumOfSyncSlots: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfSyncSlots);
	printf("CTE_MaxNumAntennaIDs: \t%06d\r\n",BLE_STACK_InitParams.CTE_MaxNumAntennaIDs);
	printf("CTE_MaxNumIQSamples: \t%06d\r\n",BLE_STACK_InitParams.CTE_MaxNumIQSamples);
	printf("isr0_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.isr0_fifo_size);
	printf("isr1_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.isr1_fifo_size);
	printf("user_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.user_fifo_size);
/*	
    .BLEStartRamAddress = (uint8_t*)dyn_alloc_a,                                \
    .TotalBufferSize = DYNAMIC_MEMORY_SIZE,                                     \
    .NumAttrRecords = NUM_GATT_ATTRIBUTES,                                      \
    .MaxNumOfClientProcs = NUM_OF_CONCURRENT_GATT_CLIENT_PROC_CONF,             \
    .NumOfLinks = NUM_LINKS,                                                    \
    .NumOfEATTChannels = NUM_EATT_CHANNELS,                                     \
    .NumBlockCount = MBLOCKS_COUNT,                                             \
    .ATT_MTU = MAX_ATT_MTU_CONF,                                                \
    .MaxConnEventLength = MAX_CONN_EVENT_LENGTH_CONF,                           \
    .SleepClockAccuracy = SLEEP_CLOCK_ACCURACY,                                 \
    .NumOfAdvDataSet = NUM_ADV_SETS_CONF,                                       \
    .NumOfAuxScanSlots = NUM_AUX_SCAN_SLOTS_CONF,                               \
    .WhiteListSizeLog2 = WHITE_LIST_SIZE_LOG2_CONF,                             \
    .L2CAP_MPS = L2CAP_MPS_CONF,                                                \
    .L2CAP_NumChannels = NUM_L2CAP_COCS_CONF,                                   \
    .NumOfSyncSlots = NUM_SYNC_SLOTS_CONF,                                      \
    .CTE_MaxNumAntennaIDs = MAX_NUM_CTE_ANTENNA_IDS,                            \
    .CTE_MaxNumIQSamples = MAX_NUM_CTE_IQ_SAMPLES, 		         	\
    .isr0_fifo_size = ISR0_FIFO_SIZE,                                           \
    .isr1_fifo_size = ISR1_FIFO_SIZE,                                           \
    .user_fifo_size = USER_FIFO_SIZE 
*/	
//PB0 --- Indication event and confirmation send
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
		LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS0, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS0, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_PUSHPULL);
		
//PA13 --- Read data from buffer
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
		LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS13, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS13, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_PUSHPULL);
		
//PA1 --- 
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
		LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS1, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS1, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_PUSHPULL);				

//LED		
//PA6 --- BLUE
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
		LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS6, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS6, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_PUSHPULL);
		
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH);
//PB8 --- GREEN
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
		LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS8, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS8, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_PUSHPULL);
		
		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_HIGH);		
//PB9 --- RED
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
		LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS9, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS9, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_PUSHPULL);
		
		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_HIGH);			
		
		LED_GREEN_OFF();
		LED_RED_ON();
  //Initialize Bluetooth Controller
  BLECNTR_InitGlobal();
	//
	bluetoothSetting();
	//startScanning();

	sysTickMem = Clock_Time();
	
//	ScanListInit(&scanList[1]);
	ScanListInit(&scanList);
//ZAO - start test section
	uint8_t status;
	uint16_t suggestedMaxTxOctets = 251;// bytes
	uint16_t suggestedMaxTxTime = (251 + 14)*8;//microsecunds
	status = hci_le_write_suggested_default_data_length(suggestedMaxTxOctets,
                                                      suggestedMaxTxTime);
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function: hci_le_write_suggested_default_data_length()\r\n");
		printf("return error code: 0x%02x\r\n", status);
	}
	else
	{
		printf("====Function: hci_le_write_suggested_default_data_length()\r\n");
		printf("return OK\r\n");
		
		uint16_t supportedMaxTxOctets = 0;
		uint16_t supportedMaxTxTime = 0;
		uint16_t supportedMaxRxOctets = 0;
		uint16_t supportedMaxRxTime = 0;
		
		status = hci_le_read_maximum_data_length(&supportedMaxTxOctets,
																				&supportedMaxTxTime,
																				&supportedMaxRxOctets,
																				&supportedMaxRxTime);
		if(status != BLE_STATUS_SUCCESS)
		{
			printf("====Function: hci_le_read_maximum_data_length()\r\n");
			printf("return error code: 0x%02x\r\n", status);			
		}
		else
		{
			printf("====Function: hci_le_read_maximum_data_length()\r\n");
			printf("MaxTxOctets:%03d, maxTxTime:%05d, maxRxOctets:%03d, maxRxTime:%05d\r\n",
						 supportedMaxTxOctets,
						 supportedMaxTxTime,
						 supportedMaxRxOctets,
						 supportedMaxRxTime);
		}
																				
		
		
	}
//ZAO - end test section	
	
//==============================================================================	
	//Main loop
	while(1)
	{
//		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_HIGH);
		HAL_VTIMER_Tick();
		BLE_STACK_Tick();
		NVMDB_Tick();
//		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_LOW);

	//ModulesTick();
		PacketHandler();
		applicationTick();
		if(comRxBuffer.readBufferFlag == 1)
		{
//			ReadMessageFromRx(&comRxBuffer);
		}
		
	
	}

}
//==============================================================================
void StateMachine(state_machine_t *current_state)
{
	switch(*current_state)
	{
	//========		
	case SCANNING:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				ScanListClear(&scanList);
				StartScanningStatePrint();

				scanOn = 1;
				StartScanning();
				GetLinksStatus(&linksStatusList);
				LinksStatisPrint(&linksStatusList);
				sysTickMem = Clock_Time();
			}

			if(Clock_Time() >= (sysTickMem + 1500))
			{
				state = STOP_SCANNING;
				sysTickMem = Clock_Time();
				firstEntryToState = 1;				
			}
			break;
		}
		//=========
	case STOP_SCANNING:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				StopScanning();
				sysTickMem = Clock_Time();				
			}
			if(Clock_Time() >= (sysTickMem + 100))
			{
//				ScanListPrint(&scanList);
				GetLinksStatus(&linksStatusList);
				LinksStatisPrint(&linksStatusList);
				ScanListPrint(&scanList);

				state = WAITING_CONNECT;
				firstEntryToState = 1;
			}			
			break;
		}
		//=========
	case WAITING_CONNECT:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				WaitingConnectStatePrint();
				//clear message and wait enter value;
			}
			if(ReadMessageFromRx(&comRxBuffer, rxMessage) == 1)
			{
				printf("You write: %s\r\n", rxMessage);
				uint8_t length = (uint8_t)strlen(rxMessage);
				connectionNumber = strToUInt16(rxMessage);
				printf("Conputed Number is: %03d\r\n", connectionNumber);				
				//test for symbol "s"
				if((length == 1) && ((rxMessage[0] == 'S') || (rxMessage[0] == 's')))
				{
					sysTickMem = Clock_Time();
					printf("Go to scanning mode...\r\n");	
				
					while(Clock_Time() <= (sysTickMem + 1000))				
					{
					}
					state = SCANNING;
					firstEntryToState = 1;	
				//test for num	
				}else if((strIsNum(rxMessage) && \
								(strToUInt16(rxMessage) <= scanList.nextElement - 1)) && \
								 scanList.nextElement != 0)
				{
					connectionNumber = strToUInt16(rxMessage);
					state = CONNECTING;
					firstEntryToState = 1;
					
				} else
				{
					printf("Uncorrect input valie.\r\n");
					//print menu again
					firstEntryToState = 1;					
				}
			}

			break;
		}
		//=========
	case CONNECTING:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				uint8_t deviceAddress[6] = {0};
				//WaitingConnectStatePrint();
				uint8_t status;
					for(uint8_t i = 0; i < 6; i++)
					{
						deviceAddress[i] = scanList.deviceInfo[connectionNumber].address[5 - i];
					}
						printf("Connecting to the device: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
								 deviceAddress[0], deviceAddress[1],deviceAddress[2],deviceAddress[3],
								 deviceAddress[4],deviceAddress[5]);
					
#ifdef ACI_GAP_CONNECTION		
					
						status = aci_gap_create_connection(scanList.deviceInfo[connectionNumber].primPHY,
																scanList.deviceInfo[connectionNumber].addressType,
																deviceAddress);
				if(status != BLE_STATUS_SUCCESS)
									
					{
						printf("====Function: aci_gap_create_connection() error: 0x%02x,\r\n", status);	
					} else
					{
						printf("====Function: aci_gap_create_connection() OK\r\n");	
					}
#else				

				if(status != BLE_STATUS_SUCCESS)
					{
						printf("====Function: hci_le_create_connection() error: 0x%02x,\r\n", status);	
					} else
					{
						printf("====Function: hci_le_create_connection() OK\r\n");	
					}																					
#endif //ACI_GAP_CONNECTION
				GetLinksStatus(&linksStatusList);
				LinksStatisPrint(&linksStatusList);
			}
			
			
			break;
		}
		//=========
	case CONNECTED:
		{
			//status CONNECTED setted by hci_le_connection_complete_event()
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				printf("State = CONNECTED\r\n");
				connDevListPrint(&connDevList);
				state = READ_PRIMARY_SERVICE;
//				state = FEAT_EXCHANGE;				
				firstEntryToState = 1;		
			}
			break;
		}
		//=========
	case FEAT_EXCHANGE:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				printf("State = FEAT_EXCHANGE\r\n");
				uint8_t status;
				status = hci_le_read_remote_used_features(connDevList.connDev[0].
																				 connectionHandle);
				if(status != BLE_STATUS_SUCCESS)
				{
					printf("====Function: hci_le_read_remote_used_features()\r\n");
					printf("Error code: 0x%02x\r\n", status);			
				}
				else
				{
					printf("Status: OK\r\n");
				}
			}
			if(featureExchangedFlag == 1)
			{
			featureExchangedFlag = 0;
			state = SET_DATA_LENGHT;				
			firstEntryToState = 1;									
			}
			break;
		}
		
		
		
		
		//=========		
	case SET_DATA_LENGHT:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				uint8_t status;
				status = hci_le_set_data_length(connDevList.connDev[0].
																				 connectionHandle,
																				 251, ((251 + 14) * 8));
				
				if(status != BLE_STATUS_SUCCESS)
				{
					printf("====Function: hci_le_set_data_length() error: 0x%02x\r\n", status);
				}
				else
				{
					printf("Function: hci_le_set_data_length() OK\r\n");
				}
			}
			state = READ_PRIMARY_SERVICE;
			firstEntryToState = 1;
			break;
		}
		//=========		
	case DISCONNECTION:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				DisconnectDevice();
			}
			break;
		}
		//=========		
	case AUTOCONNECT:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				sysTickMem = Clock_Time();
		
				
#ifdef AUTOCONNECT_PHY_1M
				uint8_t PHY = LE_1M_PHY_BIT;				
#else
				uint8_t PHY = LE_CODED_PHY_BIT
#endif	//AUTOCONNECT_PHY_1M
				uint8_t addressType = 0x00;//public address
				uint8_t status;
				printf("Start autoconnect to 02:80:E1:00:00:A2\r\n");
				//device address 02:80:e1:00:00:a2
				uint8_t deviceAddress[6] = {0xA2, 0x00, 0x00, 0xE1, 0x80, 0x02};
				status = aci_gap_create_connection(PHY,
																					 addressType,
																					 deviceAddress);
				if(status != BLE_STATUS_SUCCESS)
									
					{
						printf("====Function: aci_gap_create_connection() error: 0x%02x,\r\n", status);	
					} else
					{
						printf("====Function: aci_gap_create_connection() OK\r\n");	
					}
				
			}
			if(Clock_Time() >= sysTickMem + 1000)
			{
				state = RECONNECT;
				firstEntryToState = 1;
			}		
			break;
		}		
		//=========	
	case RECONNECT:
		{
			//memmorize system mime
			if(firstEntryToState == 1)
			{
				printf("Reconect attemption\r\n");
				state = AUTOCONNECT;
				firstEntryToState = 1;
			}
			break;
		}
		//=========
		
	case READ_PRIMARY_SERVICE:	
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				GetPrimaryService(&connDevList, 0);
			}
			
			break;
		}
		//=========		
	case READ_CHARACTERISTICS:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				GetCharactristics();
			}
			break;
		}
		//=========	
	case READ_DESCRIPTORS:
		{
			if(firstEntryToState == 1)
			{
				firstEntryToState = 0;
				GetDescriptors();
			}
			break;
		}
			//=========	
		case READ_VALUES:
			{
				static uint8_t UUIDCounter = 0;
				static uint8_t allUUIDReadDoneFlag = 0;
				if(firstEntryToState == 1)
				{
					firstEntryToState = 0;
					readDoneFlag = 1;
					printf("Start reading values...\r\n");
					//ReadValue(0x0006);//client supported features	
				}
				
				if(allUUIDReadDoneFlag != 1)
				{
					if(readDoneFlag == 1)//check read done for start next read
					{
						readDoneFlag = 0;
					if(UUIDCounter <= 20)//check completetion of array handling
						{
						ReadValue(readValueList[UUIDCounter]);//deviceName
						UUIDCounter ++;	
						} else
						{
							allUUIDReadDoneFlag = 1;
						}
					}
				}
				

				break;
			}
			//=========
	case GET_NOTIFICATION:
		{
				if(firstEntryToState == 1)
				{
					firstEntryToState = 0;
					printf("Make motification...\r\n");
					GetNotification();
					//
					packetHandlerSettings_t settings;
					settings.cltOrSrv = CLIENTS;
					settings.notificationType = NOTIFICATION;
					PacketHandlerInit(connDevList.connDev[0].connectionHandle,
														NULL,
														settings.cltOrSrv,
														settings.notificationType,
														phRxBuffer);
					PacketHandlerRxDoneSetCallback(&receptionDataDone);
					PacketHandlerSetRxBuffer(phRxBuffer);
				}
			break;
		}
			//=========		
	case READ_MTU:
		{
			if(firstEntryToState == 1)
			{
				uint8_t status;
				firstEntryToState = 0;
				printf("Start to read MTU from server\r\n");
				GetMaxMTU();
				#ifdef ENABLE_RADIO_ACTIVITY_EVENT
				
				status = aci_hal_set_radio_activity_mask(0x00FF);
				if(status == BLE_STATUS_SUCCESS)
				{
					printf("=========Radio activity event enable!!!\r\n");
				}
				#endif

				
//test - set 
				status = hci_le_set_phy(connDevList.connDev[0].connectionHandle,
																0x00, //preference for both data direction 
																PHY_SPEED, //LE_2M for TX
																PHY_SPEED, //LE_2M for RX 
																PHY_OPTION); // no coded
				if(status != BLE_STATUS_SUCCESS)
				{
					printf("====hci_le_set_phy() error: 0x%02x\r\n", status);
				} else
				{
					printf("====hci_le_set_phy() OK LE_2M\r\n");
				}		
//endtest				
				uint8_t txPHY = 0;
				uint8_t rxPHY = 0;
				status = hci_le_read_phy(connDevList.connDev[0].connectionHandle,
																 &txPHY, &rxPHY);
					if(status != BLE_STATUS_SUCCESS)
				{
					printf("====hci_le_read_phy() error: 0x%02x\r\n", status);
				} else
				{
					printf("====hci_le_read_phy() OK , RxPHY = 0x%02x, TxPHY = 0x%02x\r\n",
								 txPHY,rxPHY);
				}				
				//			

				//read preffered time a
				uint16_t suggestedMaxTxOctets = 0;
				uint16_t suggestedMaxTxTime = 0;
				status = hci_le_read_suggested_default_data_length(&suggestedMaxTxOctets,
																													 &suggestedMaxTxTime);
				if(status != BLE_STATUS_SUCCESS)
				{
					printf("==== Function: hci_le_read_suggested_default_data_length()\r\n");
					printf("return error code: 0x%02x\r\n", status);					
				}
				else
				{
					printf("==== Function: hci_le_read_suggested_default_data_length()\r\n");
					printf("return OK. MaxTxOctets: %04d, MaxTxTime: %04d\r\n",
								 suggestedMaxTxOctets,
								 suggestedMaxTxTime);	
				}
				
			}
			
			break;
		}
			//=========
	case READ_DATA:
		{
				if(firstEntryToState == 1)
				{
					firstEntryToState = 0;
					printf("Ready to read data from server thorogh notification...\r\n");
				}
			break;			
		}
		
	}//end switch
}


//aci_gap_create_connection
//tBleStatus aci_gap_create_connection(uint8_t Initiating_PHY,
//                                     uint8_t Peer_Address_Type,
//                                     uint8_t Peer_Address[6]);

//==============================================================================
void applicationTick(void)
{	
	StateMachine(&state);
}
//==============================================================================	
void bluetoothSetting(void)
{
	uint16_t service_handle;
	uint16_t device_name_char_handle;
	uint16_t appearance_char_handle;
	tBleStatus status; 
	uint8_t name[] = {"BLE_CEN"};
	linksStatusList.bankIndex = 0;
	connDevList.nextElement = 0;
	//events mask
/*	uint8_t le_event_mask[] = {
		0x00 |	//byte 0
		HCI_LE_EVT_MASK_BYTE0_CONNECTION_COMPLETE 
	| HCI_LE_EVT_MASK_BYTE0_ADVERTISING_REPORT,
		0x00		//byte 1
	| HCI_LE_EVT_MASK_BYTE1_EXTENDED_ADVERTISING_REPORT,
		0x00,		//byte 2
		0x00,		//byte 3
		0x00,		//byte 4
		0x00,		//byte 5
		0x00,		//byte 6
		0x00,		//byte 7		
	};
*/	
  uint8_t le_event_mask[] = {
		//byte 0
		0x00 |
		HCI_LE_EVT_MASK_BYTE0_CONNECTION_COMPLETE |
		HCI_LE_EVT_MASK_BYTE0_READ_REMOTE_FEATURES_COMPLETE|
		HCI_LE_EVT_MASK_BYTE0_REMOTE_CONNECTION_PARAMETER_REQUEST ,
		//byte 1
		0x00 |
    HCI_LE_EVT_MASK_BYTE1_EXTENDED_ADVERTISING_REPORT | 
    HCI_LE_EVT_MASK_BYTE1_PERIODIC_ADVERTISING_SYNC_ESTABLISHED |
    HCI_LE_EVT_MASK_BYTE1_PERIODIC_ADVERTISING_REPORT |
    HCI_LE_EVT_MASK_BYTE1_PERIODIC_ADVERTISING_SYNC_LOST,
    HCI_LE_EVT_MASK_BYTE2_CONNECTIONLESS_IQ_REPORT,
		//byte 2
    0x00,
		//byte 3
		0x00,
		//byte 4
		0x00,
		//byte 5
		0x00,
		//byte 6
		0x00};
	
	//third argumens is device name size
	status = aci_gap_init(GAP_CENTRAL_ROLE ,
												0x00, 
												0x07, 
												PUBLIC_ADDR, 
												&service_handle, 
												&device_name_char_handle, 
												&appearance_char_handle);
	if(status != BLE_STATUS_SUCCESS)
	{printf("====Function: aci_gap_init() failed:0x%02x\r\n", status);
	}else
	{
		printf("====Function: aci_gap_init() succsess\r\n");
	}
	//set event mask
	status = hci_le_set_event_mask(le_event_mask);
	if(status !=BLE_STATUS_SUCCESS)
		{printf("====Function: hci_le_set_event_mask() failed:0x%02x\r\n", status);
		}else
		{printf("====Functon: hci_le_set_event_mask() success\r\n");
	}
	//set scan configuraton
	uint8_t scan_filter = SCAN_ACCEPT_ALL;
	status = aci_gap_set_scan_configuration(DUPLICATE_FILTER_DISABLED, 
																					scan_filter, 
																					LE_1M_PHY_BIT, 
																					PASSIVE_SCAN,
																					SCAN_INTERVAL_MS*1000/625,
																					SCAN_WINDOW_MS*1000/625);
	if(status != BLE_STATUS_SUCCESS)
	{printf("====Function: aci_gap_set_scan_configuration() failed:0x%02x\r\n", status);
	}else
	{printf("====Function: aci_gap_set_scan_configuration() success\r\n");
	}
	//set connection configuration
	/*status = aci_gap_set_connection_configuration(LE_1M_PHY_BIT,
																								CONN_INTERVAL_MIN,
																								CONN_INTERVAL_MAX,
																								0,
																								SUPERVISION_TIMEOUT,
																								CE_LENGTH,
																								CE_LENGTH);
	*/
	status = aci_gap_set_connection_configuration(LE_1M_PHY_BIT,
																								CONN_INTERVAL_MIN,
																								CONN_INTERVAL_MAX,
																								SLAVE_LATENCY,
																								SUPERVISION_TIMEOUT,
																								CE_LENGTH_MIN,
																								CE_LENGTH_MAX);	
	
	
	
//	aci_gap_set_event_mask(0xFFFF & (~0x0080));// test set for request name
	
}
//==============================================================================	
void StartScanning(void)
{
	tBleStatus status;
	status = aci_gap_start_procedure(GAP_OBSERVATION_PROC,
																	 LE_1M_PHY_BIT,
																	 0x0000,
																	 0x0000);
	if(status != BLE_STATUS_SUCCESS)
	{	printf("====Function: aci_gap_start_procedure() failed: 0x%02x\r\n", status);
	}else
	{	printf("====Function: aci_gap_start_procedure() success\r\n");
	}	
}
//==============================================================================	
void StopScanning(void)
{
	tBleStatus status;
	status = aci_gap_terminate_proc(GAP_OBSERVATION_PROC);
	if(status != BLE_STATUS_SUCCESS)
	{	printf("====Function: aci_gap_terminate_proc() failed: 0x%02x\r\n", status);
	}else
	{	printf("====Function: aci_gap_start_procedure() success\r\n");
	}
}
//==============================================================================
void MakeConnection(list_t scanList, uint16_t number)
{
//	aci_gap_create_connection(
}


//==============================================================================
//void aci_gap_proc_complete_event(uint8_t Procedure_Code,
//                                 uint8_t Status,
//                                 uint8_t Data_Length,
//                                 uint8_t Data[])
//{
//	if(Procedure_Code == GAP_NAME_DISCOVERY_PROC)
//	{
//		printf("Name responce. Device data is: %s \r\n", Data);
//	}
	
//}

//==============================================================================
void GetLinksStatus(listLinks_t *linkList)
{
	uint8_t status;
	status = aci_hal_get_link_status(linkList->bankIndex,
													linkList->linkStatus,
													linkList->LinkConnectionHandle);
			
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function: aci_hal_get_link_status() error code: 0x%02x\r\n", status);
	}else
	{
		printf("====Function: aci_hal_get_link_status() OK\r\n");
	}
}
//==============================================================================
void GetPrimaryService(connDevList_t *connDevList, uint8_t order)
{	 
	uint8_t status =	aci_gatt_clt_disc_all_primary_services(connDevList->connDev[order].
																				 connectionHandle);
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function: aci_gatt_clt_disc_all_primary_services() error code: 0x%02x\r\n",status);		
	}else
	{
		printf("====Function: aci_gatt_clt_disc_all_primary_services() OK\r\n");
	}
		
	//response in aci_att_clt_read_by_group_type_resp_event()
	
}
//==============================================================================
uint8_t strIsNum(uint8_t string[])
{
	uint8_t length = strlen((const char*)string);
	for(uint8_t i = 0; i < length; i++)
	{
		if(!(string[i] >= 0x30 && string[i] <= 0x39))
		{
			return 0;
		}
		return 1;
	}
	
}
//==============================================================================
uint16_t	strToUInt16(uint8_t string[])
{
	uint16_t number = 0;
	uint32_t base = 1;
	uint8_t length = strlen((const char*)string);
	for(uint8_t i = length; i > 0; i--)
	{
		number = number + base * ( ((uint8_t)string[i-1]) - 0x30);
		base = base * 10;
	}
	return number;
}
//==============================================================================					 
void printScaningData(uint16_t adv_data_len, uint8_t * adv_data, char * head_string)
{
	
}
//==============================================================================
void GetCharactristics(void)
{
	uint8_t status;
	status = aci_gatt_clt_disc_all_char_of_service(connDevList.connDev[0].connectionHandle,
																				0x0001,
																				0xFFFF);
	/*status = aci_gatt_clt_disc_all_char_of_service(connDevList.connDev[0].connectionHandle,
																				0x1000,
																				0x1500);	
	*/
	
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function: aci_gatt_clt_disc_all_char_of_service() return error: 0x%02x\r\n", status);		
	} else
	{
		printf("====Function: aci_gatt_clt_disc_all_char_of_service() OK\r\n");
	}
	
}
//==============================================================================
void GetDescriptors(void)
{
	uint8_t status;
	status = aci_gatt_clt_disc_all_char_desc(connDevList.connDev[0].connectionHandle,
																					 0x0001,
																					 0xFFFF);
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function: aci_gatt_clt_disc_all_char_desc() return error: 0x%02x\r\n", status);		
	} else
	{
		printf("====Function: aci_gatt_clt_disc_all_char_desc() OK\r\n");
	}	
}
//==============================================================================
void DisconnectDevice(void)
{
	uint8_t status;
	status = hci_disconnect(connDevList.connDev[0].connectionHandle,
													0x13); //reason = 0x13 remote device terminate connection
		if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function: hci_disconnect() return error: 0x%02x\r\n", status);		
	} else
	{
		printf("====Function: hci_disconnect() OK\r\n");
	}	
	
}		
//==============================================================================
void ReadValue(uint16_t UUID)
{
	uint8_t status;
	status = aci_gatt_clt_read(connDevList.connDev[0].connectionHandle,
										UUID);
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function aci_gatt_clt_read(0x%04x)return error: 0x%02x\r\n",
					 UUID, status);
	}else
	{
		printf("====Function aci_gatt_clt_read(0x%04x) OK\r\n", UUID);
	}
}
//==============================================================================
void GetNotification(void)
{
	uint8_t status;
	//indication
	uint8_t data[] = {0x01,0x00};//{0x01,0x00} notificaation
//	uint8_t data[] = {0x02,0x00};//{0x01,0x00} indication
	status =  aci_gatt_clt_write(connDevList.connDev[0].connectionHandle,
															 0x013,// TX_Handle = 0x0011 + 2 offset to 0x0013 (UUID = 0x2902 CCC)
															 2,// 2 bytes
															 data);
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====aci_gatt_clt_write()return error: 0x%02x\r\n",
					 status);
	}else
	{
		printf("====aci_gatt_clt_write() OK\r\n");
	}
}
//==============================================================================
void GetMaxMTU(void)
{
	uint8_t status;	
	status = aci_gatt_clt_exchange_config(connDevList.connDev[0].connectionHandle);
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====aci_gatt_clt_exchange_config()return error: 0x%02x\r\n",
					 status);
	}else
	{
		printf("====aci_gatt_clt_exchange_config() OK\r\n");
	}	
	//aci_att_exchange_mtu_resp_event is callback
}
//==============================================================================
void ReadDataFromBufferToBuffer(uint8_t *sourceBuffer,
																	 uint8_t *destinationBuffer,
																	 uint16_t Lenght)
{
LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_HIGH);	
	for(uint16_t i = 0; i < Lenght; i++)
		{
			destinationBuffer[i] = sourceBuffer[i];
		}	
	
				uint8_t txPHY = 0;
				uint8_t rxPHY = 0;
				
//				uint8_t status = hci_le_read_phy(connDevList.connDev[0].connectionHandle,
//																 &txPHY, &rxPHY);
//					if(status != BLE_STATUS_SUCCESS)
//				{
//					printf("====hci_le_read_phy() error: 0x%02x\r\n", status);
//				} else
//				{
//					printf("====hci_le_read_phy() OK , RxPHY = 0x%02x, TxPHY = 0x%02x\r\n",
//								 txPHY,rxPHY);	
//				}

LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_LOW);	
}
//==============================================================================
void receptionDataDone(void)
{
//	printf("Data has been read through packetHandler\r\n");
	LED_BLUE_ON();
	int8_t RSSI;
	int8_t transmitPowerLevel;
	int8_t powerLost;
	
//   uint16_t num_pkts;          /**< The number of received packets, valid or with CRC errors. */
//   uint16_t num_crc_err;       /**< The number of packets received with CRC errors. */
//    uint16_t num_evts;          /**< The number of past connection events, including missed ones. */
//    uint16_t num_miss_evts;     /**< The number of missed RX packets, because of RX timeout or invalid packet length. */	
	//function with NUUL pointer will be stop statistic
	llc_conn_per_statistic(connDevList.connDev[0].connectionHandle,
												 NULL);

	hci_read_transmit_power_level(connDevList.connDev[0].connectionHandle,
                                         0,
                                         &transmitPowerLevel);
	transmitPowerLevel = 8;//input power power is +8dBm instead 6
	
	hci_read_rssi(connDevList.connDev[0].connectionHandle, &RSSI);

//	powerLost = transmitPowerLevel - RSSI;
	powerLost = transmitPowerLevel - RSSI;
	//RSSI = RSSI - 128;
	printf("Read data, T:%06dmS, RSSI:%04ddBm, SrvPower:%04ddBm, PwrLost:%04ddBm\r\n",
				 packetHandlerRxStatistic.timeSpent,
				 RSSI,
				 transmitPowerLevel,
				 powerLost);
	printf("Statistic. num_pkts:%05d, num_crc_err:%05d, num_evts:%05d, num_miss_evts:%05d\r\n",
				 statisticPerConnection.num_pkts,
				 statisticPerConnection.num_crc_err,
				 statisticPerConnection.num_evts,
				 statisticPerConnection.num_miss_evts);
	allTimeStatistic.allPackets += statisticPerConnection.num_pkts;
	allTimeStatistic.allBadCRCPackets += statisticPerConnection.num_crc_err;
	allTimeStatistic.allTime += packetHandlerRxStatistic.timeSpent;
	allTimeStatistic.allDataCounter ++;
	printf("============ ALL TIME STATISTIC =========== \r\n");
	printf("Data quantity:\t%020lld\r\n", allTimeStatistic.allDataCounter);	
	printf("Packets:\t\t%020lld\r\n", allTimeStatistic.allPackets);
	printf("Bad CRC packets:\t%020lld\r\n", allTimeStatistic.allBadCRCPackets);
	printf("Sum time:\t\t%020lld\r\n", allTimeStatistic.allTime);
	printf("Lost connections:\t%020lld\r\n", allTimeStatistic.allConnectionLost);
	printf("=========================================== \r\n");
	
	
	
		
	//function with pointer to data will be zeroing values	
	llc_conn_per_statistic(connDevList.connDev[0].connectionHandle,
												 &statisticPerConnection);

	
		LED_BLUE_OFF();
}
//==============================================================================
void ConnDevListEntryDeliteByConnectionHandle(connDevList_t *list,
																							uint16_t connectionHandle)
{
	for(uint8_t i = 0; i < CONNECTED_DEV_LIST_SIZE; i++)
	{
		if(list->connDev[i].connectionHandle ==  connectionHandle)
		{
			ConnDevListEntryDeliteByOrder(list, 
																		list->connDev[i].order);
		}
	}
}
//==============================================================================
void ConnDevListEntryDeliteByOrder(connDevList_t *list, uint8_t order)
{	
	list->connDev[order].order = 0;
	list->connDev[order].peerAddressType = 0;
	for(uint8_t i = 0; i < 6; i++)
	{
			list->connDev[order].peerAddress[i] = 0;
	}
	list->connDev[order].connectionHandle = 0;
	list->connDev[order].role = 0;
	list->connDev[order].connInterval = 0;
	list->connDev[order].connLatency = 0;		
	list->connDev[order].supervisionTimeout = 0;
	list->connDev[order].masterClockAccuracy = 0;
	list->nextElement--;		
}
//==============================================================================