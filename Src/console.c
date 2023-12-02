#include "console.h"
#include "ble_const.h"
#include <stdint.h>
#include "main.h"

extern rxBuffer_t comRxBuffer;

//==============================================================================
void ScanListInit(list_t *scanList)
{
	for(uint8_t i = 0; i <= SCAN_LIST_SIZE; i ++)
		{
			//clear order data
			scanList->deviceInfo[i].order = 0;
			//clear address data
			for (uint8_t k = 0; k <= 5; k++)
			{
				scanList->deviceInfo[i].address[k] = 0;
			}
			//clear rssi data
			scanList->deviceInfo[i].rssi = 0;
			//clear laste element data
			scanList->nextElement = 0;
		}			
}
//==============================================================================
void ScanListClear(list_t *scanList)
{
	ScanListInit(scanList);
}
//==============================================================================
uint8_t ScanListUpdate(list_t *scanList, Extended_Advertising_Report_t *(data))
{
	uint8_t result;
	uint8_t dataAddress[6];
	uint8_t checkResult = 1;
	//read address from report MSB to LSB
	for (uint8_t i = 0; i <= 5; i++)
	{
		dataAddress[i] = data->Address[5-i];
	}
	uint8_t count = scanList->nextElement;
	//check new address
	for(uint8_t i = 0; i <= count; i++)
		{
			checkResult = AddressCheck(scanList->deviceInfo[i].address, dataAddress);
			//if data from report equal any addresses on the scan table - go out
			if(checkResult == 1)
			{
				return 0;//not new address
			}
		}
	//add new data if we read new address
	if(checkResult != 1)
	{
		//add address device
		for(uint8_t i = 0; i<=5; i++)
		{
			scanList->deviceInfo[(scanList->nextElement)].address[i] = dataAddress[i];			
		}
		//add rssi data
		scanList->deviceInfo[(scanList->nextElement)].rssi = data->RSSI;
		//add address type data
		scanList->deviceInfo[scanList->nextElement].addressType = data->Address_Type;
		//add primary PHY
		scanList->deviceInfo[scanList->nextElement].primPHY = data->Primary_PHY;
		//add secondary PHY
		scanList->deviceInfo[scanList->nextElement].secPHY = data->Secondary_PHY;
		
	
		
		//increment nextElement value
		scanList->nextElement++;
	}
	
	
	return result;
}
//==============================================================================
uint8_t AddressCheck(uint8_t deviceAddress[6], uint8_t dataAddress[6])
{
	uint8_t result = 0;
	
	for(uint8_t i = 0;i <=5; i++)
	{
		if(deviceAddress[i] != dataAddress[i])
		{
			return 0;
		}
	}
	result = 1;
	return result;
}
//==============================================================================
void StartScanningStatePrint(void)
{
	printf("=====================================================================\r\n");
	printf("| START SCANING BLUETOOTH DEVICES...                                |\r\n");
	printf("=====================================================================\r\n");
}
//==============================================================================
void WaitingConnectStatePrint(void)
{
	printf("=====================================================================\r\n");
	printf("| WAITING CONNECTION COMMAND...                                     |\r\n");
	printf("| Print number device you want to connect or print 'S' to           |\r\n");	
	printf("| repeat the scan.                                                  |\r\n");		
	printf("=====================================================================\r\n");	
}
//==============================================================================
void connDevListPrint(connDevList_t *connDevList)
{
	printf("=====================================================================\r\n");
	printf("| CONNECTION DEVICE TABLE                                           |\r\n");
	printf("=====================================================================\r\n");
	printf("|Nr| HIdn |AdrT|  Device Address |Role|ConnInt|ConnLt|SupT |MrClkAcc|\r\n");	
	printf("=====================================================================\r\n");	
	for(uint8_t i = 0; i < connDevList->nextElement; i++)
	{
		//print order
		printf("|%02d", connDevList->connDev[i].order);
		//print handle identifier
		printf("|0x%04x",connDevList->connDev[i].connectionHandle);
		//print peer addres type
		if(connDevList->connDev[i].peerAddressType == 0x00)
		{
			printf("|Pblc");
		} else
		{
			printf("|Rund");
		}
		//print peer MAC address
		printf("|%02x:%02x:%02x:%02x:%02x:%02x",
					connDevList->connDev[i].peerAddress[0],
					connDevList->connDev[i].peerAddress[1],					
					connDevList->connDev[i].peerAddress[2],					
					connDevList->connDev[i].peerAddress[3],
					connDevList->connDev[i].peerAddress[4],
					connDevList->connDev[i].peerAddress[5]);
		//print role
		if(connDevList->connDev[i].role == 0x00)
		{
			printf("|Mstr");
		}else
		{
			printf("|Slv ");	
		}
		//print connection interval in ms
		
		printf("|%7.2f",(1.25 *(float) connDevList->connDev[i].connInterval));
		//print connection latency in ms
		printf("|0x%04x", connDevList->connDev[i].connLatency);
		//print supervision timeout in ms
		printf("|%05d", 10 * connDevList->connDev[i].supervisionTimeout);
		switch(connDevList->connDev[i].masterClockAccuracy)
		{
			case 0x00:	{printf("|500     |\r\n"); break;}
			case 0x01: 	{printf("|250     |\r\n"); break;}
			case 0x02: 	{printf("|150     |\r\n"); break;}
			case 0x03: 	{printf("|100     |\r\n"); break;}
			case 0x04: 	{printf("|75      |\r\n"); break;}
			case 0x05: 	{printf("|50      |\r\n"); break;}
			case 0x06: 	{printf("|30      |\r\n"); break;}
			case 0x07: 	{printf("|20      |\r\n"); break;}			
		}	
	}
	printf("=====================================================================\r\n");
}

//==============================================================================
//==============================================================================
void LinksStatisPrint(listLinks_t *linksList)
{
	printf("=====================================================================\r\n");
	printf("| LINKS STATUS TABLE                                                |\r\n");
	printf("=====================================================================\r\n");
	printf("| Nr |    Conn. Handle     |               Link Status              |\r\n");
	printf("=====================================================================\r\n");
	//we use only one bank = 8 links 
	for (uint8_t i = 0; i < 8; i++)
	{
		printf("|  %01d ", i);
		printf("|       0x%04x        ", linksList->LinkConnectionHandle[i]);
		if(linksList->linkStatus[i] == 0x00)
		{
			printf("|                 Idle                   |\r\n");
		}
		else if(linksList->linkStatus[i] == 0x01)
		{
			printf("|              Advertising               |\r\n");			
		}
		else if(linksList->linkStatus[i] == 0x02)
		{
			printf("|           Connected as slave           |\r\n");			
		}
		else if(linksList->linkStatus[i] == 0x03)
		{
			printf("|               Scanning                 |\r\n");			
		}
		else if(linksList->linkStatus[i] == 0x04)
		{
			printf("|             Inintiating                |\r\n");			
		}
		else if(linksList->linkStatus[i] == 0x05)
		{
			printf("|         Conncectef as master           |\r\n");			
		}
		else if(linksList->linkStatus[i] == 0x06)
		{
			printf("|            TX test mode                |\r\n");			
		}
		else if(linksList->linkStatus[i] == 0x07)
		{
			printf("|            RX test mode                |\r\n");			
		}
	}
	printf("=====================================================================\r\n");		
}
//==============================================================================
void ScanListPrint(list_t *scanList)
{
	//Table header
//	uint8_t rowCount = scanList->nextElement;
	printf("=====================================================================\r\n");
	printf("| SCAN LIST TABLE                                                   |\r\n");
	printf("=====================================================================\r\n");
	printf("|Nr");	
	printf("|     ADDRESS     ");
	printf("|  RSSI  |\r\n");	
	printf("=====================================================================\r\n");
	for(uint8_t i = 0; i < (scanList->nextElement); i++)
	{	//num
		printf("|%02d|", i);
		//address
		for(uint8_t k = 0; k <=5; k++)
		{
			printf("%02x", scanList->deviceInfo[i].address[k]);		
			if(k != 5)
			{
				printf(":");
			}else
			{
				printf("|");
			}
		}
		//rssi
		printf(" %03ddBm |",scanList->deviceInfo[i].rssi);
		printf("\r\n");
	}
	printf("=====================================================================\r\n");
//	printf("| %02d ",s.num);		
}	
//==============================================================================
uint8_t ReadMessageFromRx(rxBuffer_t *buffer, uint8_t message[])
{
	if(buffer->readBufferFlag == 1)
	{
		//disable UART interrupt
  	LL_USART_Disable(BSP_UART);
//		uint8_t rxBufferLenght =(uint8_t) strlen(buffer->readBuffer);
//		uint8_t messageLenght =(uint8_t) sizeof(message);
		//clear message before read
		for(uint8_t i = 0; i <= (READ_BUFFER_SIZE - 1); i++)
		{
			message[i] = '\0';
		}
		uint8_t k = 0;
		//read whole buffer
		for(uint8_t i = 0; i <= (READ_BUFFER_SIZE - 1); i++)
		{
			if((buffer->readBuffer[i] != '\r') && 
					(buffer->readBuffer[i] != '\n') &&
					(buffer->readBuffer[i] != '\0') && 
					(buffer->readBuffer[i] != 0xF8))
			{
				message[k] = buffer->readBuffer[i];
				k++;
			
			}
			//clear buffer
			buffer->readBuffer[i] = '\0';
		}
		buffer->readBufferFlag = 0;
  	LL_USART_Enable(BSP_UART);
		return 1;
		//enable UART interrupt
	}else
	return 0;	
}


//==============================================================================
uint8_t ReadDevicesName(list_t scanList)
{
	
	static uint8_t reading = 0;
	if(reading == 0)
		{
			
			
			reading = 1;
		}
	
	
	//
	
	//if read all addresses - done status
	if(0)
	{
		return 1;
	}
}

//==============================================================================
void UartInit(void)
{
  /* Initialize the GPIOs associated to the UART port */
  BSP_UART_TX_GPIO_CLK_ENABLE();
  LL_GPIO_SetPinMode(BSP_UART_TX_GPIO_PORT, BSP_UART_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(BSP_UART_TX_GPIO_PORT, BSP_UART_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(BSP_UART_TX_GPIO_PORT, BSP_UART_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(BSP_UART_TX_GPIO_PORT, BSP_UART_TX_PIN, LL_GPIO_PULL_UP);
  BSP_UART_TX_GPIO_AF();

  BSP_UART_RX_GPIO_CLK_ENABLE();
  LL_GPIO_SetPinMode(BSP_UART_RX_GPIO_PORT, BSP_UART_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(BSP_UART_RX_GPIO_PORT, BSP_UART_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(BSP_UART_RX_GPIO_PORT, BSP_UART_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(BSP_UART_RX_GPIO_PORT, BSP_UART_RX_PIN, LL_GPIO_PULL_UP);
  BSP_UART_RX_GPIO_AF();

  /* Initialize the UART clock */
  BSP_UART_CLK_ENABLE();

  /* UART parameter configuration*/
  LL_USART_SetBaudRate(BSP_UART, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, BSP_UART_BAUDRATE);
  LL_USART_ConfigCharacter(BSP_UART, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  LL_USART_SetTransferDirection(BSP_UART, LL_USART_DIRECTION_TX_RX);
  LL_USART_SetHWFlowCtrl(BSP_UART, LL_USART_HWCONTROL_NONE);

  LL_USART_SetRXFIFOThreshold(BSP_UART, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetTXFIFOThreshold(BSP_UART, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_EnableFIFO(BSP_UART);

  LL_USART_ConfigAsyncMode(BSP_UART);
  
  /* Enable the UART */
  LL_USART_Enable(BSP_UART);

  /* If the user callback is not NULL */
  if(1) {
    /* Record the user callback for handling the RX data */
//    BSP_COM_RxDataCb.RxDataUserCb = pRxDataCb;

    /* Enable the RX not empty interrupt */
    LL_USART_EnableIT_RXNE(BSP_UART);

    /* Enable the UART IRQ */
    NVIC_SetPriority(BSP_UART_IRQn, IRQ_HIGH_PRIORITY);
    NVIC_EnableIRQ(BSP_UART_IRQn);
#if defined(__GNUC__) && !defined(__ARMCC_VERSION)
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
  }

}

