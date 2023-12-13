#include "events.h"
#include "main.h"

#define DISABLE_PRINTF
#define USE_EVENTS_FROM_PACKET_HANDLER

#define EVENTS_C_DEBUGGING

	extern list_t scanList;
	extern state_machine_t state;
	extern uint8_t scanOn;
	extern listLinks_t linksStatusList;
	extern connDevList_t connDevList;
	extern uint8_t firstEntryToState;
	extern uint8_t readDoneFlag;
	extern uint8_t testBuffer[];
	extern uint8_t rssiTestNr;
	extern uint8_t packetTestNr;
	
	uint8_t eventArray[1000];
	uint16_t eventArrayP = 0;
	
//==============================================================================
void hci_le_extended_advertising_report_event(uint8_t num_reports,
																							Extended_Advertising_Report_t extended_advertising_report[])
{
	uint16_t eventType = extended_advertising_report[0].Event_Type;

	printf("====Function: hci_le_ectended_advertising_report_event() Data come\r\n");
	printf("NumReport: %02d\r\n", num_reports);
	printf("EventType: 0x%04x\r\n", eventType);
	uint8_t addressType = extended_advertising_report[0].Address_Type;
	//====Address type
	if(addressType == 0x00)
	{
		printf("Address_Type = Public Device Address. Val: 0x%04x\r\n", addressType);
	} else if(addressType == 0x01)
	{
		printf("Address_Type = Random Device Address. Val: 0x%04x\r\n", addressType);
	} else if (addressType == 0x02)
	{
		printf("Address_Type = Public Identity Address. Val: 0x%04x\r\n", addressType);		
	} else if(addressType == 0x03)
	{
		printf("Address_Type = Random (static) Identiti Device Address. Val: 0x%04x\r\n",
					 addressType);
	} else if(addressType == 0xFF)
	{
		printf("Address_Type = No address provided. Val: 0x%04x\r\n", addressType);		
	}
	//====Address
	uint8_t address[6];
	for(uint8_t i= 0; i <= 6; i++)
	{
		address[i] = extended_advertising_report[0].Address[i];
	}
	printf("Address = \t %x:%x:%x:%x:%x:%x\r\n", address[0],address[1],address[2],
				 address[3],address[4],address[5]);
	//====Prim PHY
	uint8_t primPHY = extended_advertising_report[0].Primary_PHY;
	if(primPHY == 0x01)
	{
		printf("Primary PHY: LE_1M_PHY \t0x%02x\r\n", primPHY);
	} else if (primPHY == 0x03)
	{
		printf("Primary PHY: LE_CODED_PHY \t0x%02x\r\n", primPHY);
	}
	//====Sec PHY
	uint8_t secPHY = extended_advertising_report[0].Secondary_PHY;
	if(primPHY == 0x00)
	{
		printf("Secondary PHY: No Packets \t0x%02x\r\n", secPHY);
	} else if (primPHY == 0x01)
	{
		printf("Secondary PHY: LE_1M_PHY \t0x%02x\r\n", secPHY);
	} else if (primPHY == 0x02)
			{
		printf("Secondary PHY: LE_2M_PHY \t0x%02x\r\n", secPHY);
	} else if (primPHY == 0x03)
			{
		printf("Secondary PHY: LE_CODED_PHY \t0x%02x\r\n", secPHY);
	}
	//====Advertising SID
	uint8_t advert_SID = extended_advertising_report[0].Advertising_SID;
	if(advert_SID == 0xFF)
	{
		printf("No Ardertising SID \t0x%02x\r\n",advert_SID);
	}else printf("Advertising SID: \t0x%02x\r\n", advert_SID);
	//==== Power
	int8_t txPower = extended_advertising_report[0].TX_Power;
	int8_t rssi = extended_advertising_report[0].RSSI;
	int8_t difference = txPower - rssi;
	if((txPower == 127) && (rssi == 127))
	{
		printf("No signal information\r\n");
	}else if((txPower != 127)&&(rssi != 127))
	{
		printf("TxPower: %03d dBm, RSSi: %03d dBm. Power lost: %02d dBm\r\n", txPower, rssi, difference);
	}else if ((txPower == 127)&&(rssi != 127))
	{
				printf("TxPower: No information, RSSi: %03d dBm \r\n", rssi);
	}
	//====Periodic advertising interval
	uint16_t advertInterval = extended_advertising_report[0].Periodic_Advertising_Interval;
	float_t advertInterval_ms = 1.25 * advertInterval;
	if(advertInterval_ms == 0)
	{
		printf("No advertising interval, 0x%4x \r\n", advertInterval);
	} else
	{
		printf("Periodic advertising interval: %7.2fms" , advertInterval_ms);
	}
	//====Direct address type
	uint8_t dirAdrType = extended_advertising_report[0].Direct_Address_Type;
	if(dirAdrType == 0x00)
	{
		printf("Dir adr type: Public dev adr. 0x%02x \r\n", dirAdrType);
	} else if (dirAdrType == 0x01)
	{
		printf("Dir adr type: Random dev adr. 0x%02x \r\n", dirAdrType);
	} else if (dirAdrType == 0x02)
	{
		printf("Dir adr type: Public identify adr. 0x%02x \r\n", dirAdrType);
	} else if (dirAdrType == 0x03)
	
	{
		printf("Dir adr type: Random (static) identify adr. 0x%02x \r\n", dirAdrType);
	}else if (dirAdrType == 0xFE)
	 
	{
		printf("Dir adr type: Random device adr \
									(controller enable to resolve). 0x%02x \r\n", dirAdrType);
	}
	//====Direct address
		uint8_t dirAddress[6];
	for(uint8_t i= 0; i <= 6; i++)
	{
		dirAddress[i] = extended_advertising_report[0].Direct_Address[i];
	}
	printf("Direct Address = \t %x:%x:%x:%x:%x:%x\r\n", dirAddress[0],
																											dirAddress[1],
																											dirAddress[2],
																											dirAddress[3],
																											dirAddress[4],
																											dirAddress[5]);
	//====Data length
	uint8_t dataLenght = extended_advertising_report[0].Data_Length;
	printf("Data length: \t0x%02x\r\n", dataLenght);
	//====Data
	printf("Data: ");
	for (uint8_t i = 0; i <= dataLenght; i++)
	{
		if(i%8 == 0)
		{
			printf("\r\n");
		}
		printf("0x%02x,",extended_advertising_report[0].Data[i]);
	}
	printf("\r\n");	
	printf("============================================================\r\n");
	
	ScanListUpdate(&scanList, extended_advertising_report);
	//print the summury information after 10sec scanning
	if(state == WAITING_CONNECT)
	{
		scanOn = 0;
		//ScanListPrint(&scanList);

	}
}
//==============================================================================
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
	if(Status != 0x00)
	{
		printf("====Function: hci_le_connection_complete_event() error code: 0x%02x\r\n", Status);
		/*if autocconnect mode is enabled and attemption to connect is unsuccessfull
		 we must go to the reconnect state*/
#ifdef AUTOCONNECT_MODE
		printf("Go to reconnect\r\n");
		state = RECONNECT;
		firstEntryToState = 1;
#endif	//AUTOCONNECT_MODE
	}else
	{
		printf("====Function: hci_le_connection_complete_event() OK\r\n");	
	}
	if((state == CONNECTING) && (Status == 0x00) || ((state == AUTOCONNECT) && Status == 0x00))
	{
		state = CONNECTED;
		firstEntryToState = 1;
	}
	GetLinksStatus(&linksStatusList);
	LinksStatisPrint(&linksStatusList);
	//read order number
	connDevList.connDev[connDevList.nextElement].order = connDevList.nextElement;
	//read handler identifier
	connDevList.connDev[connDevList.nextElement].	\
		connectionHandle = Connection_Handle;
	//read role
	connDevList.connDev[connDevList.nextElement].role = Role;
	//read peer address type
	connDevList.connDev[connDevList.nextElement].	\
		peerAddressType = Peer_Address_Type;
	//read peer address
	for (uint8_t i = 0; i < 6; i++)	
	{
		connDevList.connDev[connDevList.nextElement].	\
			peerAddress[i] = Peer_Address[i];
	}
	//read connection interval
	connDevList.connDev[connDevList.nextElement].connInterval = Conn_Interval;
	//read connecton latency
	connDevList.connDev[connDevList.nextElement].connLatency = Conn_Latency;
	//read supervisor timeout
	connDevList.connDev[connDevList.nextElement].	\
		supervisionTimeout = Supervision_Timeout;
	//read master clock accuracy
	connDevList.connDev[connDevList.nextElement].
		masterClockAccuracy = Master_Clock_Accuracy;
	
	//increment valuet for new entry
	connDevList.nextElement++;
	LED_GREEN_ON();
	LED_RED_OFF();
	
}
//==============================================================================
void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 uint8_t Data[])
	{
#ifdef EVENTS_C_DEBUGGING
		printf("====Function begin: acie_gap_proc_complete_event()\r\n");
		printf("Procedure code: 0x%02x \r\n",Procedure_Code);

		if(Procedure_Code == GAP_NAME_DISCOVERY_PROC)
		{
			printf("GAP_NAME_DISCOVERY_PROC\r\n");
		}
		printf("====Function end: acie_gap_proc_complete_event()\r\n");
#endif //EVENTS_C_DEBUGGING		
	}
//==============================================================================
void aci_att_clt_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                               uint8_t Attribute_Data_Length,
                                               uint16_t Data_Length,
                                               uint8_t Attribute_Data_List[])
{
	printf("====Function begin: aci_att_clt_read_by_group_type_resp_event()\r\n");
	if(state == READ_PRIMARY_SERVICE)
	{
		printf("Read primary service\r\n");
	}
	if(state == READ_CHARACTERISTICS)
	{
		printf("Read characteristic\r\n");
	}
		printf("Connection Handle: 0x%04x\r\n", Connection_Handle);
		printf("Attribute data length: %03d\r\n",Attribute_Data_Length);
		printf("Data length: 0x%04x\r\n", Data_Length);
		printf("Attribute data list : %s\r\n", Attribute_Data_List);
		for (uint8_t i = 0; i < Data_Length; i++)
		{
			if(i%8 == 0 && i != 0)
		{
			printf("\r\n");
		}
		printf("0x%02x",Attribute_Data_List[i]);
		if(i != (Data_Length - 1))
		{
			printf(",");
		}
			
		}
		printf("\r\n");		
	printf("====Function end: aci_att_clt_read_by_group_type_resp_event()\r\n");
}
//==============================================================================					 
void aci_att_clt_read_by_type_resp_event(uint16_t Connection_Handle,
                                         uint8_t Handle_Value_Pair_Length,
                                         uint16_t Data_Length,
                                         uint8_t Handle_Value_Pair_Data[])
{
	printf("====Function begin: eci_att_clt_read_by_type_resp_event()\r\n");
	if(state == READ_PRIMARY_SERVICE)
	{
		printf("Read primary service\r\n");
	} else if (state == READ_CHARACTERISTICS)
	{
		printf("Read charactristic\r\n");		
	}
	
	printf("Connection handle is: 0x%04x\r\n", Connection_Handle);
	printf("Handle value pair length: %03d\r\n", Handle_Value_Pair_Length);
	printf("Data length: 0x%04x\r\n", Data_Length);
	printf("Handle Value Pair Data:\r\n");	
	for (uint8_t i = 0; i < Data_Length; i++)
		{
			if(i%8 == 0 && i != 0)
			{
				printf("\r\n");
			}
			printf("0x%02x", Handle_Value_Pair_Data[i]);
			if(i != (Data_Length - 1))
			{
				printf(",");
			}
			
		}
		printf("\r\n");	
	printf("====Function end: eci_att_clt_read_by_type_resp_event()\r\n");
	
}
//==============================================================================
void aci_gatt_clt_proc_complete_event(uint16_t Connection_Handle,
                                      uint8_t Error_Code)
{
	printf("====Function begin: aci_gatt_clt_proc_complete_event()\r\n");
	printf("Connection handle: 0x%04x\r\n", Connection_Handle);
	printf("Error code: 0x%02x\r\n", Error_Code);
	
	if(state == READ_PRIMARY_SERVICE)
	{
		if(Error_Code == 0x00)
		{
			printf("All Services has been read!!!");
		}
			state = READ_CHARACTERISTICS;
			firstEntryToState = 1;
	}else if(state == READ_CHARACTERISTICS)
	{
		if(Error_Code == 0x00)
		{
			printf("All Characteristics has been read!!!\r\n");
		}
			state = READ_DESCRIPTORS;
			firstEntryToState = 1;
	} else if(state == READ_DESCRIPTORS)
		{
			printf("All Descriptors has beend read!!! =)\r\n");
			state = READ_VALUES;
			firstEntryToState = 1;
		} else if(state == READ_VALUES)
		{
			printf("Value has been read\r\n");
			state = GET_NOTIFICATION;
			firstEntryToState = 1;
			//readDoneFlag = 1;
			
		} else if(state == GET_NOTIFICATION)
		{
			state = READ_MTU;
			firstEntryToState = 1;
		} else if(state == READ_MTU)
		{
			printf("MTU size has been read from server\r\n");
			state = READ_DATA;
			firstEntryToState = 1;
			
		}else
		{
			printf("REACTION NOT DEFENED!!!!!\r\n");
		}
		
	{
	}
	

	printf("====Function end: aci_gatt_clt_proc_complete_event()\r\n");
}
//==============================================================================
void aci_att_clt_find_info_resp_event(uint16_t Connection_Handle,
                                      uint8_t Format,
                                      uint16_t Event_Data_Length,
                                      uint8_t Handle_UUID_Pair[])
{
	printf("====Function begin: aci_att_clt_find_info_event()\r\n");
	printf("Read descriptor\r\n");
	printf("Connection handle: 0x%04x\r\n", Connection_Handle);
	printf("Format: 0x%02x\r\n", Format);
	printf("Event Data Lenght: %04d\r\n", Event_Data_Length);
	printf("Handle UUID Pair:\r\n");
	for (uint16_t i = 0; i < Event_Data_Length; i++)
		{
			if(i%8 == 0 && i != 0)
			{
				printf("\r\n");
			}
			printf("0x%02x", Handle_UUID_Pair[i]);
			if(i != (Event_Data_Length - 1))
			{
				printf(",");
			}		
		}
		printf("\r\n");	
	printf("====Function end: aci_att_clt_find_info_event()\r\n");
}
//==============================================================================
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
	printf("====Function begin: hci_disconnection_complete_event()\r\n");
	printf("Status: 0x%02x\r\n", Status);
	printf("Conneection Handle: 0x%04x\r\n", Connection_Handle);
	printf("Reason: 0x%02x\r\n", Reason);
	allTimeStatistic.allConnectionLost++;
	LED_GREEN_OFF();
	LED_RED_ON();
#ifdef AUTOCONNECT_MODE
	printf("Go to reconnect\r\n");
	state = RECONNECT;
	firstEntryToState = 1;
#endif	//AUTOCONNECT_MODE
	ConnDevListEntryDeliteByConnectionHandle(&connDevList, Connection_Handle);
	//clear entry in CONNECTION DEVICE TABLE
//	for(uint8_t i = 0; i < (connDevList.connDev[connDevList.nextElement - 1]); i++)
//	{
//		if(connDevList.connDev[i].)
//	}
		printf("====Function end: hci_disconnection_complete_event()\r\n");
}					 
//==============================================================================
void aci_att_clt_read_resp_event(uint16_t Connection_Handle,
                                 uint16_t Event_Data_Length,
                                 uint8_t Attribute_Value[])
{

	printf("====Functon begin: aci_att_clt_read_resp_event()\r\n");
	printf("Conneection Handle: 0x%04x\r\n", Connection_Handle);
	printf("Event Data Lenght: %04d\r\n", Event_Data_Length);
	printf("Atribute value:\r\n");
		for (uint16_t i = 0; i < Event_Data_Length; i++)
		{
			if(i%8 == 0 && i != 0)
			{
				printf("\r\n");
			}
			printf("0x%02x", Attribute_Value[i]);
			if(i != (Event_Data_Length - 1))
			{
				printf(",");
			}		
		}
		printf("\r\n");	
	printf("====Function end: aci_att_clt_read_resp_event()\r\n");

}

//==============================================================================
#ifndef USE_EVENTS_FROM_PACKET_HANDLER					 
void aci_gatt_clt_notification_event(uint16_t Connection_Handle,
                                     uint16_t Attribute_Handle,
                                     uint16_t Attribute_Value_Length,
                                     uint8_t Attribute_Value[])
{
//	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH);
	printf("====Functon begin: aci_gatt_clt_notification_event()\r\n");
	printf("Conneection Handle: 0x%04x\r\n", Connection_Handle);
	printf("Attribute Handle: 0x%04x\r\n", Attribute_Handle);
	printf("Attribute Value Lenght: 0x%04d\r\n", Attribute_Value_Length);
	printf("Atribute value:\r\n");
		for (uint16_t i = 0; i < Attribute_Value_Length; i++)
		{
			if(i%8 == 0 && i != 0)
			{
				printf("\r\n");
			}
			printf("0x%02x", Attribute_Value[i]);
			if(i != (Attribute_Value_Length - 1))
			{
				printf(",");
			}		
		}
	printf("\r\n");
	printf("====Functon end: aci_gatt_clt_notification_event()\r\n");
//	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW);
}	
#endif //		USE_EVENTS_FROM_PACKET HANDLER				 
//==============================================================================																			
void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Server_RX_MTU)
{
	printf("====Functon begin: aci_att_exchange_mtu_resp_event()\r\n");
	printf("Conneection Handle: 0x%04x\r\n", Connection_Handle);
	printf("Server Rx_MTU: %04d\r\n", Server_RX_MTU);
	printf("====Functon end: aci_att_exchange_mtu_resp_event()\r\n");
}
//==============================================================================
/*void aci_gatt_clt_indication_event(uint16_t Connection_Handle,
                                   uint16_t Attribute_Handle,
                                   uint16_t Attribute_Value_Length,
                                   uint8_t Attribute_Value[])
{
	uint8_t status;
	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH);
	ReadDataFromBufferToBuffer(Attribute_Value, testBuffer, Attribute_Value_Length);
	packetCounter++;
	printf("PC: %05d, DL: %05d\r\n",packetCounter, Attribute_Value_Length);
#ifdef DISABLE_PRINTF
	aci_gatt_clt_confirm_indication(connDevList.connDev[0].connectionHandle);
#endif	
#ifndef DISABLE_PRINTF
		status = aci_gatt_clt_confirm_indication(connDevList.connDev[0].connectionHandle);
	printf("====Functon begin: aci_gatt_clt_indication_event()\r\n");
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function aci_gatt_clt_confirm_indication()return error: 0x%02x\r\n"
					 , status);
	}else
	{
		printf("Conneection Handle: 0x%04x\r\n", Connection_Handle);
		printf("Attribute Handle: 0x%04x\r\n", Attribute_Handle);
		printf("Attribute Value Lenght: 0x%04d\r\n", Attribute_Value_Length);
		printf("Atribute value:\r\n");
		for (uint16_t i = 0; i < Attribute_Value_Length; i++)
		{
			if(i%8 == 0 && i != 0)
			{
				printf("\r\n");
			}
			printf("0x%02x", Attribute_Value[i]);
			if(i != (Attribute_Value_Length - 1))
			{
				printf(",");
			}		
		}
		printf("\r\n");
		printf("====Function aci_gatt_clt_confirm_indication(0x%04x) OK\r\n", status);
	}
	printf("====Functon end: aci_gatt_clt_indication_event()\r\n");
	#endif //DISABLE_PRINTF	
	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW);

}
*/
//==============================================================================
void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
	printf("====Functon begin: hci_le_data_length_change_event()\r\n");
	printf("Connection handle: 0x%04d\r\n", Connection_Handle);
	printf("MaxTxOctets: %05d\r\n", MaxTxOctets);
	printf("MaxTxTime: %05d\r\n", MaxTxTime);
	printf("MaxRxOctets: %05d\r\n", MaxRxOctets);
	printf("MaxRxTime: %05d\r\n", MaxRxTime);	
	printf("====Functon end: hci_le_data_length_change_event()\r\n");
}
//==============================================================================
void hci_le_read_remote_used_features_complete_event(uint8_t Status,
                                                     uint16_t Connection_Handle,
                                                     uint8_t LE_Features[8])
{
	printf("====Functon begin: hci_le_read_remote_used_features_complete_event()\r\n");
	
	
	printf("====Functon end: hci_le_read_remote_used_features_complete_event()\r\n");
	printf("Status code: 0x%02x\r\n",Status);
	printf("Connection Handle: 0x%04x\r\n", Connection_Handle);
	printf("LEFeature:\r\n");
	for(uint8_t i = 0; i < 8; i++)
	{
		printf("Byte %01d --- 0x%02x\r\n", i ,LE_Features[i]);
	}
	featureExchangedFlag = 1;
}
//==============================================================================					 
 /*        - 0x00: Idle
 *        - 0x01: Advertising
 *        - 0x02: Connection event slave
 *        - 0x03: Scanning
 *        - 0x04: Connection request
 *        - 0x05: Connection event master
 *        - 0x06: TX test mode
 *        - 0x07: RX test mode
*/
#ifdef ENABLE_RADIO_ACTIVITY_EVENT
void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)

{
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH);
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_LOW);
}
					 
#endif					 
