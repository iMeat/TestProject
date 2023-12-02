#ifndef EVENTS_H
#define EVENTS_H


//includes
//
#include <stdint.h>
#include "ble_const.h"
#include <stdio.h>
#include "console.h"

//#define ENABLE_RADIO_ACTIVITY_EVENT



void hci_le_extended_advertising_report_event(uint8_t num_reports,
																							Extended_Advertising_Report_t extended_advertising_report[]);

void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy);


void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 uint8_t Data[]);



#endif //EVENTS_H