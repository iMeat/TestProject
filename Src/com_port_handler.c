#include "com_port_handler.h"
#include "rf_driver_ll_usart.h"
#include "steval_idb011V1_config.h"

rxBuffer_t comRxBuffer;
uint8_t rxne = 0;

void comPortRead(uint8_t * pRxDataBuffer, uint16_t DataSize)
{
	//make ring buffer
	if(comRxBuffer.pointer == &comRxBuffer.readBuffer[31])
	{
		comRxBuffer.pointer = &(comRxBuffer.readBuffer[0]);
	}
	//read value from rx fifo
	*(comRxBuffer.pointer) = *pRxDataBuffer;
//*pointer) = *pRxDataBuffer;
	rxne = (LL_USART_IsActiveFlag_RXNE(BSP_UART));
	if((*(comRxBuffer.pointer) == '\n' || *(comRxBuffer.pointer) == '\r') && ( rxne != 1)) 
		//isrflags & USART_ISR_RXNE_RXFNE) != 0U)
	{	
		comRxBuffer.readBufferFlag = 1;
		
		comRxBuffer.pointer = &(comRxBuffer.readBuffer[0]);
		return;
	}
	comRxBuffer.pointer++;
}
//==============================================================================
//(uint8_t * pRxDataBuff, uint16_t nDataSize);

uint8_t comPortHandler (uint8_t Data)
{
return Data;
	
}
//==============================================================================
void BufferInit()
{
comRxBuffer.pointer = &(comRxBuffer.readBuffer[31]);
comRxBuffer.readBufferFlag = 0;
}
//==============================================================================