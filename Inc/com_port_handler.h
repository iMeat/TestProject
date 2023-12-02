#ifndef COM_PORT_HANDLER_H
#define COM_PORT_HANDLER_H
#include <stdint.h>

#define READ_BUFFER_SIZE		32


typedef struct
{
volatile uint8_t readBuffer[READ_BUFFER_SIZE];
volatile uint8_t *pointer;
volatile uint8_t readBufferFlag;
} rxBuffer_t;



void comPortRead(uint8_t * pRxDataBuffer, uint16_t DataSize);
void BufferInit();

#endif //COM_PORT_HANDLER_H
