#ifndef __USART_H
#define __USART_H

void USART_Init_Config(USART_TypeDef *USARTx, uint32_t baudrate, uint16_t stop_bits, uint16_t parity, FunctionalState remap);

void Serial1_SendByte(uint8_t Byte);
void Serial1_SendArray(uint8_t *Array, uint16_t Length);
void Serial1_SendString(char *String);

void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array, uint16_t Length);
void Serial2_SendString(char *String);

void Serial3_SendByte(uint8_t Byte);
void Serial3_SendArray(uint8_t *Array, uint16_t Length);
void Serial3_SendString(char *String);
#endif
