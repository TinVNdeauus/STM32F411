# UART Communication in STM32CubeIDE

## Overview
This repository contains a project for UART (Universal Asynchronous Receiver/Transmitter) communication using STM32CubeIDE. The project is designed for STM32F411 microcontrollers.

## Features
- UART communication protocol implementation
- Data transmission and reception
- Integrating UART interrupt.

## Getting Started
### Init UART

    // init GPTO, PA2(UART2_TX), PA3(UART2_RX)
    // enable clock for GPIOA
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // set PA2 PA3 in alternate function mode
    
    uint32_t *MODER = (uint32_t *)(GPIOA_BASE_ADDR + 0x00);
    *MODER &= ~(0b1111 << 4);
    *MODER |= (0b10 << 4) | (0b10 << 6);
    
    // set PA2 PA3 in alternate function AF07
    
    uint32_t *AFRL = (uint32_t *)(GPIOA_BASE_ADDR + 0x20);
    *AFRL &= ~(0xff << 8);
    *AFRL |= (0b0111 << 8) | (0b0111 << 12);
    
    // init UART2
    // enable lock for UART2
    
    __HAL_RCC_USART2_CLK_ENABLE();
    
    // baudrate: 9600
    
    uint32_t *USART_BRR = (uint32_t *)(UART2_BASE_ADDR + 0x08);
    *USART_BRR = (104 << 4) | 3;
    
    // frame:
    //  + data size: 8 bite
    
    uint32_t *USART_CR1 = (uint32_t *)(UART2_BASE_ADDR + 0x0C);
    *USART_CR1 &= ~(1 << 12);
    
    //  + parity bit: none
    
    *USART_CR1 &= ~(1 << 10);
    
    // enable interrupt when RXNE is set
    
    *USART_CR1 |= (1<<5);
    
    // NVIC accept interrupt signal from UART2
    
    uint32_t* ISER1 = (uint32_t*)(0xe000e104);
    *ISER1 |= (1<<6);
    
    // enable TX, Rx
    
    *USART_CR1 |= (1 << 2) | (1 << 3);
    
    // enable UART
    
    *USART_CR1 |= (1 << 13);


### Send 1 byte data

    uint32_t* USART_SR=(uint32_t*)(UART2_BASE_ADDR+0x00);

    - Check DR is emty
    
    while((((*USART_SR)>>7)&1)!=1);
    uint32_t* USART_DR=(uint32_t*)(UART2_BASE_ADDR+0x04);
    *USART_DR=data;
    
    - Wait UART transfer data
    
    while((((*USART_SR)>>7)&1)!=1);

### Recieve data

    uint32_t* USART_SR=(uint32_t*)(UART2_BASE_ADDR+0x00);

    //check DR is emty
    
    while((((*USART_SR)>>7)&1)!=1);
    uint32_t* USART_DR=(uint32_t*)(UART2_BASE_ADDR+0x04);
    *USART_DR=data;
    
    //wait UART transfer data
    
    while((((*USART_SR)>>7)&1)!=1);
    
### UART interrupt
//declare global variable

char uart_rx_buff[128] = {0};
int uart_rx_index = 0;

//Create a function IQRHandler

uint32_t* USART_DR = (uint32_t*)(UART2_BASE_ADDR + 0x04);
uart_rx_buff[uart_rx_index] = *USART_DR;
uart_rx_index++;
if(strstr(uart_rx_buff, "LED ON") != 0) 
{
		led_crtl(LED_RED, 1);
		memset(uart_rx_buff, 0, sizeof(uart_rx_buff));
		uart_rx_index=0;

}
	else if(strstr(uart_rx_buff, "LED OFF") != 0) 
 {
		led_crtl(LED_RED, 0);
		memset(uart_rx_buff, 0, sizeof(uart_rx_buff));
		uart_rx_index=0;
  }

### Note:
With folder My_driver for control LED



