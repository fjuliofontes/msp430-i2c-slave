/** File: uart.h
 *  Brief: uart interface header
 *  Author : Fernando Fontes
 *  Contact : 
 */
#ifndef __UART_H__
#define __UART_H__

// Define return codes
#define UART_SUCCESS 0
#define UART_BR_NOT_SUP 1
#define UART_PBCLOCK_NOT_SUP 2

#define UART0               0
#define UART0_RX_PIN        BIT0
#define UART0_TX_PIN        BIT1
#define UART0_INIT_CONF     (P2SEL1 |= (UART0_RX_PIN | UART0_TX_PIN)); (P2SEL0 &= ~(UART0_RX_PIN | UART0_TX_PIN))

#define UART1               1
#define UART1_RX_PIN        BIT5
#define UART1_TX_PIN        BIT6
#define UART1_INIT_CONF     (P2SEL0 &= ~(UART1_RX_PIN | UART1_TX_PIN)); (P2SEL1 |= (UART1_RX_PIN | UART1_TX_PIN))

#define UART2               2
#define UART2_RX_PIN        BIT5
#define UART2_TX_PIN        BIT4
#define UART2_INIT_CONF     (P5SEL0 |= (UART2_RX_PIN | UART2_TX_PIN)); (P5SEL1 &= ~(UART2_RX_PIN | UART2_TX_PIN))

#define UART3               3
#define UART3_RX_PIN        BIT1
#define UART3_TX_PIN        BIT0
#define UART3_INIT_CONF     (P6SEL0 |= (UART3_RX_PIN | UART3_TX_PIN)); (P6SEL1 &= ~(UART3_RX_PIN | UART3_TX_PIN))


#define UART0_STATE         UCA0STATW        //BUSY
#define UART0TX_BUFF        UCA0TXBUF        //BUFFER
#define UART0RX_BUFF        UCA0RXBUF        //rx BUFFER

#define UART1_STATE         UCA1STATW        //BUSY
#define UART1TX_BUFF        UCA1TXBUF        //BUFFER
#define UART1RX_BUFF        UCA1RXBUF        //rx BUFFER

#define UART3_STATE         UCA3STATW        //BUSY
#define UART3TX_BUFF        UCA3TXBUF        //BUFFER
#define UART3RX_BUFF        UCA3RXBUF        //rx BUFFER

#define UART2_STATE         UCA2STATW        //BUSY
#define UART2TX_BUFF        UCA2TXBUF        //BUFFER
#define UART2RX_BUFF        UCA2RXBUF        //rx BUFFER

#include <stdint.h>

/**
 * _putc
 * Input : char s
 * Brief description :
 *  Puts the char in the serial port
 */
void _putc(char s);
void _putc_uart1(char s);
void _putc_uart2(char s);
void _putc_uart3(char s);

/**
 * _puts
 * Input : char * s
 * Brief description :
 *  Puts the char in the serial port
 */
void _puts(char *s);
void _puts_uart1(char *s);
void _puts_uart2(char *s);
void _puts_uart3(char *s);

/**
 * _printf
 * Input : Can be used as the normal printf
 * Brief description :
 *  Prints the following formats : %c %f %d %u %x %o %s (MAX OF INT16 TYPE)
 *  USE ONLY WITH INT16 NUMBERS
 */
void _printf(char *,...);

void DEBUG_PRINTF(char* format,...);

void DEBUG_FLUSH(void);

/**
 * UartInit
 * Input : Baudrate
 * Brief description :
 *  Inits the uart with the 115200 baudrate
 */
uint8_t UartInit(uint8_t _uart);

/**
 * _getc
 * Input : Destination byte
 * Brief description :
 *  Get a byte from serial port
 */
uint8_t _getc(uint8_t *byte);
uint8_t _getc_uart1 (uint8_t *byte);
uint8_t _getc_uart2 (uint8_t *byte);
uint8_t _getc_uart3 (uint8_t *byte);

/**
 * convert
 * Input : num
 * Brief description :
 *  convert num to string
 */
char *myITOA(uint32_t num, uint8_t base, uint8_t decimal_points);

/**
 * convert
 * Input : num
 * Brief description :
 *  convert double num to string
 */

char *myFTOA(float float_part, uint8_t decimal_points , uint8_t base);

#endif
