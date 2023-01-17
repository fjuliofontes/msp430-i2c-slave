/** File: uart.c
 *  Brief: uart interface functions
 *  Author : Fernando Fontes
 *  Contact : 
 */
#include <msp430.h>
#include <math.h>
#include "src/uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

//******************************************************************************
// UART DEGUG ******************************************************************
//******************************************************************************
#define ENABLE_DEBUG
#define BUFFER_LENGHT 100
#define BUFFER_POSITIONS 20

// variables
static char _buffer [BUFFER_POSITIONS][BUFFER_LENGHT]; // 20 positions
static int _head = 0;
static int _cursor = 0;

void initUartPins(uint8_t _uart){
    switch(_uart){
    case 0:
        UART0_INIT_CONF;
        break;
    case 1:
        UART1_INIT_CONF;
        break;
    case 2:
        UART2_INIT_CONF;
        break;
    case 3:
        UART3_INIT_CONF;
        break;
    default:
        break;
    }
}

uint8_t UartInit(uint8_t _uart){
    // init uart pins
    initUartPins(_uart);

    switch(_uart){
    case 0:
        // config with 115200 baudrate
        UCA0CTLW0 = UCSWRST;  // reset hold
        UCA0CTLW0 |= UCSSEL__SMCLK; // CLK = SMCLK
        UCA0CTLW0 &= ~(UC7BIT); // 8 bit of data
        // Baud _Rate calculation
        // 16000000/(16*115200) = 8.680555556
        // Fractional portion = 0.680555556
        // User's Guide Table 21-4: UCBRSx = 0xF7
        // UCBRFx = int ( (8.680555556-8)*16) = 10
        UCA0BRW = 8;
        UCA0MCTLW |= UCOS16 | UCBRF_10 | 0xF700;
        UCA0CTLW0 &= ~UCSWRST; // Initialize eUSCI
        UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupt
        break;
    case 1:
        // 19200 baud
        UCA1CTLW0 = UCSWRST;
        UCA1CTLW0 |= UCSSEL__SMCLK; // CLK = SMCLK
        UCA1BRW = 26;
        UCA1MCTLW |= UCOS16 | UCBRF_0 | 0xB600;
        UCA1CTLW0 &= ~UCSWRST; // Initialize eUSCI
        break;
    case 2:
        // 9600 baud
        UCA2CTLW0 = UCSWRST;
        UCA2CTLW0 |= UCSSEL__SMCLK; // CLK = SMCLK
        UCA2BRW = 52;
        UCA2MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
        UCA2CTLW0 &= ~UCSWRST; // Initialize eUSCI
        break;
    case 3:
        // 9600 baud
        UCA3CTLW0 = UCSWRST;
        UCA3CTLW0 |= UCSSEL__SMCLK; // CLK = SMCLK
        UCA3BRW = 52;
        UCA3MCTLW |=  UCOS16 | UCBRF_1 | 0x4900;
        UCA3CTLW0 &= ~UCSWRST; // Initialize eUSCI
        break;
    default:
        break;
    }

    return UART_SUCCESS;
}

uint8_t _getc (uint8_t *byte)
{
    if(UCA0IFG & UCRXIFG)
    {
        *byte = UCA0RXBUF;              // get data from UART RX FIFO
        return 1;
    }
    return 0;
}

void _putc(char s){
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = s;              //start of frame
}

void _puts(char *s){
  while(*s) _putc(*s++);
}

char *myITOA(uint32_t num, uint8_t base, uint8_t decimal_points)
{
    static char Representation[]= "0123456789ABCDEF";
    static char buffer[50];
    char *ptr;
    uint8_t i = 0;

    ptr = &buffer[49];
    *ptr = '\0';

    do
    {
        i++;
        *--ptr = Representation[num%base];
        num /= base;
    }while(num != 0);

    // padding with left zeros if needed
    while (i < decimal_points){
        i++;
        *--ptr = '0';
    }

    return(ptr);
}

char *myFTOA(float float_part, uint8_t decimal_points , uint8_t base){
    static char buffer[50];
    char *tmp, point = '.';
    uint32_t integer_part = (uint32_t)(float_part);
    uint32_t decimal_part = (uint32_t)((float_part-integer_part)*pow(10,decimal_points));

    memset(buffer,'\0',sizeof(buffer)); // clean string
    tmp = myITOA(integer_part,base, 0);     //  convert messages counter to decimal
    memcpy(buffer, tmp, strlen(tmp));
    strncat(buffer, &point, 1);
    tmp = myITOA(decimal_part,base, decimal_points);     //  convert messages counter to decimal
    strncat(buffer, tmp, strlen(tmp));

    return &buffer[0];
}

void _printf(char* format,...){
    int16_t int_numb;
    uint32_t i, format_len;
    float float_numb;
    char *str;

    // Initializing _printf arguments
    va_list arg;
    va_start(arg, format);
    format_len = strlen(format);

    for(i=0; i < format_len; i++){
        // if current char is not an input spec print it
        // stop at the end of the format string
        while((format[i] != '%') && (i < format_len)){
            _putc(format[i]);
            i++;
        }

        // if is end of string break
        if(i >= format_len){
            break;
        }

        //Fetching and executing arguments
        switch(format[++i]){
            case 'c' : int_numb = va_arg(arg,int);         //Fetch char argument
                        _putc(int_numb);
                        break;
            case 'i' :
            case 'd' : int_numb = va_arg(arg,int);         //Fetch Decimal/Integer argument
                        if(int_numb<0){ int_numb *= -1; _putc('-'); }
                        _puts(myITOA(int_numb,10,0));
                        break;

            case 'u' : int_numb = va_arg(arg,int);         //Fetch unsigned Decimal/Integer argument
                        _puts(myITOA(int_numb,10,0));
                        break;

            case 'o': int_numb = va_arg(arg,int); //Fetch Octal representation
                        _puts(myITOA(int_numb,8,0));
                        break;

            case 's': str = va_arg(arg,char *);       //Fetch string
                        _puts(str);
                        break;

            case 'x': int_numb = va_arg(arg,int); //Fetch Hexadecimal representation
                        _puts(myITOA(int_numb,16,0));
                        break;

            case 'f': float_numb = va_arg(arg,double);         //Fetch float representation
                        if(float_numb<0){    float_numb *= -1;    _putc('-'); }
                        _puts(myFTOA(float_numb,3,10));
                        break;
        }
    }

    //Module 3: Closing argument list to necessary clean-up
    va_end(arg);
}

#ifdef ENABLE_DEBUG
void DEBUG_PRINTF(char* format,...) {
    static uint16_t msg_counter = 0;

    // loop variables
    int16_t int_numb;
    uint32_t i, format_len, pos = 0;
    float float_numb;
    char *str, *destination = _buffer[_head];

    // add msg counter [value]
    str = myITOA(msg_counter++,10,0);
    destination[pos++] = '[';
    memcpy(&destination[pos],str,strlen(str));
    pos += strlen(str);
    destination[pos++] = ']';
    destination[pos++] = ' ';

    // Initializing _printf arguments
    va_list arg;
    va_start(arg, format);
    format_len = strlen(format);

    for(i=0; i < format_len; i++){
        // if current char is not an input spec print it
        // stop at the end of the format string
        while((format[i] != '%') && (i < format_len)){
            destination[pos++] = format[i];
            i++;
        }

        // if is end of string break
        if(i >= format_len){
            break;
        }

        //Fetching and executing arguments
        switch(format[++i]){
            case 'c' : int_numb = va_arg(arg,int);         //Fetch char argument
                        destination[pos++] = (char)int_numb;
                        break;
            case 'u' :
            case 'i' :
            case 'd' : int_numb = va_arg(arg,int);         //Fetch Decimal/Integer argument
                        if(int_numb<0){ int_numb *= -1; destination[pos++] = '-'; }
                        str = myITOA(int_numb,10,0);
                        memcpy(&destination[pos],str,strlen(str));
                        pos += strlen(str);
                        break;

            case 's': str = va_arg(arg,char *);       //Fetch string
                        memcpy(&destination[pos],str,strlen(str));
                        pos += strlen(str);
                        break;

            case 'x': int_numb = va_arg(arg,int); //Fetch Hexadecimal representation
                        str = myITOA(int_numb,16,0);
                        memcpy(&destination[pos],str,strlen(str));
                        pos += strlen(str);
                        break;

            case 'f': float_numb = va_arg(arg,double);         //Fetch float representation
                        if(float_numb<0){    float_numb *= -1;    destination[pos++] = '-'; }
                        str = myFTOA(float_numb,3,10);
                        memcpy(&destination[pos],str,strlen(str));
                        pos += strlen(str);
                        break;
        }
    }

    //Module 3: Closing argument list to necessary clean-up
    va_end(arg);

    // add string terminator
    destination[pos] = '\0';

    // increment buffer head
    _head = (_head + 1) % BUFFER_POSITIONS;
}
#else
void DEBUG_PRINTF(char* format,...) {
    return;
}
#endif

void DEBUG_FLUSH(void) {
    while (_head != _cursor)
    {
        _printf("%s\r\n",_buffer[_cursor]);
        _cursor = (_cursor + 1) % BUFFER_POSITIONS;

        // burst flush delay
        if (_head != _cursor)
        {
            __delay_cycles(10);
        }
    }
}

/* UART_1*/
uint8_t _getc_uart1 (uint8_t *byte)
{
    if(UCA1IFG & UCRXIFG)
    {
        *byte = UCA1RXBUF;              // get data from UART RX FIFO
        return 1;
    }
    return 0;
}

void _putc_uart1(char s){
    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = s;              //start of frame
}

void _puts_uart1(char *s){
  while(*s) _putc_uart1(*s++);
}

/* UART_2*/
uint8_t _getc_uart2 (uint8_t *byte)
{
    if(UCA2IFG & UCRXIFG)
    {
        *byte = UCA2RXBUF;              // get data from UART RX FIFO
        return 1;
    }
    return 0;
}

void _putc_uart2(char s){
    while(!(UCA2IFG & UCTXIFG));
    UCA2TXBUF = s;              //start of frame
}

void _puts_uart2(char *s){
  while(*s) _putc_uart2(*s++);
}

/* UART_3*/
uint8_t _getc_uart3 (uint8_t *byte)
{
    if(UCA3IFG & UCRXIFG)
    {
        *byte = UCA3RXBUF;              // get data from UART RX FIFO
        return 1;
    }
    return 0;
}

void _putc_uart3(char s){
    while(!(UCA3IFG & UCTXIFG));
    UCA3TXBUF = s;              //start of frame
}

void _puts_uart3(char *s){
  while(*s) _putc_uart3(*s++);
}
