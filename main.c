//******************************************************************************
//   MSP430FR599x Demo - eUSCI_B2, I2C Slave multiple byte TX/RX
//
//   Description: I2C master communicates to I2C slave sending and receiving
//   3 different messages of different length. (This is the slave code). The
//   slave will be in LPM0 mode, waiting for the master to initiate the
//   communication. The slave will send/receive bytes based on the master's
//   request. The slave will handle I2C bytes sent/received using the
//   I2C interrupt.
//   ACLK = NA, MCLK = SMCLK = DCO 16MHz.
//
//                                     /|\ /|\
//                   MSP430FR5994      4.7k |
//                 -----------------    |  4.7k
//            /|\ |             P7.1|---+---|-- I2C Clock (UCB2SCL)
//             |  |                 |       |
//             ---|RST          P7.0|-------+-- I2C Data (UCB2SDA)
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//
//   Nima Eskandari and Ryan Meredith
//   Texas Instruments Inc.
//   November 2017
//   Built with CCS V7.3
//******************************************************************************

#include <msp430.h> 
#include <stdint.h>

// Librarys for interfacing with some texas modules, like uart, spi ...
#include "src/uart.h"

#define FALSE 0
#define TRUE 1

//******************************************************************************
// Pin Config ******************************************************************
//******************************************************************************

#define LED_OUT     P1OUT
#define LED_DIR     P1DIR
#define LED0_PIN    BIT0
#define LED1_PIN    BIT1

//******************************************************************************
// XGZP6869 ************************************************************
//******************************************************************************
#define SLAVE_ADDR  0x50
#define XGZP6869_DATA_MSB_ADDR 0x06 /* Slave report pressure MSB */
#define XGZP6869_DATA_CSB_ADDR 0x07 /* Slave report pressure CSB */
#define XGZP6869_DATA_LSB_ADDR 0x08 /* Slave report pressure LSB */
#define XGZP6869_TEMP_MSB_ADDR 0x09 /* Slave report temperature MSB */
#define XGZP6869_TEMP_LSB_ADDR 0x0A /* Slave report temperature LSB */
#define XGZP6869_CMD_ADDR 0x30 /* Slave report data availability & Master request read */
#define XGZP6869_SYS_CONF_ADDR 0xA5 /* Slave report config & Master set config */
#define XGZP6969_PCONFIG_ADDR 0xA6

typedef enum XGZP6869_Buffer_pos{
    XGZP6869_DATA_MSB_POS = 0x0,
    XGZP6869_DATA_CSB_POS,
    XGZP6869_DATA_LSB_POS,
    XGZP6869_TEMP_MSB_POS,
    XGZP6869_TEMP_LSB_POS,
    XGZP6869_CMD_POS,
    XGZP6869_SYS_CONF_POS,
    XGZP6969_PCONFIG_POS,
    XGZP6969_MAX_POS
} XGZP6869_Buffer;

/* Master & Slaves buffers
 * Master buffer will retain master received data
 * Slave buffer will retain slave received data
 * */
//uint8_t SlaveBuffer[XGZP6969_MAX_POS] = {0x04,0x5d,0xe4,0x18,0xe4,0x2,0x11,0x20};
uint8_t SlaveBuffer[XGZP6969_MAX_POS] = {0x61,0xa8,0x00,0x18,0xe4,0x2,0x11,0x20};
uint8_t slave_pos = XGZP6869_DATA_MSB_POS;

//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    I2C_IDLE = 0x0,
    I2C_ADDR_RECEIVED,
    I2C_TX_MODE,
    I2C_RX_MODE
} I2C_Mode;

/* Used to track the state of the software state machine*/
I2C_Mode SlaveMode = I2C_IDLE;

/* The Register Address/Command to use*/
uint8_t ReceiveRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
#define MAX_BUFFER_SIZE 20
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initGPIO()
{
    //LEDs
    LED_DIR |= LED0_PIN | LED1_PIN;
    LED_OUT |= LED0_PIN | LED1_PIN;         // P1 setup for LED & reset output

    // SWs pins


    // I2C pins
    P7SEL0 |= BIT0 | BIT1;
    P7SEL1 &= ~(BIT0 | BIT1);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initI2C()
{
    UCB2CTLW0 = UCSWRST;                      // Software reset enabled
    UCB2CTLW0 |= UCMODE_3 | UCSYNC;           // I2C mode, sync mode
    UCB2I2COA0 = SLAVE_ADDR | UCOAEN;        // Own Address and enable
    UCB2CTLW0 &= ~UCSWRST;                    // clear reset register
    UCB2IE |= UCTXIE + UCRXIE + UCSTPIE + UCSTTIE;
}


void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz
    // Set SMCLK = MCLK = DCO, ACLK = LFXTCLK (VLOCLK if unavailable)
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;   // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz
    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                           // Lock CS registers
}

uint8_t getSlavePos(uint8_t addr) {
    uint8_t pos = 0;
    switch(addr)
    {
        case XGZP6869_DATA_MSB_ADDR:
            pos = XGZP6869_DATA_MSB_POS;
            break;
        case XGZP6869_DATA_CSB_ADDR:
            pos = XGZP6869_DATA_CSB_POS;
            break;
        case XGZP6869_DATA_LSB_ADDR:
            pos = XGZP6869_DATA_LSB_POS;
            break;
        case XGZP6869_TEMP_MSB_ADDR:
            pos = XGZP6869_TEMP_MSB_POS;
            break;
        case XGZP6869_TEMP_LSB_ADDR:
            pos = XGZP6869_TEMP_LSB_POS;
            break;
        case XGZP6869_CMD_ADDR:
            pos = XGZP6869_CMD_POS;
            break;
        case XGZP6869_SYS_CONF_ADDR:
            pos = XGZP6869_SYS_CONF_POS;
            break;
        case XGZP6969_PCONFIG_ADDR:
            pos = XGZP6969_PCONFIG_POS;
            break;
        default:
            break;
    }
    return pos;
}

void incSlavePos(uint8_t *ptr) {
    if (*ptr < (XGZP6969_MAX_POS - 1)) {
        *ptr = *ptr + 1;
    }
}

//******************************************************************************
// Main ************************************************************************
// Enters LPM0 and waits for I2C interrupts. The data sent from the master is  *
// then interpreted and the device will respond accordingly                    *
//******************************************************************************


void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    initClockTo16MHz();
    initGPIO();
    initI2C();
    UartInit(UART0);

    // test debug functionality
    DEBUG_PRINTF("I2C Slave ready! Waiting requests on slave ID(%x)", SLAVE_ADDR);

    // enable interrupts
    __bis_SR_register(GIE);

    for (;;)
    {
        DEBUG_FLUSH();
    }

}

//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B2_VECTOR))) USCI_B2_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Must read from UCB2RXBUF
  uint8_t rx_val = 0;
  switch(__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:
        break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      break;
    case USCI_I2C_UCSTTIFG:
        // start bit detected
        SlaveMode = I2C_IDLE;
        break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:
        // Stop bit detected? No more TX data required
        if (SlaveMode != I2C_ADDR_RECEIVED) {
            SlaveMode = I2C_IDLE;
        }
        UCB2IFG &= ~(UCTXIFG0);
        break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:
        break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:
        break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:
        break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:
        break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:
        break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:
        break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB2RXBUF;

        switch (SlaveMode) {
            case I2C_IDLE:
                SlaveMode = I2C_ADDR_RECEIVED;
                ReceiveRegAddr = rx_val;
                break;
            case I2C_ADDR_RECEIVED:
                SlaveMode = I2C_RX_MODE;
            case I2C_RX_MODE:
                // process received data
                // not implemented since we don't care!
                break;
            default:
                __no_operation();
                break;
        }
        break;

    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        switch (SlaveMode) {
            case I2C_IDLE:
            case I2C_ADDR_RECEIVED:
                slave_pos = getSlavePos(ReceiveRegAddr);
            case I2C_TX_MODE:
                SlaveMode = I2C_TX_MODE;
                UCB2TXBUF = SlaveBuffer[slave_pos];
                // increment for burst readings
                incSlavePos(&slave_pos);
                break;
            default:
                UCB2TXBUF = 0xff;
//                __no_operation();
                break;
        }
        break;                      // Interrupt Vector: I2C Mode: UCTXIFG
    default: break;
  }
}

/**
 * Interrupt handlers
 *  TIMER0_B0_VECTOR -> For Timer B0
 *  TIMER0_B1_VECTOR -> For Timer B1
 *  USCI_A0_VECTOR   -> For UART0 RX and TX
 *  PORT4_VECTOR     -> For Port 4
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void){
    uint8_t rcv_char = UCA0RXBUF;

    switch(rcv_char) {
        case '0':
            SlaveBuffer[0] = 0x00;
            SlaveBuffer[1] = 0x00;
            SlaveBuffer[2] = 0x00;
            break;
        case '1':
            SlaveBuffer[0] = 0x04;
            SlaveBuffer[1] = 0xE2;
            SlaveBuffer[2] = 0x00;
            break;
        case '2':
            SlaveBuffer[0] = 0x09;
            SlaveBuffer[1] = 0xC4;
            SlaveBuffer[2] = 0x00;
            break;
        case '3':
            SlaveBuffer[0] = 0x0E;
            SlaveBuffer[1] = 0xA6;
            SlaveBuffer[2] = 0x00;
            break;
        case '4':
            SlaveBuffer[0] = 0x13;
            SlaveBuffer[1] = 0x88;
            SlaveBuffer[2] = 0x00;
            break;
        case '5':
            SlaveBuffer[0] = 0x18;
            SlaveBuffer[1] = 0x6a;
            SlaveBuffer[2] = 0x00;
            break;
        case '6':
            SlaveBuffer[0] = 0x1d;
            SlaveBuffer[1] = 0x4c;
            SlaveBuffer[2] = 0x00;
            break;
        case '7':
            SlaveBuffer[0] = 0x22;
            SlaveBuffer[1] = 0x2e;
            SlaveBuffer[2] = 0x00;
            break;
        case '8':
            SlaveBuffer[0] = 0x27;
            SlaveBuffer[1] = 0x10;
            SlaveBuffer[2] = 0x00;
            break;
        case '9':
            SlaveBuffer[0] = 0x2b;
            SlaveBuffer[1] = 0xf2;
            SlaveBuffer[2] = 0x00;
            break;
        default:
            break;
    }
}

