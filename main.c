#include <LcdDriver/lower_driver.h>
#include "msp430fr6989.h"
#include "Grlib/grlib/grlib.h"
#include <stdint.h>
void HAL_LCD_PortInit(void) {
    /////////////////////////////////////
    // Configuring the SPI pins
    /////////////////////////////////////
    // Configure Serial Data Out (MOSI) on P1.6
    P1SEL0 |= BIT6; // P1.6 for UCB0SIMO (MOSI)
    P1SEL1 &= ~BIT6;
    // Configure Serial Clock (SCLK) on P1.4
    P1SEL0 |= BIT4; // P1.4 for UCB0CLK (SCLK)
    P1SEL1 &= ~BIT4;
    // Chip Select (CS) on P2.5
    P2DIR |= BIT5; // Set P2.5 as output
    P2OUT &= ~BIT5; // Set CS low to activate display
    // Data/Command (DC) on P2.3
    P2DIR |= BIT3; // Set P2.3 as output
    P2OUT &= ~BIT3; // Initialize DC to command mode (0)
    // Reset on P9.4
    P9DIR |= BIT4; // Set P9.4 as output
    P9OUT |= BIT4; // Initialize Reset high
    // Backlight on P2.6
    P2DIR |= BIT6; // Set P2.6 as output
    P2OUT |= BIT6; // Turn on Backlight
    // Apply a reset pulse for the display
    P9OUT &= ~BIT4; // Reset low
    __delay_cycles(16000); // Hold reset for a short period
    P9OUT |= BIT4; // Release reset
}
void HAL_LCD_SpiInit(void) {
    //////////////////////////
    // SPI configuration
    //////////////////////////
    // Put eUSCI_B0 in reset state
    UCB0CTLW0 = UCSWRST;
    // Configure eUSCI_B0 for SPI
    UCB0CTLW0 |= UCSYNC       // Synchronous mode
               | UCMST        // Master mode
               | UCCKPH       // Phase: capture on first clock edge
               | UCMSB        // MSB first
               | UCMODE_0     // 3-pin SPI mode
               | UCSSEL__SMCLK; // Use SMCLK as clock source
    // Set SPI clock frequency to 8 MHz (SMCLK is 16 MHz)
    UCB0BRW = 2; // Divide SMCLK by 2
    // Release eUSCI_B0 for operation
    UCB0CTLW0 &= ~UCSWRST;
}
//*****************************************************************************
// Writes a command to the CFAF128128B-0145T.  This function implements the basic SPI
// interface to the LCD display.
//*****************************************************************************
void HAL_LCD_writeCommand(uint8_t command)
{
    // Wait as long as the module is busy
    while (UCB0STATW & UCBUSY);
    // For command, set the DC' bit to low before transmission
    P2OUT &= ~BIT3;
    // Transmit data
    UCB0TXBUF = command;
    return;
}
//*****************************************************************************
// Writes a data to the CFAF128128B-0145T.  This function implements the basic SPI
// interface to the LCD display.
//*****************************************************************************
void HAL_LCD_writeData(uint8_t data)
{
    // Wait as long as the module is busy
    while (UCB0STATW & UCBUSY);
    // Set DC' bit back to high
    P2OUT |= BIT3;
    // Transmit data
    UCB0TXBUF = data;
    return;
}
