/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000



#define GPIO_A0			IOPORT_CREATE_PIN(PORTA, 0)
#define GPIO_A1			IOPORT_CREATE_PIN(PORTA, 1)
#define GPIO_A2			IOPORT_CREATE_PIN(PORTA, 2)
#define GPIO_A3			IOPORT_CREATE_PIN(PORTA, 3)
#define GPIO_A4			IOPORT_CREATE_PIN(PORTA, 4)
#define GPIO_A5			IOPORT_CREATE_PIN(PORTA, 5)
#define GPIO_A6			IOPORT_CREATE_PIN(PORTA, 6)
#define GPIO_A7			IOPORT_CREATE_PIN(PORTA, 7)

#define GPIO_B0			IOPORT_CREATE_PIN(PORTB, 0)
#define GPIO_B1			IOPORT_CREATE_PIN(PORTB, 1)
#define GPIO_B2			IOPORT_CREATE_PIN(PORTB, 2)
#define GPIO_B3			IOPORT_CREATE_PIN(PORTB, 3)
#define GPIO_B4			IOPORT_CREATE_PIN(PORTB, 4)
#define GPIO_B5			IOPORT_CREATE_PIN(PORTB, 5)
#define GPIO_B6			IOPORT_CREATE_PIN(PORTB, 6)
#define GPIO_B7			IOPORT_CREATE_PIN(PORTB, 7)

#define GPIO_C0			IOPORT_CREATE_PIN(PORTC, 0)
#define GPIO_C1			IOPORT_CREATE_PIN(PORTC, 1)
#define GPIO_C2			IOPORT_CREATE_PIN(PORTC, 2)
#define GPIO_C3			IOPORT_CREATE_PIN(PORTC, 3)
#define GPIO_C4			IOPORT_CREATE_PIN(PORTC, 4)
#define GPIO_C5			IOPORT_CREATE_PIN(PORTC, 5)
#define GPIO_C6			IOPORT_CREATE_PIN(PORTC, 6)
#define GPIO_C7			IOPORT_CREATE_PIN(PORTC, 7)

#define GPIO_D0			IOPORT_CREATE_PIN(PORTD, 0)
#define GPIO_D1			IOPORT_CREATE_PIN(PORTD, 1)
#define GPIO_D2			IOPORT_CREATE_PIN(PORTD, 2)
#define GPIO_D3			IOPORT_CREATE_PIN(PORTD, 3)
#define GPIO_D4			IOPORT_CREATE_PIN(PORTD, 4)
#define GPIO_D5			IOPORT_CREATE_PIN(PORTD, 5)
#define LED_D5			GPIO_D5
// #define GPIO_D6			IOPORT_CREATE_PIN(PORTD, 6) // USB D-
// #define GPIO_D7			IOPORT_CREATE_PIN(PORTD, 7) // USB D+

#define GPIO_E0			IOPORT_CREATE_PIN(PORTE, 0)
#define GPIO_E1			IOPORT_CREATE_PIN(PORTE, 1)
#define GPIO_E2			IOPORT_CREATE_PIN(PORTE, 2)
#define GPIO_E3			IOPORT_CREATE_PIN(PORTE, 3)
#define GPIO_E4			IOPORT_CREATE_PIN(PORTE, 4)
#define GPIO_E5			IOPORT_CREATE_PIN(PORTE, 5)

#define GPIO_F0			IOPORT_CREATE_PIN(PORTF, 0)
#define GPIO_F1			IOPORT_CREATE_PIN(PORTF, 1)
#define GPIO_F2			IOPORT_CREATE_PIN(PORTF, 2)
#define GPIO_F3			IOPORT_CREATE_PIN(PORTF, 3)
#define GPIO_F4			IOPORT_CREATE_PIN(PORTF, 4)
#define GPIO_F6			IOPORT_CREATE_PIN(PORTF, 6)
#define GPIO_F7			IOPORT_CREATE_PIN(PORTF, 7)

#define GPIO_R0			IOPORT_CREATE_PIN(PORTR, 0)
#define GPIO_R1			IOPORT_CREATE_PIN(PORTR, 1)

#endif // USER_BOARD_H
