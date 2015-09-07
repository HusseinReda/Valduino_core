/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

/*
 * To define the P and PM include iodefine.h
 */
#ifndef  __INTRINSIC_FUNCTIONS
#include "iodefine.h"
#endif

/*
 *  This macro takes the bit number and returns the bit mask
 */
#define _BV(bit) (1<<bit)	//FIXME: Move this macro to a suitable library

#ifndef VALDUINO
#define VALDUINO
#endif
//#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)

#if defined(__AVR_ATmega8__)
#define digitalPinHasPWM(p)         ((p) == 9 || (p) == 10 || (p) == 11)
#else
#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)
#endif

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t SDA = 18;
static const uint8_t SCL = 19;
#define LED_BUILTIN 13

static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;
static const uint8_t A6 = 20;
static const uint8_t A7 = 21;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))
//TODO: AMMAR: definethis section as ARDUINO_MAIN

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

// ATMEL ATMEGA1280 / ARDUINO
//
// 0-7 PE0-PE7   works
// 8-13 PB0-PB5  works
// 14-21 PA0-PA7 works 
// 22-29 PH0-PH7 works
// 30-35 PG5-PG0 works
// 36-43 PC7-PC0 works
// 44-51 PJ7-PJ0 works
// 52-59 PL7-PL0 works
// 60-67 PD7-PD0 works
// A0-A7 PF0-PF7
// A8-A15 PK0-PK7


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, /* 0 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, /* 14 */
	PC,
	PC,
	PC,
	PC,
	PC,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(0), /* 14, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, /* 0 - port D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	// on the ATmega168, digital pin 3 has hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
#else
	TIMER2B,
#endif
	NOT_ON_TIMER,
	// on the ATmega168, digital pins 5 and 6 have hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
	NOT_ON_TIMER,
#else
	TIMER0B,
	TIMER0A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 8 - port B */
	TIMER1A,
	TIMER1B,
#if defined(__AVR_ATmega8__)
	TIMER2,
#else
	TIMER2A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 14 - port C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};

#endif

//HUSSEIN: ADD VALDUINO DEFINE HERE PLZ
#ifdef VALDUINO

//FIXME: add array to program memory
const uint8_t digital_pin_to_port_PGM[] = {
	NOT_A_PORT,
	_P15,
	_P15,
	_P15,
	_P15,
	_P4,
	_P4,
	_P4,
	_P4,
	_P4,
	_P4,
	_P4,
	_P4,
	NOT_A_PORT,
	_P12,
	_P12,
	_P13,
	_P12,
	_P12,
	NOT_A_PORT,
	NOT_A_PORT,
	NOT_A_PORT,
	NOT_A_PORT,
	NOT_A_PORT,
	_P6,
	_P6,
	_P6,
	_P6,
	_P6,
	_P6,
	_P6,
	_P6,
	_P15,
	_P15,
	_P0,
	_P15,
	_P15,
	_P14,
	_P13,
	_P7,
	_P7,
	_P7,
	_P7,
	NOT_A_PORT,
	_P7,
	_P7,
	_P7,
	_P7,
	_P0,
	_P3,
	_P3,
	_P1,
	_P1,
	NOT_A_PORT,
	_P1,
	_P3,
	_P5,
	_P5,
	_P5,
	_P5,
	_P1,
	_P1,
	_P1,
	_P1,
	_P1,
	_P5,
	_P5,
	_P5,
	_P10,
	_P10,
	_P10,
	_P10,
	_P3,
	_P3,
	_P5,
	_P8,
	_P8,
	_P8,
	_P8,
	_P8,
	_P8,
	_P8,
	_P8,
	_P9,
	_P9,
	_P9,
	_P9,
	_P9,
	_P9,
	_P9,
	_P9,
	_P10,
	_P10,
	_P10,
	_P10,
	_P0,
	_P12,
	_P12,
	_P0,
	_P12,
	_P12,
};

//FIXME: add array to program memory
const uint8_t *  port_to_output_PGM[] = {
	&P0,
	&P1,
	NOT_A_PORT,
	&P3,
	&P4,
	&P5,
	&P6,
	&P7,
	&P8,
	&P9,
	&P10,
	NOT_A_PORT,
	&P12,
	&P13,
	&P14,
	&P15,
};

//FIXME: add array to program memory
//FIXME: port 13 doesn't have a macro PM13
const uint8_t * port_to_mode_PGM[] = {
	&PM0,
	&PM1,
	NOT_A_PORT,
	&PM3,
	&PM4,
	&PM5,
	&PM6,
	&PM7,
	&PM8,
	&PM9,
	&PM10,
	NOT_A_PORT,
	&PM12,
	NOT_A_PORT,
	&PM14,
	&PM15,
};

const uint8_t digital_pin_to_bit_mask_PGM[] = {
	NOT_A_PIN,
	_BV(3),	/*P15*/
	_BV(2),
	_BV(1),
	_BV(0),
	_BV(7),	/*P4*/
	_BV(6),
	_BV(5),
	_BV(4),
	_BV(3),
	_BV(2),
	_BV(1),
	_BV(0),
	NOT_A_PIN,
	_BV(4),	/*P12*/
	_BV(3),
	_BV(7),	/*P13*/
	_BV(2),
	_BV(1),
	NOT_A_PIN,
	NOT_A_PIN,
	NOT_A_PIN,
	NOT_A_PIN,
	NOT_A_PIN,
	_BV(0),	/*P6*/
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(4),	/*P15*/
	_BV(5),
	_BV(0),	/*P0*/
	_BV(6),	/*P15*/
	_BV(7),
	_BV(0),	/*P14*/
	_BV(0),	/*P13*/
	_BV(7),	/*P7*/
	_BV(6),
	_BV(5),
	_BV(4),
	NOT_A_PIN,
	_BV(3),	/*P7*/
	_BV(2),
	_BV(1),
	_BV(0),
	_BV(3),	/*P0*/
	_BV(2),	/*P3*/
	_BV(0),
	_BV(7),	/*P1*/
	_BV(6),
	NOT_A_PIN,
	_BV(5),	/*P1*/
	_BV(1),	/*P3*/
	_BV(0),	/*P5*/
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),	/*P1*/
	_BV(3),
	_BV(2),
	_BV(1),
	_BV(0),
	_BV(4),	/*P5*/
	_BV(5),
	_BV(6),
	_BV(7),	/*P10*/
	_BV(6),
	_BV(5),
	_BV(4),
	_BV(3),	/*P3*/
	_BV(4),
	_BV(7),	/*P5*/
	_BV(0),	/*P8*/
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0),	/*P9*/
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0),	/*P10*/
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(2),	/*P0*/
	_BV(7),	/*P12*/
	_BV(6),
	_BV(1),	/*P0*/
	_BV(5),	/*P12*/
	_BV(0),
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
