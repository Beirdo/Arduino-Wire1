/*
  twi1.h - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2016 by Gavin Hurlbut <gjhurlbu@gmail.com> to work on TWI1 on 
  ATMega168PB
  Copyright (c) 2016 Gavin Hurlbut.  All rights reserved.
*/

#ifndef twi1_h
#define twi1_h

  #include <inttypes.h>

  //#define ATMEGA8

  #ifndef TWI1_FREQ
  #define TWI1_FREQ 100000L
  #endif

  #ifndef TWI1_BUFFER_LENGTH
  #define TWI1_BUFFER_LENGTH 32
  #endif

  #define TWI1_READY 0
  #define TWI1_MRX   1
  #define TWI1_MTX   2
  #define TWI1_SRX   3
  #define TWI1_STX   4
  
  void twi1_init(void);
  void twi1_disable(void);
  void twi1_setAddress(uint8_t);
  void twi1_setFrequency(uint32_t);
  uint8_t twi1_readFrom(uint8_t, uint8_t*, uint8_t, uint8_t);
  uint8_t twi1_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t, uint8_t);
  uint8_t twi1_transmit(const uint8_t*, uint8_t);
  void twi1_attachSlaveRxEvent( void (*)(uint8_t*, int) );
  void twi1_attachSlaveTxEvent( void (*)(void) );
  void twi1_reply(uint8_t);
  void twi1_stop(void);
  void twi1_releaseBus(void);

#endif

