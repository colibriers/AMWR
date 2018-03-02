/* tests/unit-test.h.  Generated from unit-test.h.in by configure.  */
/*
 * Copyright © 2008-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _UNIT_TEST_H_
#define _UNIT_TEST_H_

/* Constants defined by configure.ac */
#define HAVE_INTTYPES_H 1
#define HAVE_STDINT_H 1

#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif
#ifdef HAVE_STDINT_H
# ifndef _MSC_VER
# include <stdint.h>
# else
# include "stdint.h"
# endif
#endif

#define SERVER_ID         1 //for multi aiv should diff for distinguish
#define INVALID_SERVER_ID 18

const uint16_t UT_BITS_ADDRESS = 0x01 ; //define the wr bit operation mem
const uint16_t UT_BITS_NB = 0x03;
const uint8_t UT_BITS_TAB[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint16_t UT_INPUT_BITS_ADDRESS = 0x2711;
const uint16_t UT_INPUT_BITS_NB = 0x03;
const uint8_t UT_INPUT_BITS_TAB[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint16_t UT_INPUT_REGISTERS_ADDRESS = 0x7531;
const uint16_t UT_INPUT_REGISTERS_NB = 0x7F;
const uint16_t UT_INPUT_REGISTERS_TAB[] = {0xFFF1, 0xFFF2,0xFFF3, 0xFFF4};

const uint16_t UT_REGISTERS_ADDRESS = 0x9C41;
const uint16_t UT_REGISTERS_NB = 0xFF;
const uint16_t UT_REGISTERS_TAB[] = { 0x0000, 0x0001, 0x0002,0x0003};

const uint16_t UT_REGISTERS_ADDRESS_SPECIAL = 0xAFC8;  
const uint16_t UT_REGISTERS_NB_SPECIAL = 0x20;

const float UT_REAL = 916.540649;
const uint32_t UT_IREAL = 0x4465229a;

#endif /* _UNIT_TEST_H_ */
