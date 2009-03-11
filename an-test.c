// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =           Extensively modified 2008-9 by Prof. Greg Egan            =
// =                          http://www.uavp.org                        =
// =======================================================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


// Utilities and subroutines

#include "pu-test.h"
#include "bits.h"


void AnalogTest(void)
{
	// UBatt
	val = ADC(0, ADCVREF5V);	
		
	val *= 46;
	val += 9;	// round up
	val /= 17;	// resolution is 0,01 Volt

	TxChar('V');
	TxChar('b');
	TxChar(':');
	
	TxValUL(NKS2 + LEN5);	// print millivolts from val
	TxText(SerVolt);

// URoll
#ifdef OPT_ADXRS
	val = ADC(1, ADCVREF5V);	
#endif
#ifdef OPT_IDG
	val = ADC(2, ADCVREF5V);	
#endif
	val *= 49;
	val += 5;	// round up
	val /= 10;	// resolution is 0,001 Volt

	TxChar('V');
	TxChar('r');
	TxChar(':');
	
	TxValUL(NKS3+LEN5);	// print millivolts from val
	TxText(SerVolt);

// UNick
#ifdef OPT_ADXRS
	val = ADC(2, ADCVREF5V);	
#endif
#ifdef OPT_IDG
	val = ADC(1, ADCVREF5V);	
#endif
	val *= 49;
	val += 5;	// round up
	val /= 10;	// resolution is 0,001 Volt

	TxChar('V');
	TxChar('p');
	TxChar(':');
	
	TxValUL(NKS3+LEN5);	// print millivolts from val
	TxText(SerVolt);

// UYaw
	val = ADC(4, ADCVREF5V);	
	val *= 49;
	val += 5;	// round up
	val /= 10;	// resolution is 0,001 Volt

	TxChar('V');
	TxChar('y');
	TxChar(':');
	
	TxValUL(NKS3+LEN5);	// print millivolts from val
	TxText(SerVolt);

// Uref
	val = ADC(3, ADCVREF5V);	
	val *= 49;
	val += 5;	// round up
	val /= 10;	// resolution is 0,001 Volt

	TxChar('V');
	TxChar('f');
	TxChar(':');
	
	TxValUL(NKS3+LEN5);	// print millivolts from val
	TxText(SerVolt);

}
