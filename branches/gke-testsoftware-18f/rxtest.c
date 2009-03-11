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


// Receiver signal test

#include "pu-test.h"
#include "bits.h"

// output all the signal values and the validity of signal
void ReceiverTest(void)
{
	uint8 s;
	uint16 *p;

	p=&CurrK1;
	for( s=1; s <= 7; s++ )
	{
		TxChar(s+'0');
		TxChar(':');
		val = *p;
		TxChar('0');
		TxChar('x');
		TxValH(val>>8);
		TxValH(val&0xff);
		if( ( val & 0xff00) != 0x0100 ) 
			TxText(SerFail);
		TxNextLine();
		p++;
	}
// show pause time
	TxChar('P');
	TxChar(':');
	val = 2*PauseTime;
 	val += (uint16)TMR2_5MS * 64;	// 78 * 16*16/4 us
	TxValUL(NKS3+LEN5);	// print from val
	TxText(SerMS);
}

void TogglePPMPolarity(void)
{
    Invert(ConfigReg,0);	// toggle bit
	if( NegativePPM )
		TxText(SerPPMN);
	else
		TxText(SerPPMP);

	CurrK1 = CurrK2 = CurrK3 = CurrK4 = CurrK5 = CurrK6 = CurrK7 = 0xFFFF;
	PauseTime = 0;
	_NewValues = 0;
	_NoSignal = 1;
}
