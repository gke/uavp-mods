// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =      Rewritten and ported to 18F2520 2008 by Prof. Greg Egan        =
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

// Autonomous flight routines

#include "c-ufo.h"
#include "bits.h"

uint8 RCLinkRestored(int32 d)
{
	// checks for good RC frames for a period d mS
	if ( !_Signal )
		TimerMilliSec = ClockMilliSec + d;
	return(ClockMilliSec > TimerMilliSec );
} // RCLinkRestored

void CheckThrottleMoved(void)
{
	if ( _Signal )							// strictly redundant
	{
   		_ThrChanging = _NewValues 
			&& (IThrottle > Limit(PrevIThrottle - THR_WINDOW, _Minimum, _Maximum)) 
			&& (IThrottle < Limit(PrevIThrottle + THR_WINDOW, _Minimum, _Maximum) );
		PrevIThrottle = IThrottle;
	}
	else
		_ThrChanging = false;	
} // CheckThrottleMoved

uint8 Descend(uint8 T)
{
	if ( _UseBaro )
	{
		DesiredBaroAltitude = Limit(DesiredBaroAltitude - 1, 0, 32000);				// ??? needs more thought
	}
	else
	{	
	// need to use accelerometer based descent control
	//	if (((ClockMilliSec & 0x000000ff) == 0 ) && ( T > 0 ))
		if ( T > 0 )
			T -= 1;
	}
		return(T);
} // Descend

void Navigation(void)
{




} // Navigation

void DoAutonomous(void)
{

	DesiredThrottle = Descend(DesiredThrottle);
	IRoll = IPitch = IYaw = 0;

} // DoAutonomous
