// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

// Analog Gyros

void GetAnalogGyroValues(void)
{
	// change of sign to get to normal aircraft sense 
	A[Roll].GyroADC = -ADC(ADCRollChan);
	A[Pitch].GyroADC = -ADC(ADCPitchChan);
	A[Yaw].GyroADC = ADC(ADCYawChan);

} // GetAnalogGyroValues

void InitAnalogGyros(void)
{
	// nothing to be done for analog gyros - could check nominal midpoints?
	F.GyroFailure = false;
} // InitGyros










