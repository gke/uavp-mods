=========================================================================

  UAVX - Experimental Quadrocopter Flight Software

  Based partially on UAVP (c) 2007 Ing. Wolfgang Mahringer

  Rewritten as UAVX (c) 2008 Prof. Greg Egan

=========================================================================

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

=========================================================================
  Reference Sites                           
=========================================================================

The original project may be found at:

	http://www.uavp.org 

The variants of UAVP to which this readme applies are hosted
at http:

	http://code.google.com/p/uavp-mods/ 

The related RCGroups thread is at:

	http://www.rcgroups.com/forums/showthread.php?t=811550

Disclaimer: The project at the Google Site is not endorsed by 
nor is it part of the official UAVP project.

=========================================================================
 The GNU GENERAL PUBLIC LICENSE             
=========================================================================

This software is released under GNU GPL license.
The GPL license is contained in the files
gpl-en.txt (in English) and gpl-de.txt (in German).

=========================================================================
 How to setup the Compilation Software      
=========================================================================

    * This version is for the 18F2620 PIC
    * Obtain MPLab and the C18 compiler from Microchip 

=========================================================================
 Transferring Firmware to the PIC           
=========================================================================

    * Use a PIC programmer to burn a version
      to the PIC - this will include the bootloader.
    * Once the bootloader is loaded once UAVPSet 
      may be used to load future versions of the
      flight and TestSoftware

=========================================================================
 VERSION HISTORY                            
=========================================================================

22.06.2010
- Draft version of a universal Rx decoder

=========================================================================
 SAFETY FIRST!                              
=========================================================================

If you work on your Quadrocopter, PLEASE TAKE CARE!

Nowadays, extremely powerful motors and accumulators
can pose a serious threat to your safety.

Safety measures:

- Remove the propellers before you test parameter or firmware modifications
- Work with a good bench power supply instead of an accupack.
  A short circuit will then not be able to destroy your hardware too easily
- Your ufo mainboards electronics can be damaged by 
  electrostatic discharge sparks. Make sure you are doing any
  work at the electronics in an ESD safe workplace.
- When using your transmitter, be sure to use the correct model memory on it.
- Be sure to have no other transmitters on your channel on air.

The authors will NOT BE RESPONSABLE FOR ANY ACCIDENTS OR
INJURIES IN ANY WAY!

PLEASE do yourself a favour and get an insurance which covers risks by doing 
model flight!

Imagine what happens if you only damage someone else's car or something....


=========================================================================
 Questions?                                
=========================================================================

BUT PLEASE, BEFORE YOU ASK, READ THE MANUAL. It is all in there, really! 
The manual may be found at the original project site.

Also, if you have some hardware related questions, please download the 
datasheets for the chips and read them thoroughly!

The variants of UAVP to which this readme applies are hosted
at:
	 http://code.google.com/p/uavp-mods/

There is an active RCGroups thread at:

	http://www.rcgroups.com/forums/showthread.php?t=811550

If you have any questions on the Official UAVP Project, please go to the forum at:
 
	http://forum.uavp.ch 

or the original project site at:

	http://www.uavp.org 

However, a registration is required to gain access to the forum.


Make sure you have the most recent version of the firmware!

