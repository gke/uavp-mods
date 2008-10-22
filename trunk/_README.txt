==============================================
=   U    U      AA     V     V   PPPPP       =
=   U    U     A  A    V     V   P    P      =
=   U    U    A    A   V     V   P    P      =
=   U    U    AAAAAA    V   V    PPPPP       =
=   U    U    A    A     V V     P           =
=    UUUU  U  A    A A    V  V   P    P      =
=                                            =
=  Universal  Airial   Video     Platform    =
=                                            =
=      U.A.V.P Brushless UFO Controller      =
=           Professional Version             =
= Copyright (c) 2007 Ing. Wolfgang Mahringer =
==============================================

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

==============================================
=  please visit http://www.uavp.org          =
=               http://www.mahringer.co.at   =
==============================================

==============================================
= Contact the author:                        =
==============================================
email: wolfgang@opensourcequadrocopter.de
snail: Ing. Wolfgang Mahringer
       Wiesersberg 54
       5760 Saalfelden
       AUSTRIA/EUROPE

==============================================
= The GNU GENERAL PUBLUC LICENSE             =
==============================================

This software is released under GNU GPL license.
The GPL license is contained in the files
gpl-en.txt (in English) and gpl-de.txt (in German).


==============================================
= How to setup the project                   =
==============================================

In order to work with this firmware you need:
- MPLAB IDE, at least version 7.40
  free download at http://www.microchip.com
- CC5X Compiler, at least version 3.3a
  free download at http://wwww.bknd.com
- The free UAVPset software (for Windows)

Follow the instructions found in the CC5X package on how to integrate 
this language toolset into MPLAB

Create a project, which creates a project directory for the source files.
Put the contents of this ZIP file into that directory.

Add all .c files and bootloader.asm to the "Source Files" tag
Add all .h-Files to the "Header Files" tag
Add p16f876i.lkr-Files to the "Linker Scripts" tag

Setup your Build options:
Set "Include-Path" and "Library Path" to the directory where you installed CC5X
Set "Linker-Script Path" to your project directory.
Set your Assembler options to
    /e+ /l+ /x- /c+ /q
    Additionally, you must define your PCB board version: BOARD_3_0 or BOARD_3_1
Set your CC5X-C-Compiler options to 
    "-CC -fINHX8M -a -L -Q -V -FM -DMATHBANK_VARS=0 -DMATHBANK_PROG=2"
    The selected output format is not important here.
    Additionally, you must define your PCB board version: BOARD_3_0 or BOARD_3_1
Set "MPLINK" options to
    "/m profi-ufo.map /aINHX8M"
Select the output format that suits your PIC programmer
Add an option "-ro1" to the build options for "sensor.c" only

BOARD_3_0 selects the green PCB (older version)
BOARD_3_1 selects the black PCB (current version)

Caution: There is a bug in MPLAB (still in 7.60) which does not correctly promote
the BOARD_3_x symbol to all the nodes. 
Please check the files own build options if BOARD_3_x is set correctly.

==============================================
= How to build the firmware                  =
==============================================

Open "c-ufo.h" and setup all #defines found
in "Global compiler switches" to suit your hardware.

Press Ctrl+F10 to compile and link the project.
If all went fine, a HEX files has been created.
Use your PIC programmer to burn it into the PIC.
If your PIC already has a version with bootloader in it, you can also 
use UAVPset to flash the new HEX-file on the board, so there is no 
special programming hardware needed.

==============================================
= Transferring firmware to the PIC           =
==============================================

If you already have a PIC with at least software version 3.05,
You can upload your new version using the bootloader.
Use UAVPset.exe to do that.

But be careful: If you mock up things completely, you can lock
yourself out. Then you would need a PIC programmer to get things working again.


==============================================
= SAFETY FIRST!                              =
==============================================

If you work on your ufo model, PLEASE TAKE CARE!

Nowadays, extremely powerful motors and accumulators
can pose a serious threat to your and others safety.

Safety measures:
- Remove the propellers before you test parameter or firmware modifications
- Work with a good bench power supply instead of an accupack.
  A short circuit will then not be able to destroy your hardware too easily
- Your ufo mainboards electronics can be damaged by 
  electrostatic discharge sparks. Make sure you are doing any
  work at the electronics in an ESD safe workplace.
- When using your transmitter, be sure to use the correct model memory on it.
- Be sure to have no other transmitters on your channel on air.

The author will NOT BE RESPONSIBLE FOR ANY ACCIDENTS OR
INJURIES IN ANY WAY!

PLEASE do yourself a favour and get an insurance which covers risks by doing model flight!
Imaging what happens if you only damage someone elses car or something....


==============================================
= Questions?                                 =
==============================================

If you have any questions, please go to the forum at 
http://www.opensourcequadrocopter.de 
and feel free to ask. There are a lot of people that will happily help you,
including the author (his nickname is "Wolferl").

However, a registration is required to gain access to the forum.

Alternatively, you can contact the author by e-mail at
wolfgang@opensourcequadrocopter.de

BUT PLEASE, BEFORE YOU ASK, READ THE MANUAL.
It is all in there, really!

Also, if you have some hardware related questions, please download the 
datasheets for the chips and read them thoroughly!

Make sure you have the most recent version of the firmware!

==============================================
= End of README file                         =
==============================================
