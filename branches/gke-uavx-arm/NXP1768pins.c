// ===============================================================================================
// =                              UAVXArm Quadrocopter Controller                                =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVXArm.

//    UAVXArm is free software: you can redistribute it and/or modify it under the terms of the GNU
//    General Public License as published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.

//    UAVXArm is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/

//    IO functions for NXP LPC1768.

//    See also: http://bitbucket.org/jpc/lpc1768/

//    Original Copyright (c) 2010 LoEE - Jakub Piotr C&#322;apa
//    This program was released under the new BSD license.

//    Rewritten for UAVXArm by G.K Egan 2011


#include "UAVXArm.h"

boolean PinRead(uint8);
void PinWrite(uint8, boolean);
void PinSetOutput(uint8, boolean);

//extern __inline__ __attribute__((always_inline))


boolean PinRead(uint8 pn) {

    static uint8 p,m;
    p = pn >> 5;
    m = 1 << ( pn & 0x1f );

    switch ( p ) {
        case 0:
            return ( LPC_GPIO0->FIOPIN & m ) != 0;
        case 1:
            return ( LPC_GPIO1->FIOPIN & m ) != 0;
        case 2:
            return ( LPC_GPIO2->FIOPIN & m ) != 0;
        case 3:
            return ( LPC_GPIO3->FIOPIN & m ) != 0;
        case 4:
            return ( LPC_GPIO4->FIOPIN & m ) != 0;
        default:
            return (0);
    }
} // PinRead

void PinWrite(uint8 pn, boolean v) {

    static uint8 p, m;
    p = pn >> 5;
    m = 1 << ( pn & 0x1f );

    switch ( p ) {
        case 0:
            if ( v )
                LPC_GPIO0->FIOSET = m;
            else
                LPC_GPIO0->FIOCLR = m;
            break;
        case 1:
            if ( v )
                LPC_GPIO1->FIOSET = m;
            else
                LPC_GPIO1->FIOCLR = m;
            break;
        case 2:
            if ( v )
                LPC_GPIO2->FIOSET = m;
            else
                LPC_GPIO2->FIOCLR = m;
            break;
        case 3:
            if ( v )
                LPC_GPIO3->FIOSET = m;
            else
                LPC_GPIO3->FIOCLR = m;
            break;
        case 4:
            if ( v )
                LPC_GPIO4->FIOSET = m;
            else
                LPC_GPIO4->FIOCLR = m;
            break;
        default:
            break;
    }
} // PinWrite

void PinSetOutput(uint8 pn, boolean PinIsOutput) {

    static uint8 p,m;
    p = pn >> 5;
    m = 1 << ( pn & 0x1f );

    switch ( p ) {
        case 0:
            if ( PinIsOutput )
                LPC_GPIO0->FIODIR |= m;
            else
                LPC_GPIO0->FIODIR &= ~m;
            break;
        case 1:
            if ( PinIsOutput )
                LPC_GPIO1->FIODIR |= m;
            else
                LPC_GPIO1->FIODIR &= ~m;
            break;
        case 2:
            if ( PinIsOutput )
                LPC_GPIO2->FIODIR |= m;
            else
                LPC_GPIO2->FIODIR &= ~m;
            break;
        case 3:
            if ( PinIsOutput )
                LPC_GPIO3->FIODIR |= m;
            else
                LPC_GPIO3->FIODIR &= ~m;
            break;
        case 4:
            if ( PinIsOutput )
                LPC_GPIO4->FIODIR |= m;
            else
                LPC_GPIO4->FIODIR &= ~m;
            break;
        default:
            break;
    }
} // PinSetOutput



