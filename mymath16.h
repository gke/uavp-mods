// ==============================================
// =      U.A.V.P Brushless UFO Controller      =
// =           Professional Version             =
// = Copyright (c) 2007 Ing. Wolfgang Mahringer =
// ==============================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// ==============================================
// =  please visit http://www.uavp.de           =
// =               http://www.mahringer.co.at   =
// ==============================================

// to define the bank for the variables, set MATHBANK_VARS
// to shrBank, bank0, bank1, bank2, or bank3
// before including this .h file!

// to define the bank for the math routine itself,
// set MATHBANK_PROG to 0, 1, 2, or 3
// before including this .h file!


// Sanity checks
#ifndef MATHBANK_VARS
#define MATHBANK_VARS bank0
#endif

#ifndef MATHBANK_PROG
#define MATHBANK_PROG 0
#endif

#if MATHBANK_PROG == 0
#define MATHBANK_PAGE page0
#endif
#if MATHBANK_PROG == 1
#define MATHBANK_PAGE page1
#endif
#if MATHBANK_PROG == 2
#define MATHBANK_PAGE page2
#endif
#if MATHBANK_PROG == 3
#define MATHBANK_PAGE page3
#endif

//extern MATHBANK_VARS uns16 nilarg1;
//extern MATHBANK_VARS uns16 nilarg2;
extern MATHBANK_VARS uns16 nilrval;
extern MATHBANK_VARS uns8  rm;
extern MATHBANK_VARS uns16 rm16;
extern MATHBANK_VARS int8  tmpArg2;
extern MATHBANK_VARS char  sign;

#define MBV 1

#if MBV == 0
#define nilarg1	0x6C
#define nilarg2 0x6E
#endif
#if MBV == 1
#define nilarg1	0xDC
#define nilarg2 0xDE
#endif
#if MBV == 2
#define nilarg1	0x16C
#define nilarg2 0x16E
#endif
#if MBV == 3
#define nilarg1	0x1DC
#define nilarg2 0x1DE
#endif

extern MATHBANK_PAGE void MathMultU8x8(void);
extern MATHBANK_PAGE void MathMultS8x8(void);
extern MATHBANK_PAGE void MathMultU16x8(void);
extern MATHBANK_PAGE void MathMultU16x16(void);
extern MATHBANK_PAGE void MathDivU16_8(void);
extern MATHBANK_PAGE void MathDivU16_16(void);
extern MATHBANK_PAGE void MathDivS16_8(void);
extern MATHBANK_PAGE void MathDivS16_16(void);


uns16 operator* (uns8 arg1@nilarg1, uns8 arg2@nilarg2) @
int8 operator* (int8 arg1@nilarg1, int8 arg2@nilarg2)
{
//	nilarg1.low8 = arg1;
//	nilarg2.low8 = arg2;
	MathMultU8x8();	// passt scho so!
	return nilrval;
}

int16 operator* (int8 arg1@nilarg1, int8 arg2@nilarg2)
{
//	nilarg1.low8 = arg1;
//	nilarg2.low8 = arg2;
	MathMultS8x8();
	return nilrval;
}

uns16 operator* (uns8 arg1@nilarg2, uns16 arg2@nilarg1)
{
//	nilarg2.low8 = arg1;
//	nilarg1 = arg2;
	MathMultU16x8();
	return nilrval;
}

uns16 operator* (uns16 arg1@nilarg1, uns8 arg2@nilarg2)
{
//	nilarg2.low8 = arg2;
//	nilarg1 = arg1;
	MathMultU16x8();
	return nilrval;
}

uns16 operator* (uns16 arg1@nilarg1, uns16 arg2@nilarg2) @
int16 operator* (int16 arg1@nilarg1, int16 arg2@nilarg2)
{
//	nilarg1 = arg1;
//	nilarg2 = arg2;
	MathMultU16x16();
	return nilrval;
}

uns16 operator/ (uns16 arg1@nilarg1, uns8 arg2@nilarg2)
{
//	nilarg2 = W;
//	nilarg1 = arg1;
	MathDivU16_8();
	return nilrval;
} 

uns16 operator/ (uns16 arg1@nilarg1, uns16 arg2@nilarg2)
{
//	nilarg1 = arg1;
//	nilarg2 = arg2;
	MathDivU16_16();
	return arg1;
}

int8  operator/ (int8 arg1@nilarg1, int8 arg2@nilarg2)
{
//	nilarg2.low8 = arg2;
//	(long)nilarg1 = arg1;	
// CHECK!
	MathDivS16_8();
	return arg1.low8;
}

int16 operator/ (int16 arg1@nilarg1, int8 arg2@nilarg2)
{
//	nilarg2.low8 = arg2;
//	nilarg1 = arg1;
	MathDivS16_8();
	return arg1;
}

int16 operator/ (int16 arg1@nilarg1, int16 arg2@nilarg2)
{
//	nilarg1 = arg1;
//	nilarg2 = arg2;
	MathDivS16_16();
	return arg1;
}


