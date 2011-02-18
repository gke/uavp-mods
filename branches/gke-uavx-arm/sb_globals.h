/*
    Copyright (c) 2010 Andy Kirkham
 
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
 
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
 
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef SB_GLOBALS_G
#define SB_GLOBALS_G

#define SERIALBUFFERED_BUFFER_SIZE 256

#define SET_REGISTER(reg, val)  *(uint32_t *)(uart_base+reg)=val
#define GET_REGISTER(reg)       *(uint32_t *)(uart_base+reg)

#define SERIALBUFFERED_RBR  0x00
#define SERIALBUFFERED_THR  0x00
#define SERIALBUFFERED_DLL  0x00
#define SERIALBUFFERED_IER  0x04
#define SERIALBUFFERED_DML  0x04
#define SERIALBUFFERED_IIR  0x08
#define SERIALBUFFERED_FCR  0x08
#define SERIALBUFFERED_LCR  0x0C
#define SERIALBUFFERED_LSR  0x14
#define SERIALBUFFERED_SCR  0x1C
#define SERIALBUFFERED_ACR  0x20
#define SERIALBUFFERED_ICR  0x24
#define SERIALBUFFERED_FDR  0x28
#define SERIALBUFFERED_TER  0x30

extern char *_tx_buffer[4];
extern int   _tx_buffer_size[4];
extern int   _tx_buffer_in[4];
extern int   _tx_buffer_out[4];
extern bool  _tx_buffer_full[4];
extern bool  _tx_buffer_overflow[4];
extern bool  _tx_buffer_used_malloc[4];

extern char *_rx_buffer[4];        
extern int   _rx_buffer_size[4];
extern int   _rx_buffer_in[4];
extern int   _rx_buffer_out[4];
extern bool  _rx_buffer_full[4];
extern bool  _rx_buffer_overflow[4];
extern bool  _rx_buffer_used_malloc[4];

#endif
