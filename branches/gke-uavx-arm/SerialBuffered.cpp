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


#include "mbed.h"

#ifndef SERIALBUFFERED_C
#define SERIALBUFFERED_C
#endif

#include "SerialBuffered.h"
#include "sb_globals.h"

enum { None, One, Two };
enum { WordLength5, WordLength6, WordLength7, WordLength8 };
enum { NoParity, OddParity, EvenParity, Forced1, Forced0 };
enum { StopBit1, StopBit2 };
        
SerialBuffered::SerialBuffered(PinName tx, PinName rx) : Serial(tx, rx) {

    // Depending upon the pins used, set-up the Uart.
    if (tx == p9 || rx == p10) {
        uart_number = 3;
        uart_base = LPC_UART3_BASE; 
        set_uart3(tx, rx);
    }
    else if (tx == p13 || tx == p26 || rx == p14 || rx == p25) {        
        uart_number = 1;
        uart_base = LPC_UART1_BASE; 
        set_uart1(tx, rx);
    }
    else if (tx == p28 || rx == p27) {
        uart_number = 2;
        uart_base = LPC_UART2_BASE; 
        set_uart2(tx, rx);
    }
    else if (tx == USBTX || rx == USBRX) {
        uart_number = 0;
        uart_base = LPC_UART0_BASE; 
        set_uart0(tx, rx);
    }
    else {
        uart_number = -1;
        return;
    }
}

SerialBuffered::~SerialBuffered() {
    if (_tx_buffer_used_malloc[uart_number] && _tx_buffer[uart_number]) free(_tx_buffer[uart_number]);
    if (_rx_buffer_used_malloc[uart_number] && _rx_buffer[uart_number]) free(_rx_buffer[uart_number]);
}
        
/** SerialBuffered_ISR
 *
 * The main Uart interrupt handler.
 */
extern "C" void SerialBuffered_ISR(unsigned long uart_base, int uart_number) {
    char c __attribute__((unused));
    volatile uint32_t iir, lsr;
    
    iir = GET_REGISTER(SERIALBUFFERED_IIR); 

    if (iir & 1) {        
        return;    /* Eh, wtf? */
    }
    
    iir = (iir >> 1) & 0x3;   
    
    if (iir == 2) {
        while(GET_REGISTER(SERIALBUFFERED_LSR) & 0x1) {
            if (_rx_buffer_in[uart_number] == _rx_buffer_out[uart_number] && _rx_buffer_full[uart_number]) {            
                c = GET_REGISTER(SERIALBUFFERED_RBR); /* Oh dear, we need a bigger buffer!, send to dev/null */
                _rx_buffer_overflow[uart_number] = true;
            }
            else {
                if (_rx_buffer[uart_number]) { /* Ensure buffer pointer is not null before use. */
                    c = GET_REGISTER(SERIALBUFFERED_RBR);
                    _rx_buffer[uart_number][_rx_buffer_in[uart_number]++] = c;
                    if (_rx_buffer_in[uart_number] >= _rx_buffer_size[uart_number]) _rx_buffer_in[uart_number] = 0;
                    if (_rx_buffer_in[uart_number] == _rx_buffer_out[uart_number]) _rx_buffer_full[uart_number] = true;
                }
            }
        }
    }
    
    if (iir == 1) {        
        if (_tx_buffer[uart_number]) { /* Ensure buffer pointer is not null before use. */
            if (_tx_buffer_in[uart_number] != _tx_buffer_out[uart_number] || _tx_buffer_full[uart_number]) {
                SET_REGISTER(SERIALBUFFERED_THR, (uint32_t)(_tx_buffer[uart_number][_tx_buffer_out[uart_number]++]));
                if (_tx_buffer_out[uart_number] >= _tx_buffer_size[uart_number]) _tx_buffer_out[uart_number] = 0;
                _tx_buffer_full[uart_number] = false;            
            }
            else {
                SET_REGISTER(SERIALBUFFERED_IER, 1);
            }                
        }
    }
    
    if (iir == 3) {  
        /* We need to provide an error handling system. For now, just dismiss the IRQ. */       
        lsr = GET_REGISTER(SERIALBUFFERED_LSR);
    } 
}

