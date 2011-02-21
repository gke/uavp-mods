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

#include "../SerialBuffered.h"
#include "../sb_globals.h"

extern "C" void SerialBuffered_ISR(unsigned long, int);

extern "C" static void SerialBuffered_ISR_2(void) __irq { SerialBuffered_ISR(LPC_UART2_BASE, 2); }

void SerialBuffered::set_uart2(PinName tx, PinName rx) {
    volatile char c __attribute__((unused));
    unsigned ier = 0;

    if (tx == NC && rx == NC) return; // Eh, why would one?

    LPC_SC->PCONP       |=  (1UL << 24);
    LPC_SC->PCLKSEL1    &= ~(3UL << 16);
    LPC_SC->PCLKSEL1    |=  (1UL << 16);
    if (rx == p27) { 
        set_rx_buffer_size(SERIALBUFFERED_BUFFER_SIZE); 
        LPC_PINCON->PINSEL0 &= ~(3UL << 22); 
        LPC_PINCON->PINSEL0 |=  (1UL << 22); 
        ier |= 1; 
    }
    else { reset_uart_rx(2); }
    if (tx == p28) { 
        set_tx_buffer_size(SERIALBUFFERED_BUFFER_SIZE); 
        LPC_PINCON->PINSEL0 &= ~(3UL << 20); 
        LPC_PINCON->PINSEL0 |= (1UL << 20); 
        ier |= 2; 
    }
    else { reset_uart_tx(2); }
    
    if (ier) {
        baud(9600);
        format(SerialBuffered::WordLength8, SerialBuffered::NoParity, SerialBuffered::StopBit1);
        LPC_UART2->FCR = 0x7;
        NVIC_SetVector(UART2_IRQn, (uint32_t)SerialBuffered_ISR_2);
        NVIC_EnableIRQ(UART2_IRQn);    
        LPC_UART2->IER = ier;
    }
}

