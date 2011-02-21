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

extern "C" static void SerialBuffered_ISR_1(void) __irq { SerialBuffered_ISR(LPC_UART1_BASE, 1); }

void SerialBuffered::set_uart1(PinName tx, PinName rx) {
    unsigned ier = 0;
    
    if (tx == NC && rx == NC) return; // Eh, why would one?

    LPC_SC->PCONP       |=  (1UL << 4);
    LPC_SC->PCLKSEL0    &= ~(3UL << 8);
    LPC_SC->PCLKSEL0    |=  (1UL << 8);
    
    if (rx == p14) { 
        set_rx_buffer_size(SERIALBUFFERED_BUFFER_SIZE); 
        LPC_PINCON->PINSEL1 &= ~(3UL << 0); 
        LPC_PINCON->PINSEL1 |=  (1UL << 0); 
        ier |= 1; 
    }
    else if (rx == p25) { 
        set_rx_buffer_size(SERIALBUFFERED_BUFFER_SIZE); 
        LPC_PINCON->PINSEL4 &= ~(3UL << 2); 
        LPC_PINCON->PINSEL4 |=  (2UL << 2); 
        ier |= 1; 
    }
    else { reset_uart_rx(1); }
    
    if (tx == p13) { 
        set_tx_buffer_size(SERIALBUFFERED_BUFFER_SIZE); 
        LPC_PINCON->PINSEL0 &= ~(3UL << 30); 
        LPC_PINCON->PINSEL0 |= (1UL << 30); 
        ier |= 2; 
    }
    else if (tx == p26) { 
        set_tx_buffer_size(SERIALBUFFERED_BUFFER_SIZE); 
        LPC_PINCON->PINSEL4 &= ~(3UL <<  0); 
        LPC_PINCON->PINSEL4 |= (2UL <<  0); 
        ier |= 2; 
    }
    else { reset_uart_tx(1); }
    
    if (ier) {
        baud(9600);
        format(SerialBuffered::WordLength8, SerialBuffered::NoParity, SerialBuffered::StopBit1);
        LPC_UART1->FCR = 0x7;
        NVIC_SetVector(UART1_IRQn, (uint32_t)SerialBuffered_ISR_1);
        NVIC_EnableIRQ(UART1_IRQn);    
        LPC_UART1->IER = ier;
    }
}
