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

void SerialBuffered::format(int bits, int parity, int stopbits) {
    SET_REGISTER(SERIALBUFFERED_LCR, (bits | parity | stopbits) & 0x7F);
}

void SerialBuffered::baud(int baud) {
    uint16_t rate = calculate_baud(baud, uart_number);        
    SET_REGISTER(SERIALBUFFERED_LCR, GET_REGISTER(SERIALBUFFERED_LCR) | 0x80);
    SET_REGISTER(SERIALBUFFERED_DML, (rate >> 8) & 0xFF);
    SET_REGISTER(SERIALBUFFERED_DLL, (rate &0xFF));
    SET_REGISTER(SERIALBUFFERED_LCR, GET_REGISTER(SERIALBUFFERED_LCR) & 0x7F);
}

uint16_t SerialBuffered::calculate_baud(int baud, int uart) {
    static int multipliers[4] = { 4, 1, 2, 8 };
    int clock = 0;
    
    switch (uart) {
        case 0: clock = (LPC_SC->PCLKSEL0 >>  6) & 0x3; break;
        case 1: clock = (LPC_SC->PCLKSEL0 >>  8) & 0x3; break;
        case 2: clock = (LPC_SC->PCLKSEL1 >> 16) & 0x3; break;
        case 3: clock = (LPC_SC->PCLKSEL1 >> 18) & 0x3; break;
    }
    
    return (uint16_t)(((SystemCoreClock / 16 / baud) * multipliers[clock]) & 0xFFFF);
}

void SerialBuffered::set_tx_buffer_size(int buffer_size, char *buffer) {
    SET_REGISTER(SERIALBUFFERED_IER, GET_REGISTER(SERIALBUFFERED_IER) & (1UL << 2));
    SET_REGISTER(SERIALBUFFERED_FCR, (1UL << 2) | 1);
    _tx_buffer_size[uart_number] = buffer_size;
    if (_tx_buffer_used_malloc[uart_number] && _tx_buffer[uart_number]) {
        free(_tx_buffer[uart_number]);
        _tx_buffer_used_malloc[uart_number] = false;
    }
    if (buffer == (char *)NULL) {
        _tx_buffer[uart_number] = (char *)malloc(buffer_size);
        _tx_buffer_used_malloc[uart_number] = true;
    }
    else {
        _tx_buffer[uart_number] = buffer;
        _tx_buffer_used_malloc[uart_number] = false;
    }
    _tx_buffer_in[uart_number] = _tx_buffer_out[uart_number] = 0;
    _tx_buffer_full[uart_number] = false;
    SET_REGISTER(SERIALBUFFERED_IER, GET_REGISTER(SERIALBUFFERED_IER) | 0x2);
}

void SerialBuffered::set_tx_buffer_size(int buffer_size) {
    set_tx_buffer_size(buffer_size, (char *)NULL);
}

/**
 */
void SerialBuffered::set_rx_buffer_size(int buffer_size, char *buffer) {
    SET_REGISTER(SERIALBUFFERED_IER, GET_REGISTER(SERIALBUFFERED_IER) & (1UL << 1));
    SET_REGISTER(SERIALBUFFERED_FCR, (1UL << 1) | 1);
    _rx_buffer_size[uart_number] = buffer_size;
    if (_rx_buffer_used_malloc[uart_number] && _rx_buffer[uart_number]) {
        free(_rx_buffer[uart_number]);
        _rx_buffer_used_malloc[uart_number] = false;
    }
    if (buffer == (char *)NULL) {
        _rx_buffer[uart_number] = (char *)malloc(buffer_size);
        _rx_buffer_used_malloc[uart_number] = true;
    }
    else {
        _rx_buffer[uart_number] = buffer;
        _rx_buffer_used_malloc[uart_number] = false;
    }
    _rx_buffer_in[uart_number] = _rx_buffer_out[uart_number] = 0;
    _rx_buffer_full[uart_number] = false;
    SET_REGISTER(SERIALBUFFERED_IER, GET_REGISTER(SERIALBUFFERED_IER) | 0x1); 
}

void SerialBuffered::set_rx_buffer_size(int buffer_size) {
    set_rx_buffer_size(buffer_size, (char *)NULL);
}


void SerialBuffered::reset_uart_tx(int uart_number) {
    SET_REGISTER(SERIALBUFFERED_IER, GET_REGISTER(SERIALBUFFERED_IER) & (1UL << 1));
    SET_REGISTER(SERIALBUFFERED_FCR, (1UL << 2) | 1);
    if (_tx_buffer_used_malloc[uart_number] && _tx_buffer[uart_number]) {
        free(_tx_buffer[uart_number]);
        _tx_buffer[uart_number] = (char *)NULL;
        _tx_buffer_used_malloc[uart_number] = false;
    }
    _tx_buffer_in[uart_number] = _tx_buffer_out[uart_number] = 0;
    _tx_buffer_full[uart_number] = false;
}

void SerialBuffered::reset_uart_rx(int uart_number) {
    SET_REGISTER(SERIALBUFFERED_IER, GET_REGISTER(SERIALBUFFERED_IER) & (1UL << 0));
    SET_REGISTER(SERIALBUFFERED_FCR, (1UL << 1) | 1);    
    if (_rx_buffer_used_malloc[uart_number] && _rx_buffer[uart_number]) {
        free(_rx_buffer[uart_number]);
        _rx_buffer[uart_number] = (char *)NULL;
        _rx_buffer_used_malloc[uart_number] = false;
    }
    _rx_buffer_in[uart_number] = _rx_buffer_out[uart_number] = 0;
    _rx_buffer_full[uart_number] = false;
}
