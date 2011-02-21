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

int SerialBuffered::putc(int c, bool blocking) {
    if ((GET_REGISTER(SERIALBUFFERED_LSR) & 0x20) && (_tx_buffer_in[uart_number] == _tx_buffer_out[uart_number] && !_tx_buffer_full[uart_number])) {
        SET_REGISTER(SERIALBUFFERED_THR, (uint8_t)c);
    }
    else {
        if (_tx_buffer_full[uart_number]) {
            if (blocking) while (_tx_buffer_full[uart_number]) ; /* Blocks!!! */    
            else {
                _tx_buffer_overflow[uart_number] = true;
                return -1;
            }
        }  
        _tx_buffer[uart_number][_tx_buffer_in[uart_number]++] = (char)c;
        if (_tx_buffer_in[uart_number] >= _tx_buffer_size[uart_number]) {
            _tx_buffer_in[uart_number] = 0;
        }
        if (_tx_buffer_in[uart_number] == _tx_buffer_out[uart_number] && !_tx_buffer_full[uart_number]) _tx_buffer_full[uart_number] = true;
        SET_REGISTER(SERIALBUFFERED_IER, 0x3);        
    }
    return 0;
}

/** putc
 */
int SerialBuffered::putc(int c) {
    return putc(c, true);
}

int SerialBuffered::writeable(void) {
    return (!_tx_buffer_full[uart_number] && !_tx_buffer_overflow[uart_number]) ? 1 : 0;
}
