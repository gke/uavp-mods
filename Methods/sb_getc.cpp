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

int SerialBuffered::getc(bool blocking) {
    char c;
    
    if (blocking) while (_rx_buffer_out[uart_number] == _rx_buffer_in[uart_number] && !_rx_buffer_full[uart_number]) ; /* Blocks! */    
    else if (_rx_buffer_in[uart_number] == _rx_buffer_out[uart_number] && !_rx_buffer_full[uart_number]) return -1;
    
    c = _rx_buffer[uart_number][_rx_buffer_out[uart_number]++];
    if (_rx_buffer_out[uart_number] >= _rx_buffer_size[uart_number]) {
        _rx_buffer_out[uart_number] = 0;
    } 
    if (_rx_buffer_full[uart_number]) _rx_buffer_full[uart_number] = false;     
    return (int)c;   
}

char SerialBuffered::getc(void) {
    return (char)getc(true);
}

int SerialBuffered::readable(void) {
    return (_rx_buffer_full[uart_number] || _rx_buffer_overflow[uart_number] || _rx_buffer_in[uart_number] != _rx_buffer_out[uart_number]) ? 1 : 0;
}
