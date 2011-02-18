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

/* Module globals, defined here rather than as private
   variables in the class so the C interrupt routine can
   access these variables rather than trying to discover 
   where they are at runtime. */
char *_tx_buffer[4];
int   _tx_buffer_size[4];
int   _tx_buffer_in[4];
int   _tx_buffer_out[4];
bool  _tx_buffer_full[4];
bool  _tx_buffer_overflow[4];
bool  _tx_buffer_used_malloc[4];

char *_rx_buffer[4];        
int   _rx_buffer_size[4];
int   _rx_buffer_in[4];
int   _rx_buffer_out[4];
bool  _rx_buffer_full[4];
bool  _rx_buffer_overflow[4];
bool  _rx_buffer_used_malloc[4];

