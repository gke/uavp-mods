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

#ifndef SERIALBUFFERED_H
#define SERIALBUFFERED_H

#include "mbed.h"



/** SerialBuffered based on Serial but fully buffered IO
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "SerialBuffered.h"
 *
 * SerialBuffered serial1 (p13, p14); 
 * SerialBuffered serial2 (p28, p27);
 *
 * int main() {
 *     while(1) {
 *         if (serial1.readable()) {
 *             while (!serial2.writeable());
 *             serial2.putc(serial1.getch());
 *         }
 *         if (serial2.readable()) {
 *             while (!serial1.writeable());
 *             serial1.putc(serial2.getc());
 *         }
 *     }
 * }
 * @endcode
 *
 * <b>Note</b>, because this system "traps" the interrupts for the UART
 * being used <b>do not</b> use the .attach() method, otherwise the buffers
 * will cease functioning. Or worse, behaviour becomes unpredictable.
 */

class SerialBuffered : public Serial {

    public:
        enum { None = 0, One = 1, Two = 2 };
        enum { WordLength5 = 0, WordLength6 = 1, WordLength7 = 2, WordLength8 = 3 };
        enum { NoParity = 0, OddParity = (1UL << 3), EvenParity = (3UL << 3), Forced1 = (5UL << 3), Forced0 = (7UL << 3) };
        enum { StopBit1 = (0UL << 2), StopBit2 = (1UL << 2) };
        
        /** Create a SerialBuffered object connected to the specified pins
         *
         * @param PinName tx The Mbed TX pin for the uart port.
         * @param PinName rx The Mbed RX pin for the uart port.
         */
        SerialBuffered(PinName tx, PinName rx);
        
        virtual ~SerialBuffered();
        
        /** Get a character from the serial stream.
         *
         * @return char A char value of the character read.
         */
        char getc(void);
        
        /** Gets a character from the serial stream with optional blocking.
         *
         * This method allows for getting a character from the serial stream
         * if one is available. If <b>blocking</b> is true, the method will
         * wait for serial input if the RX buffer is empty. If <b>blocking</b>
         * is false, the method will return immediately if the RX buffer is empty.
         * On return, if not blocking and the buffer was empty, -1 is returned.
         *
         * @param blocking true or false, when true will block.
         * @return int An int representation of the 8bit char or -1 on buffer empty.
         */
        int  getc(bool blocking);
        
        /** Puts a characher to the serial stream.
         *
         * This sends a character out of the uart port or, if no room in the
         * TX FIFO, will place the character into the TX buffer. <b>Note</b>, if the
         * TX buffer is also full, this method will <b>block</b> (wait) until
         * there is room in the buffer.
         *
         * @param int An int representation of the 8bit character to send.
         * @return int Always returns zero.
         */
        int  putc(int c);
        
        /** Puts a characher to the serial stream.
         *
         * As with putc(int c) this function allows for a character to be sent
         * to the uart TX port. However, an extra parameter is added to allow
         * the caller to decide if the method should block or not. If blocking
         * is disabled then this method returns -1 to signal there was no room 
         * in the TX FIFO or the TX buffer. The character c passed is has not
         * therefore been sent.
         *
         * @param int An int representation of the 8bit character to send.
         * @param bool true if blocking required, false to disable blocking.
         * @return int Zero on success, -1 if no room in TX FIFO or TX buffer.
         */
        int  putc(int c, bool blocking);
        
        /** Are there characters in the RX buffer we can read?
         *
         * @return int 1 if characters are available, 0 otherwise.
         */
        int  readable(void);
        
        /** Is there room in the TX buffer to send a character?
         *
         * @return int 1 if room available, 0 otherwise.
         */
        int  writeable(void);
        
        /** Set's the UART baud rate.
         *
         * Any allowed baudrate may be passed. However, you should
         * ensure it matches the far end of the serial link.
         * 
         * @param int The baudrate to set.
         */
        void baud(int baudrate);
        
        /** Sets the serial format.
         *
         * Valid serial bit lengths are:-
         * <ul>
         *  <li>SerialBuffered::WordLength5</li>
         *  <li>SerialBuffered::WordLength6</li>
         *  <li>SerialBuffered::WordLength7</li>
         *  <li>SerialBuffered::WordLength8</li>
         * </ul>
         *
         * Valid serial parity are:-
         * <ul>
         *  <li>SerialBuffered::NoParity</li>
         *  <li>SerialBuffered::OddParity</li>
         *  <li>SerialBuffered::EvenParity</li>
         *  <li>SerialBuffered::Forced1</li>
         *  <li>SerialBuffered::Forced0</li>
         * </ul>
         *
         * Valid stop bits are:-
         * <ul>
         *  <li>SerialBuffered::None</li>
         *  <li>SerialBuffered::One</li>
         *  <li>SerialBuffered::Two</li>
         * </ul>
         *
         * @param int bits
         * @param int parity
         * @param int stopbits
         */
        void format(int bits, int parity, int stopbits);
        
        /** Change the TX buffer size
         *
         * By default, when the SerialBuffer object is created, a default
         * TX buffer of 256 bytes in size is created. If you need a bigger
         * (or smaller) buffer then use this function to change the TX buffer
         * size. 
         *
         * <b>Note</b>, when a buffer is resized, any previous buffer
         * in operation is discarded (destroyed and lost).
         *
         * @param int The size of the TX buffer required.
         */
        void set_tx_buffer_size(int buffer_size);
        
        /** Change the TX buffer size and provide your own allocation.
         *
         * This methos allows for the buffer size to be changed and for the
         * caller to pass a pointer to an area of RAM they have already set
         * aside to hold the buffer. The standard method is to malloc space
         * from the heap. This method allows that to be overriden and use a
         * user supplied buffer. 
         *
         * <b>Note</b>, the buffer you create must be of the size you specify!
         * <b>Note</b>, when a buffer is resized, any previous buffer
         * in operation is discarded (destroyed and lost).
         *
         * @param int The size of the TX buffer required.
         * @param char* A pointer to a buffer area you previously allocated.
         */
        void set_tx_buffer_size(int buffer_size, char *buffer);
        
        /** Change the RX buffer size
         *
         * By default, when the SerialBuffer object is created, a default
         * RX buffer of 256 bytes in size is created. If you need a bigger
         * (or smaller) buffer then use this function to change the RX buffer
         * size. 
         *
         * <b>Note</b>, when a buffer is resized, any previous buffer
         * in operation is discarded (destroyed and lost).
         *
         * @param int The size of the RX buffer required.
         */
        void set_rx_buffer_size(int buffer_size);
        
        /** Change the RX buffer size and provide your own allocation.
         *
         * This methos allows for the buffer size to be changed and for the
         * caller to pass a pointer to an area of RAM they have already set
         * aside to hold the buffer. The standard method is to malloc space
         * from the heap. This method allows that to be overriden and use a
         * user supplied buffer. 
         *
         * <b>Note</b>, the buffer you create must be of the size you specify!
         * <b>Note</b>, when a buffer is resized, any previous buffer
         * in operation is discarded (destroyed and lost).
         *
         * @param int The size of the RX buffer required.
         * @param char* A pointer to a buffer area you previously allocated.
         */
        void set_rx_buffer_size(int buffer_size, char *buffer);
        
    protected:
        /** Calculate the divisors for a UART's baud
         *
         * @param int The desired baud rate
         * @param int The UART in use to calculate for
         */
        uint16_t calculate_baud(int baud, int uart);
            
    private:
        /** set_uart0
         *
         * Sets up the hardware and interrupts for the UART.
         *
         * @param PinName tx
         * @param PinName rx
         */
        void set_uart0(PinName tx, PinName rx);
        
        /** set_uart1
         *
         * Sets up the hardware and interrupts for the UART.
         *
         * @param PinName tx
         * @param PinName rx
         */
        void set_uart1(PinName tx, PinName rx);
        
        /** set_uart2
         *
         * Sets up the hardware and interrupts for the UART.
         *
         * @param PinName tx
         * @param PinName rx
         */
        void set_uart2(PinName tx, PinName rx);
        
        /** set_uart3
         *
         * Sets up the hardware and interrupts for the UART.
         *
         * @param PinName tx
         * @param PinName rx
         */
        void set_uart3(PinName tx, PinName rx);
        
        /** Reset the TX Buffer
         *
         * @param int The UART buffer to reset.
         */
        void reset_uart_tx(int uart_number);
        
        /** Reset the RX Buffer
         *
         * @param int The UART buffer to reset.
         */
        void reset_uart_rx(int uart_number);
        
        /** LPC1768 UART peripheral base address for UART in use.
         */
        unsigned long uart_base;
        
        /** LPC1768 UART number for UART in use.
         */
        int uart_number;
};

#endif
