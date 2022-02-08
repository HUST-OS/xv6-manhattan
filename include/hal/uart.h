// The general interface of serial.
//
// This interface should access UART directly, instead of using
// SBI calls. In this way the kernel will avoid the cost brought
// by trap and context saving.
//
// But because it's accessing the

#ifndef __UART_H
#define __UART_H

void uart_init(int baudrate);

// Getting a char from UART is NON-BLOCK. When there's no char in
// the UART FIFO, -1 is returned. You may surround it with a while
// loop to get blocked.
int uart_getchar(void);

// Putting a char is BLOCKING.
void uart_putchar(char c);

#endif
