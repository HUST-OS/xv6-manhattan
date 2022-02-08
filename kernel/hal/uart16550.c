#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "hal/uart.h"

#define __module_name__     "uart16550"

// retrhelo:
// You may notice that we use a virtual address here as the base of UART16550,
// So this
#define UART_BASE       (UART_V)

#define RHR 0    // Receive Holding Register (read mode)
#define THR 0    // Transmit Holding Register (write mode)
#define DLL 0    // LSB of Divisor Latch (write mode)
#define IER 1    // Interrupt Enable Register (write mode)
#define DLM 1    // MSB of Divisor Latch (write mode)
#define FCR 2    // FIFO Control Register (write mode)
#define ISR 2    // Interrupt Status Register (read mode)
#define LCR 3    // Line Control Register
#define MCR 4    // Modem Control Register
#define LSR 5    // Line Status Register
#define MSR 6    // Modem Status Register
#define SPR 7    // ScratchPad Register

#define UART_REG(reg)   ((volatile uint8 *)(UART_BASE + reg))

#define LSR_RX_READY    (1 << 0)
#define LSR_TX_IDLE     (1 << 5)

#define IER_RHR         (1 << 0)    // receive holding register interrupt
#define IER_THR         (1 << 1)    // transmit holding register interrupt
#define IER_RLS         (1 << 2)    // receive line status interrupt
#define IER_MS          (1 << 3)    // modem status interrupt

#define uart_read_reg(reg) (*(UART_REG(reg)))
#define uart_write_reg(reg, v) (*(UART_REG(reg)) = (v))

// retrhelo:
// Most of the UART initialization will be done in OpenSBI, as far as
// I see. However, due to some reasons OpenSBI doesn't enable interrupts.
// Due to this reason, I think we'd better re-init UART in the kernel to
// make sure it behaves as we expect.
//
// On QEMU platform, the baudrate means little... Whatever the baudrate is,
// you can always get the correct output. But be careful with a real machine!
//
// You may find more details about Ns16556 at this website:
// http://byterunner.com/16550.html
void uart_init(int baudrate) {
    // calculate baudrate divider
    uint8 lcr = uart_read_reg(LCR);
    uart_write_reg(LCR, lcr | (1 << 7));

    uint16 latch = CLK_FREQ / (baudrate * 16);
    uart_write_reg(DLL, latch & 0xff);
    uart_write_reg(DLM, (latch >> 8) & 0xff);

    // set UART as 8N1(word length 8bit, no parity check, 1 stop bit)
    lcr = 0;
    uart_write_reg(LCR, lcr | (3 << 0));

    // enable RX interrupts
    uart_write_reg(IER, IER_RHR);
}

int uart_getchar(void) {
    if (uart_read_reg(LSR) & LSR_RX_READY) {
        return uart_read_reg(RHR);
    }
    else {
        return -1;
    }
}

void uart_putchar(char c) {
    // wait until THR is ready
    while (0 == (uart_read_reg(LSR) & LSR_TX_IDLE))
        ;
    uart_write_reg(THR, c);
}
