//
// formatted console output -- printf, panic.
//

#include <stdarg.h>

#include "types.h"
#include "param.h"
#include "hal/riscv.h"
#include "sync/spinlock.h"
#include "console.h"
#include "printf.h"
#include "utils/debug.h"

volatile int panicked = 0;

static char digits[] = "0123456789abcdef";

// lock to avoid interleaving concurrent printf's.
static struct {
	struct spinlock lock;
	int locking;
} pr;

void printstring(const char* s) {
	while (*s)
	{
		consputc(*s++);
	}
}

static void
printint(int xx, int base, int sign)
{
	char buf[16];
	int i;
	uint x;

	if(sign && (sign = xx < 0))
		x = -xx;
	else
		x = xx;

	i = 0;
	do {
		buf[i++] = digits[x % base];
	} while((x /= base) != 0);

	if(sign)
		buf[i++] = '-';

	while(--i >= 0)
		consputc(buf[i]);
}


static void
printptr(uint64 x)
{
	int i;
	consputc('0');
	consputc('x');
	for (i = 0; i < (sizeof(uint64) * 2); i++, x <<= 4)
		consputc(digits[x >> (sizeof(uint64) * 8 - 4)]);
}

// Print to the console. only understands %d, %x, %p, %s.
void
printf(char *fmt, ...)
{
	va_list ap;
	int i, c;
	int locking;
	char *s;

	locking = pr.locking;
	if(locking)
		acquire(&pr.lock);
	
	if (fmt == 0)
		panic("null fmt");

	va_start(ap, fmt);
	for(i = 0; (c = fmt[i] & 0xff) != 0; i++){
		if(c != '%'){
			consputc(c);
			continue;
		}
		c = fmt[++i] & 0xff;
		if(c == 0)
			break;
		switch(c){
		case 'd':
			printint(va_arg(ap, int), 10, 1);
			break;
		case 'x':
			printint(va_arg(ap, int), 16, 1);
			break;
		case 'p':
			printptr(va_arg(ap, uint64));
			break;
		case 's':
			if((s = va_arg(ap, char*)) == 0)
				s = "(null)";
			for(; *s; s++)
				consputc(*s);
			break;
		case '%':
			consputc('%');
			break;
		default:
			// Print unknown % sequence to draw attention.
			consputc('%');
			consputc(c);
			break;
		}
	}
	if(locking)
		release(&pr.lock);
}

void
__panic(char *s)
{
	printf(__ERROR("panic")": ");
	printf(s);
	printf("\n");
	backtrace();
	panicked = 1; // freeze uart output from other CPUs
	intr_off();
	for(;;)
		;
}

void backtrace()
{
	uint64 *fp = (uint64 *)r_fp();
	uint64 *bottom = (uint64 *)PGROUNDUP((uint64)fp);
	printf("backtrace:\n");
	while (fp < bottom) {
		uint64 ra = *(fp - 1);
		printf("%p\n", ra - 4);
		fp = (uint64 *)*(fp - 2);
	}
}

void
printfinit(void)
{
	initlock(&pr.lock, "pr");
	pr.locking = 1;   // changed, used to be 1
}