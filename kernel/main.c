// Copyright (c) 2006-2019 Frans Kaashoek, Robert Morris, Russ Cox,
//                         Massachusetts Institute of Technology

#define __module_name__ 	"main"

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "hal/riscv.h"
#include "hal/uart.h"
#include "sbi.h"
#include "console.h"
#include "printf.h"
#include "timer.h"
#include "trap.h"
#include "sched/proc.h"
#include "hal/plic.h"
#include "mm/pm.h"
#include "mm/vm.h"
#include "mm/kmalloc.h"
#include "hal/disk.h"
#include "fs/buf.h"
#include "utils/debug.h"

#ifndef QEMU
#include "hal/fpioa.h"
#include "hal/dmac.h"
#endif

// The entry for non-boot harts. Defined in entry.S.
void _entry_nonboot(void);

static inline void inithartid(unsigned long hartid) {
    asm volatile("mv tp, %0" : : "r" (hartid));
}

// boot other non-boot harts
void hart_boot(uint64 hartid, uint64 opaque) {
    inithartid(hartid);
    floatinithart();
    kvminithart();
    plicinithart(); // ask PLIC for device interrupts
    trapinithart();

    __sync_synchronize();

    __debug_info("main", "hart %d boots successfully!\n", hartid);
    scheduler();
}

void main(uint64 hartid, uint64 dtb_pa) {
    inithartid(hartid);
    cpuinit();          // init cpu structures
    floatinithart();    // init floatpoint
    consoleinit();      // init console
    printfinit();       // init a lock for printf
    print_logo();       // display kernel logo to test printf

    // memory management
    kpminit();          // physical page allocator
    kvminit();          // create kernel page table
    kvminithart();      // turn on paging
    kmallocinit();      // small physical memory allocator

    // init devices
    uart_init(115200);
    plicinit();
    plicinithart();
    trapinithart();     // install kernel trap vector

    // process and fs
    procinit();
    disk_init();        // initialize disk interface
    binit();            // buffer cache
    userinit();         // first user process

    // wake up other harts
    for (int i = 0; i < NCPU; i ++) {
        if (i != hartid) {
            struct sbiret res = sbi_hart_start(i, (uint64)_entry_nonboot, 0);
            __debug_assert("main", SBI_SUCCESS == res.error, "sbi_hart_start failed!");
        }
    }
    __sync_synchronize();

    // into scheduler
    __debug_info("main", "hart %d boots successfully!\n", hartid);
    scheduler();
}
