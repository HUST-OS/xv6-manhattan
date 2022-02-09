// An binary interface for SBI calls. For more detailed information about SBI, please
// refer to https://github.com/riscv-non-isa/riscv-sbi-doc

#ifndef __RISCV_SBI_H
#define __RISCV_SBI_H

#include "types.h"
#include "hal/riscv.h"

#define LEGACY_SBI_CALL(eid, arg0) ({ \
    register uintptr_t a0 asm ("a0") = (uintptr_t)(arg0); \
    register uintptr_t a7 asm ("a7") = (uintptr_t)(eid); \
    asm volatile ("ecall" \
              : "+r" (a0) \
              : "r" (a7) \
              : "memory"); \
    a0; \
})

// Legacy Extensions
// Only part of these extensions are used
#define SBI_SET_TIMER           0
#define SBI_CONSOLE_PUTCHAR     1
#define SBI_CONSOLE_GETCHAR     2
#define SBI_SHUTDOWN            8

static inline void sbi_console_putchar(int ch) {
    LEGACY_SBI_CALL(SBI_CONSOLE_PUTCHAR, ch);
}

static inline int sbi_console_getchar(void) {
    return LEGACY_SBI_CALL(SBI_CONSOLE_GETCHAR, 0);
}

static inline void sbi_shutdown(void) {
    LEGACY_SBI_CALL(SBI_SHUTDOWN, 0);
}

// As SSIP can now be cleared by software in Supervisor,
// it's meaningless to trap into SBI
static inline void sbi_clear_ipi(void) {
    uint64 sip = r_sip();
    sip = sip & (~SIP_SSIP);
    w_sip(sip);
}

#define SBI_SUCCESS                 0
#define SBI_ERR_FAILED              -1
#define SBI_ERR_NOT_SUPPORTED       -2
#define SBI_ERR_INVALID_PARAM       -3
#define SBI_ERR_DENIED              -4
#define SBI_ERR_INVALID_ADDRESS     -5
#define SBI_ERR_ALREADY_AVAILABLE   -6
#define SBI_ERR_ALREADY_STARTED     -7
#define SBI_ERR_ALREADY_STOPPED     -8

struct sbiret {
    long error;
    long value;
};

#define SBI_CALL(eid, fid, arg0, arg1, arg2, arg3) ({ \
    register uintptr_t a0 asm ("a0") = (uintptr_t)(arg0); \
    register uintptr_t a1 asm ("a1") = (uintptr_t)(arg1); \
    register uintptr_t a2 asm ("a2") = (uintptr_t)(arg2); \
    register uintptr_t a3 asm ("a3") = (uintptr_t)(arg3); \
    register uintptr_t a6 asm ("a6") = (uintptr_t)(fid); \
    register uintptr_t a7 asm ("a7") = (uintptr_t)(eid); \
    asm volatile ("ecall" \
            : "+r" (a0), "+r" (a1) \
            : "r" (a2), "r" (a3), "r" (a6), "r" (a7) \
            : "memory"); \
    (struct sbiret){.error = (long)a0, .value = (long)a1}; \
})

#define SBI_CALL_0(eid, fid) \
    SBI_CALL(eid, fid, 0, 0, 0, 0)
#define SBI_CALL_1(eid, fid, arg0) \
    SBI_CALL(eid, fid, arg0, 0, 0, 0)
#define SBI_CALL_2(eid, fid, arg0, arg1) \
    SBI_CALL(eid, fid, arg0, arg1, 0, 0)
#define SBI_CALL_3(eid, fid, arg0, arg1, arg2) \
    SBI_CALL(eid, fid, arg0, arg1, arg2, 0)
#define SBI_CALL_4(eid, fid, arg0, arg1, arg2, arg3) \
    SBI_CALL(eid, fid, arg0, arg1, arg2, arg3)

// Time Extension

#define TIME_EID        0x54494d45
#define TIME_SET_TIMER  0

static inline struct sbiret sbi_set_timer(uint64 stime_value) {
    return SBI_CALL_1(TIME_EID, TIME_SET_TIMER, stime_value);
}

// sPI Extension

#define IPI_EID         0x735049
#define IPI_SEND_IPI    0

static inline struct sbiret sbi_send_ipi(
    unsigned long hart_mask,
    unsigned long hart_mask_base
) {
    return SBI_CALL_2(IPI_EID, IPI_SEND_IPI, hart_mask, hart_mask_base);
}

// HSM Extension

#define HSM_EID                 0x48534d
#define HSM_HART_START          0
#define HSM_HART_STOP           1
#define HSM_HART_GET_STATUS     2
#define HSM_HART_SUSPEND        3

// Request the SBI implementation to start executing the target hart in supervisor-mode
// at address specified by start_addr param.
// This call is ASYNCHRONOUS. sbi_hart_start() may return before the target hart starts.
static inline struct sbiret sbi_hart_start(
    uint64 hartid,
    uint64 start_addr,
    uint64 opaque       // a XLEN-bit value to be set in a1 register when the hart starts
) {
    return SBI_CALL_3(HSM_EID, HSM_HART_START, hartid, start_addr, opaque);
}

// Request the SBI implementation to stop executing the calling hart in supervisor-mode and
// return it's ownership to the SBI implementation.
static inline struct sbiret sbi_hart_stop(void) {
    return SBI_CALL_0(HSM_EID, HSM_HART_STOP);
}

// Get the current status (or HSM state id) of the given hart in sbiret.value.
#define HART_STARTED            0
#define HART_STOPPED            1
#define HART_START_PENDING      2
#define HART_STOP_PENDING       3
#define HART_SUSPENDED          4
#define HART_SUSPEND_PENDING    5
#define HART_RESUME_PENDING     6
static inline struct sbiret sbi_hart_get_status(uint64 hartid) {
    return SBI_CALL_1(HSM_EID, HSM_HART_GET_STATUS, hartid);
}

// Request the SBI implementation to put the calling hart in a platform specific suspend
// (or low power) state specified by the suspend_type parameter. The hart will automatically
// come out of suspended state and resume normal execution when it receives an INTERRUPT or
// platform specific HARTWARE EVENT.
#define HART_SUSPEND_RETENTIVE      0x00000000
#define HART_SUSPEND_NON_RETENTIVE  0x80000000
static inline struct sbiret sbi_hart_suspend(
    uint32 suspend_type,
    uint64 resume_addr,
    uint64 opaque
) {
    return SBI_CALL_3(HSM_EID, HSM_HART_SUSPEND, suspend_type, resume_addr, opaque);
}

// xv6-k210 specific SBI interface
#define XV6_EID             0x0a000210
#define XV6_SET_EXT         0x00
#define XV6_CONSOLE_PUTS    0x10
#define XV6_GET_TIMER       0x20

static inline struct sbiret sbi_xv6_set_ext(void) {
    return SBI_CALL_0(XV6_EID, XV6_SET_EXT);
}

static inline struct sbiret sbi_xv6_console_puts(char *str, int n) {
    return SBI_CALL_2(XV6_EID, XV6_CONSOLE_PUTS, str, n);
}

static inline struct sbiret sbi_xv6_get_timer(void) {
    return SBI_CALL_0(XV6_EID, XV6_GET_TIMER);
}

#endif