#ifndef __DEBUG_syscall 
#undef DEBUG 
#endif 

#define __module_name__ 	"syscall"

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "hal/riscv.h"
#include "sync/spinlock.h"
#include "sched/proc.h"
#include "syscall.h"
#include "sysinfo.h"
#include "mm/pm.h"
#include "mm/vm.h"
#include "mm/usrmm.h"
#include "utils/string.h"
#include "printf.h"
#include "utils/debug.h"

// Fetch the uint64 at addr from the current process.
int
fetchaddr(uint64 addr, uint64 *ip)
{
	// struct proc *p = myproc();

	// if (partofseg(p->segment, addr, addr + sizeof(uint64)) == NULL)
	//   return -1;
	// if(copyin(p->pagetable, (char *)ip, addr, sizeof(*ip)) != 0)
	if(copyin2((char *)ip, addr, sizeof(*ip)) != 0)
		return -1;
	return 0;
}

// Fetch the nul-terminated string at addr from the current process.
// Returns length of string, not including nul, or -1 for error.
int
fetchstr(uint64 addr, char *buf, int max)
{
	// struct proc *p = myproc();
	// int err = copyinstr(p->pagetable, buf, addr, max);
	int ret = copyinstr2(buf, addr, max);
	return ret;
	// if(err < 0)
	//   return err;
	// return strlen(buf);
}

static uint64
argraw(int n)
{
	struct proc *p = myproc();
	switch (n) {
	case 0:
		return p->trapframe->a0;
	case 1:
		return p->trapframe->a1;
	case 2:
		return p->trapframe->a2;
	case 3:
		return p->trapframe->a3;
	case 4:
		return p->trapframe->a4;
	case 5:
		return p->trapframe->a5;
	}
	panic("argraw");
	return -1;
}

// Fetch the nth 32-bit system call argument.
int
argint(int n, int *ip)
{
	*ip = argraw(n);
	struct proc *p = myproc();
	if (p->tmask/* & (1 << (p->trapframe->a7 - 1))*/) {
		if (n != 0) {
			printf(", ");
		}
		printf("%d", *ip);
	}
	return 0;
}

// Retrieve an argument as a pointer.
// Doesn't check for legality, since
// copyin/copyout will do that.
int
argaddr(int n, uint64 *ip)
{
	*ip = argraw(n);
	struct proc *p = myproc();
	if (p->tmask/* & (1 << (p->trapframe->a7 - 1))*/) {
			if (n != 0) {
			printf(", ");
		}
		printf("0x%x", *ip);
	}
	return 0;
}

// Fetch the nth word-sized system call argument as a null-terminated string.
// Copies into buf, at most max.
// Returns string length if OK (including nul), -1 if error.
int
argstr(int n, char *buf, int max)
{
	uint64 addr;
	if(argaddr(n, &addr) < 0)
		return -1;
	int ret = fetchstr(addr, buf, max);
	struct proc *p = myproc();
	if (ret >= 0 && (p->tmask/* & (1 << (p->trapframe->a7 - 1))*/)) {
		printf("=\"%s\"", buf);
	}
	return ret;
}

extern uint64 sys_chdir(void);
extern uint64 sys_close(void);
extern uint64 sys_dup(void);
extern uint64 sys_dup3(void);
extern uint64 sys_exec(void);
extern uint64 sys_exit(void);
extern uint64 sys_fork(void);
extern uint64 sys_fstat(void);
extern uint64 sys_getpid(void);
extern uint64 sys_kill(void);
extern uint64 sys_mkdirat(void);
extern uint64 sys_openat(void);
extern uint64 sys_pipe(void);
extern uint64 sys_read(void);
extern uint64 sys_brk(void);
extern uint64 sys_sbrk(void);
extern uint64 sys_sleep(void);
extern uint64 sys_wait(void);
extern uint64 sys_write(void);
extern uint64 sys_uptime(void);
extern uint64 sys_test_proc(void);
extern uint64 sys_getdents(void);
extern uint64 sys_getcwd(void);
extern uint64 sys_unlinkat(void);
extern uint64 sys_trace(void);
extern uint64 sys_sysinfo(void);
extern uint64 sys_renameat(void);
extern uint64 sys_execve(void);
extern uint64 sys_mount(void);
extern uint64 sys_umount(void);
extern uint64 sys_clone(void);
extern uint64 sys_getppid(void);
extern uint64 sys_wait4(void);
extern uint64 sys_times(void);
extern uint64 sys_uname(void);
extern uint64 sys_sched_yield(void);
extern uint64 sys_gettimeofday(void);
extern uint64 sys_nanosleep(void);
extern uint64 sys_mmap(void);
extern uint64 sys_munmap(void);
extern uint64 sys_getuid(void);
extern uint64 sys_geteuid(void);
extern uint64 sys_getgid(void);
extern uint64 sys_getegid(void);
extern uint64 sys_readv(void);
extern uint64 sys_writev(void);
extern uint64 sys_readlinkat(void);
extern uint64 sys_mprotect(void);
extern uint64 sys_rt_sigaction(void);
extern uint64 sys_rt_sigprocmask(void);
extern uint64 sys_rt_sigreturn(void);
extern uint64 sys_fstatat(void);
extern uint64 sys_fcntl(void);
extern uint64 sys_pselect(void);
extern uint64 sys_ppoll(void);
extern uint64 sys_faccessat(void);
extern uint64 sys_lseek(void);
extern uint64 sys_utimensat(void);
extern uint64 sys_ioctl(void);
extern uint64 sys_prlimit64(void);
extern uint64 sys_adjtimex(void);
extern uint64 sys_clock_settime(void);
extern uint64 sys_clock_gettime(void);
extern uint64 sys_statfs(void);
extern uint64 sys_getrusage(void);
extern uint64 sys_setitimer(void);
extern uint64 sys_sync(void);

static uint64 (*syscalls[])(void) = {
	[SYS_fork]			sys_fork,
	[SYS_exit]			sys_exit,
	[SYS_wait]			sys_wait,
	[SYS_pipe2]			sys_pipe,
	[SYS_read]			sys_read,
	[SYS_kill]			sys_kill,
	[SYS_exec]			sys_exec,
	[SYS_fstat]			sys_fstat,
	[SYS_chdir]			sys_chdir,
	[SYS_dup]			sys_dup,
	[SYS_dup3]			sys_dup3,
	[SYS_getpid]		sys_getpid,
	[SYS_brk]			sys_brk,
	[SYS_sbrk]			sys_sbrk,
	[SYS_sleep]			sys_sleep,
	[SYS_uptime]		sys_uptime,
	[SYS_openat]		sys_openat,
	[SYS_write]			sys_write,
	[SYS_mkdirat]		sys_mkdirat,
	[SYS_close]			sys_close,
	[SYS_test_proc]		sys_test_proc,
	[SYS_getdents]		sys_getdents,
	[SYS_getcwd]		sys_getcwd,
	[SYS_unlinkat]		sys_unlinkat,
	[SYS_trace]			sys_trace,
	[SYS_sysinfo]		sys_sysinfo,
	[SYS_renameat2]		sys_renameat,
	[SYS_execve]		sys_execve,
	[SYS_mount]			sys_mount,
	[SYS_umount]		sys_umount,
	[SYS_clone]			sys_clone,
	[SYS_wait4]			sys_wait4,
	[SYS_getpid]		sys_getpid,
	[SYS_getppid]		sys_getppid,
	[SYS_times]			sys_times,
	[SYS_uname]			sys_uname,
	[SYS_sched_yield]	sys_sched_yield,
	[SYS_rt_sigaction]	sys_rt_sigaction, 
	[SYS_rt_sigprocmask] sys_rt_sigprocmask, 
	[SYS_rt_sigreturn]	sys_rt_sigreturn, 
	[SYS_gettimeofday]	sys_gettimeofday,
	[SYS_nanosleep]		sys_nanosleep,
	[SYS_mmap]			sys_mmap,
	[SYS_munmap]		sys_munmap,
	[SYS_getuid]		sys_getuid,
	[SYS_geteuid]		sys_getuid,
	[SYS_getgid]		sys_getuid,
	[SYS_getegid]		sys_getuid,
	[SYS_readv]			sys_readv,
	[SYS_writev]		sys_writev,
	[SYS_readlinkat]	sys_readlinkat,
	[SYS_mprotect]		sys_mprotect,
	[SYS_exit_group]	sys_exit,
	[SYS_ioctl]			sys_ioctl,
	[SYS_fstatat]		sys_fstatat,
	[SYS_fcntl]			sys_fcntl,
	[SYS_pselect6]		sys_pselect,
	[SYS_ppoll]			sys_ppoll,
	[SYS_faccessat]		sys_faccessat,
	[SYS_lseek]			sys_lseek,
	[SYS_utimensat]		sys_utimensat,
	[SYS_prlimit64]		sys_prlimit64, 
	[SYS_adjtimex]		sys_adjtimex, 
	[SYS_clock_settime] sys_clock_settime, 
	[SYS_clock_gettime] sys_clock_gettime, 
	[SYS_statfs]		sys_statfs,
	[SYS_getrusage]		sys_getrusage,
	[SYS_setitimer]		sys_setitimer,
	[SYS_sync]			sys_sync,
};

static char *sysnames[] = {
	[SYS_fork]			"fork",
	[SYS_exit]			"exit",
	[SYS_wait]			"wait",
	[SYS_pipe2]			"pipe2",
	[SYS_read]			"read",
	[SYS_kill]			"kill",
	[SYS_exec]			"exec",
	[SYS_fstat]			"fstat",
	[SYS_chdir]			"chdir",
	[SYS_dup]			"dup",
	[SYS_dup3]			"dup3",
	[SYS_getpid]		"getpid",
	[SYS_brk]			"brk",
	[SYS_sbrk]			"sbrk",
	[SYS_sleep]			"sleep",
	[SYS_uptime]		"uptime",
	[SYS_openat]		"openat",
	[SYS_write]			"write",
	[SYS_mkdirat]		"mkdirat",
	[SYS_close]			"close",
	[SYS_test_proc]		"test_proc",
	[SYS_getdents]		"getdents",
	[SYS_getcwd]		"getcwd",
	[SYS_unlinkat]		"unlinkat",
	[SYS_trace]			"trace",
	[SYS_sysinfo]		"sysinfo",
	[SYS_renameat2]		"renameat2",
	[SYS_execve]		"execve",
	[SYS_mount]			"mount",
	[SYS_umount]		"umount",
	[SYS_clone]			"clone",
	[SYS_wait4]			"wait4",
	[SYS_getpid]		"getpid",
	[SYS_getppid]		"getppid",
	[SYS_times]			"times",
	[SYS_uname]			"uname",
	[SYS_sched_yield]	"sched_yield",
	[SYS_rt_sigaction]	"rt_sigaction", 
	[SYS_rt_sigprocmask] 	"rt_sigprocmask", 
	[SYS_rt_sigreturn]	"rt_sigreturn", 
	[SYS_gettimeofday]	"gettimeofday",
	[SYS_nanosleep]		"nanosleep",
	[SYS_mmap]			"mmap",
	[SYS_munmap]		"munmap",
	[SYS_getuid]		"getuid",
	[SYS_geteuid]		"geteuid",
	[SYS_getgid]		"getgid",
	[SYS_getegid]		"getegid",
	[SYS_readv]			"readv",
	[SYS_writev]		"writev",
	[SYS_readlinkat]	"readlinkat",
	[SYS_mprotect]		"mprotect",
	[SYS_exit_group]	"exit_group",
	[SYS_ioctl]			"ioctl",
	[SYS_fstatat]		"fstatat",
	[SYS_fcntl]			"fcntl",
	[SYS_pselect6]		"pselect",
	[SYS_ppoll]			"ppoll",
	[SYS_faccessat]		"faccessat",
	[SYS_lseek]			"lseek",
	[SYS_utimensat]		"utimensat",
	[SYS_prlimit64]		"prlimit64", 
	[SYS_adjtimex]		"adjtimex", 
	[SYS_clock_gettime]	"clock_gettime", 
	[SYS_clock_settime]	"clock_settime", 
	[SYS_statfs]		"statfs",
	[SYS_getrusage]		"getrusage",
	[SYS_setitimer]		"setitimer",
	[SYS_sync]			"sync",
};

void
syscall(void)
{
	int num;
	struct proc *p = myproc();

	__debug_info("syscall", "enter syscall\n");
	__debug_assert("syscall", p != NULL, "p == NULL\n");
	num = p->trapframe->a7;
	__debug_info("syscall", "num = %d\n", num);
	if(num > 0 && num < NELEM(syscalls) && syscalls[num]) {
		// trace
		int trace = p->tmask;// & (1 << (num - 1));
		if (trace) {
			printf("pid %d: %s(", p->pid, sysnames[num]);
		}
		p->trapframe->a0 = syscalls[num]();
		if (trace) {
			printf(") -> %d\n", p->trapframe->a0);
		}
		
	} else {
		printf(__ERROR("pid %d %s: unknown syscall %d")"\n", p->pid, p->name, num);
		p->trapframe->a0 = -1;
	}
}

uint64 
sys_test_proc(void) {
	int n;
	argint(0, &n);
	printf("hello world from proc %d, hart %d, arg %d\n", myproc()->pid, r_tp(), n);
	return 0;
}

#include "fs/buf.h"
#include "timer.h"

uint64
sys_sysinfo(void)
{
	uint64 addr;
	// struct proc *p = myproc();

	if (argaddr(0, &addr) < 0) {
		return -1;
	}

	struct sysinfo info;
	memset(&info, 0, sizeof(info));

	info.uptime = readtime() / CLK_FREQ;
	info.totalram = PHYSTOP - RUSTSBI_BASE;
	info.freeram = idlepages() << PGSHIFT;
	info.bufferram = BSIZE * BNUM;
	info.procs = procnum();
	info.mem_unit = PGSIZE;

	// if (copyout(p->pagetable, addr, (char *)&info, sizeof(info)) < 0) {
	if (copyout2(addr, (char *)&info, sizeof(info)) < 0) {
		return -1;
	}

	return 0;
}
