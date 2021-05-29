// Process Management 

#ifndef __DEBUG_proc 
#undef DEBUG 
#endif 

#define __module_name__ 	"proc"

#include "include/types.h"
#include "include/param.h"
#include "include/memlayout.h"
#include "include/riscv.h"
#include "include/spinlock.h"
#include "include/intr.h"
#include "include/pm.h"
#include "include/printf.h"
#include "include/string.h"
#include "include/fs.h"
#include "include/file.h"
#include "include/trap.h"
#include "include/vm.h"
#include "include/kmalloc.h"
#include "include/usrmm.h"
#include "include/debug.h"

#include "include/proc.h"

void swtch(struct context*, struct context*);

void forkret(void);

// pid management 
int next_pid;
struct spinlock pid_lock;
static int allocpid(void) {
	int ret;
	acquire(&pid_lock);
	ret = next_pid;
	next_pid = next_pid + 1;
	release(&pid_lock);

	return ret;
}

struct cpu cpus[NCPU];
int cpuid(void) {
	int id = r_tp();
	return id;
}
struct cpu *mycpu(void) {
	int id = cpuid();
	struct cpu *c = &cpus[id];
	return c;
}
struct proc *myproc(void) {
	push_off();
	struct cpu *c = mycpu();
	struct proc *p = c->proc;
	pop_off();
	return p;
}

// lists for scheduling 

#define TIMER_IRQ 			5
#define TIMER_NORMAL 		10 

#define PRIORITY_TIMEOUT 	0
#define PRIORITY_IRQ 		1
#define PRIORITY_NORMAL 	2
#define PRIORITY_NUMBER 	3 
struct proc *proc_runnable[PRIORITY_NUMBER];
struct proc *proc_sleep;
struct proc *proc_zombie;
struct spinlock proc_lock;

#define __enter_proc_cs \
	acquire(&proc_lock);
#define __leave_proc_cs \
	release(&proc_lock);

// before modifying proc_list, lock should be acquired 
static void proc_list_insert_no_lock(struct proc **head, struct proc *p) {
	__debug_assert("proc_list_insert", NULL != p, "insert NULL into list\n");

	struct proc *tmp = *head;
	while (NULL != tmp) {
		head = &(tmp->next);
		tmp = tmp->next;
	}
	*head = p;
	p->prev = head;
	p->next = NULL;
}
static void proc_list_remove_no_lock(struct proc *p) {
	__debug_assert("proc_list_remove", NULL != p, "remove NULL from list\n");

	*(p->prev) = p->next;
	if (NULL != p->next)
		p->next->prev = p->prev;
	p->prev = NULL;
	p->next = NULL;
}

#define __insert_runnable(priority, p) do {\
	(p)->state = RUNNABLE; \
	proc_list_insert_no_lock(&(proc_runnable[priority]), p); \
} while (0)
#define __insert_sleep(p) do {\
	(p)->state = SLEEPING; \
	proc_list_insert_no_lock(&proc_sleep, p); \
} while (0)
#define __insert_zombie(p) do {\
	(p)->state = ZOMBIE; \
	proc_list_insert_no_lock(&proc_zombie, p); \
} while (0)
#define __remove(p) \
	proc_list_remove_no_lock(p) 

static void freeproc(struct proc *p) {
	// file system fields were freed in exit() 

	// free pagetable 
	if (p->pagetable) 
		proc_freepagetable(p->pagetable, p->segment);

	// free trapframe 
	freepage(p->trapframe);

	// parenting is handled in exit() 

	// free the proc itself 
	kfree(p);
}

extern char trampoline[];	// trampoline.S

// Create a user page table for a given process,
// with no user memory, but with trampoline pages.
pagetable_t
proc_pagetable(struct proc *p)
{
  pagetable_t pagetable;

  // An empty page table.
  pagetable = kvmcreate();
  if(pagetable == 0)
	return NULL;

  // map the trapframe just below TRAMPOLINE, for trampoline.S.
  if(mappages(pagetable, TRAPFRAME, PGSIZE,
			(uint64)(p->trapframe), PTE_R | PTE_W) < 0){
	kvmfree(pagetable, 1);
	return NULL;
  }

  return pagetable;
}

// Free a process's page table, and free the
// physical memory it refers to.
void
proc_freepagetable(pagetable_t pagetable, struct seg *head)
{
	// __debug_info("proc_freepagetable", "enter\n");
  unmappages(pagetable, TRAPFRAME, 1, 0);
//   __debug_info("proc_freepagetable", "leave\n");
  delsegs(pagetable, head);
//   __debug_info("proc_freepagetable", "leave 2\n");
  uvmfree(pagetable);
  kvmfree(pagetable, 1);
}


// alloc a proc ready for running 
static struct proc *allocproc(void) {
	struct proc *p = kmalloc(sizeof(struct proc));

	if (NULL == p) {
		__debug_warn("allocproc", "fail to kmalloc() proc\n");
		return NULL;
	}

	// initlock 
	initlock(&p->lk, "p->lk");

	p->chan = NULL;
	p->killed = 0;
	p->pid = allocpid();

	p->child = NULL;

	p->proc_tms.utime = 0;
	p->proc_tms.stime = 0;
	p->proc_tms.cutime = 0;
	p->proc_tms.cstime = 0;

	// allocate a page trapframe 
	p->trapframe = (struct trapframe*)allocpage();
	if (NULL == p->trapframe) {
		__debug_warn("allocproc", "fail to alloc trapframe\n");
		kfree(p);
		return NULL;
	}

	// pagetable 
	p->segment = NULL;
	if (NULL == (p->pagetable = proc_pagetable(p))) {
		__debug_warn("allocproc", "fail to set pagetable\n");
		freeproc(p);
		return NULL;
	}

	// init fds 
	memset(&p->fds, 0, sizeof(p->fds));

	// user kernel stack 
	p->kstack = VKSTACK;

	p->context.ra = (uint64)forkret;
	p->context.sp = p->kstack + PGSIZE;

	return p;
}

int fork_cow(void) {
	struct proc *p = myproc();
	struct proc *np;

	// allocate proc 
	np = allocproc();
	if (NULL == np) {
		__debug_warn("fork", "fail to allocate new proc\n");
		return -1;
	}

	// copy parent's memory layout 
	np->segment = copysegs(p->pagetable, p->segment, np->pagetable);
	if (np->segment == NULL) {
		__debug_warn("fork", "fail to copy segments\n");
		freeproc(np);
		return -1;
	}
	// struct seg *seg;
	// struct seg **s2 = &(np->segment);
	// for (seg = p->segment; seg != NULL; seg = seg->next) {
	// 	struct seg *s = kmalloc(sizeof(struct seg));
	// 	if (s == NULL) {
	// 		__debug_warn("fork_cow", "seg kmalloc fail\n");
	// 		freeproc(np);
	// 		return -1;
	// 	}
	// 	// Copy user memory from parent to child.
	// 	if (uvmcopy_cow(p->pagetable, np->pagetable, 
	// 			seg->addr, seg->addr + seg->sz, seg->type) < 0) 
	// 	{
	// 		__debug_warn("fork_cow", "uvmcopy_cow fails\n");
	// 		__debug_warn("fork_cow", "p %s\n", p->name);
	// 		__debug_warn("fork_cow", "%p, %p, %d\n", 
	// 				seg->addr, seg->addr + seg->sz, seg->type);
	// 		kfree(s);
	// 		__debug_warn("fork_cow", "exit kfree\n");
	// 		freeproc(np);
	// 		return -1;
	// 	}
	// 	memmove(s, seg, sizeof(struct seg));
	// 	s->next = NULL;
	// 	*s2 = s;
	// 	s2 = &s->next;
	// }

	// filesystem 
	if (copyfdtable(&p->fds, &np->fds) < 0) {
		freeproc(np);
		return -1;
	}
	np->cwd = idup(p->cwd);

	// copy parent's trapframe 
	*(np->trapframe) = *(p->trapframe);
	np->trapframe->a0 = 0;

	// parenting 
	acquire(&p->lk);
	__enter_proc_cs
	np->parent = p;
	np->sibling_prev = &(p->child);
	np->sibling_next = p->child;
	if (NULL != p->child) {
		p->child->sibling_prev = &(np->sibling_next);
	}
	p->child = np;
	release(&p->lk);
	// __debug_info("fork_cow", "%d check children: ", p->pid);
	// for (struct proc *tmp = p->child; tmp != NULL; tmp = tmp->sibling_next) {
	// 	printf("%d ", tmp->pid);
	// }
	// printf("\n");
	__leave_proc_cs

    //__debug_assert("fork_cow", p->pid < np->pid, "child pid bigger\n");

	// copy debug info 
	safestrcpy(np->name, p->name, sizeof(p->name));
	np->tmask = p->tmask;

	int pid = np->pid;

	__enter_proc_cs 
	// init timer 
	np->timer = TIMER_NORMAL;
	// set runnable 
	__insert_runnable(PRIORITY_NORMAL, np);
	__leave_proc_cs 

	__debug_info("fork_cow", "leave %d -> %d\n", p->pid, pid);

	return pid;
}

// similiar to fork, but different 
// a lot can be improved later 
int clone(uint64 flag, uint64 stack) {
	// currently don't deal with flag 
	if (NULL == stack) 
		return fork_cow();

	struct proc *p = myproc();
	struct proc *np;

	// allocate proc 
	np = allocproc();
	if (NULL == np) {
		__debug_warn("fork", "fail to allocate new proc\n");
		return -1;
	}

	// copy parent's memory layout 
	np->segment = copysegs(p->pagetable, p->segment, np->pagetable);
	if (np->segment == NULL) {
		__debug_warn("fork", "fail to copy segments\n");
		freeproc(np);
		return -1;
	}
	// struct seg *seg;
	// struct seg **s2 = &(np->segment);
	// for (seg = p->segment; seg != NULL; seg = seg->next) {
	// 	struct seg *s = kmalloc(sizeof(struct seg));
	// 	if (s == NULL) {
	// 		__debug_warn("fork_cow", "seg kmalloc fail\n");
	// 		freeproc(np);
	// 		return -1;
	// 	}
	// 	// Copy user memory from parent to child.
	// 	if (uvmcopy_cow(p->pagetable, np->pagetable, 
	// 			seg->addr, seg->addr + seg->sz, seg->type) < 0) 
	// 	{
	// 		__debug_warn("fork_cow", "uvmcopy_cow fails\n");
	// 		__debug_warn("fork_cow", "p %s\n", p->name);
	// 		__debug_warn("fork_cow", "%p, %p, %d\n", 
	// 				seg->addr, seg->addr + seg->sz, seg->type);
	// 		kfree(s);
	// 		__debug_warn("fork_cow", "exit kfree\n");
	// 		freeproc(np);
	// 		return -1;
	// 	}
	// 	memmove(s, seg, sizeof(struct seg));
	// 	s->next = NULL;
	// 	*s2 = s;
	// 	s2 = &s->next;
	// }

	// these parts may be improved later 
	// filesystem 
	if (copyfdtable(&p->fds, &np->fds) < 0) {
		freeproc(np);
		return -1;
	}
	np->cwd = idup(p->cwd);
	
	// these parts may be improved later 
	// filesystem 
	if (copyfdtable(&p->fds, &np->fds) < 0) {
		freeproc(np);
		return -1;
	}

	np->cwd = idup(p->cwd);
	// copy parent's trapframe 
	*(np->trapframe) = *(p->trapframe);
	np->trapframe->a0 = 0;
	np->trapframe->sp = stack;

	// parenting 
	acquire(&p->lk);
	np->parent = p;
	np->sibling_prev = &(p->child);
	np->sibling_next = p->child;
	if (NULL != p->child) {
		p->child->sibling_prev = &(np->sibling_next);
	}
	p->child = np;
	release(&p->lk);

    __debug_assert("clone", p->pid < np->pid, "child pid bigger\n");

	// copy debug info 
	safestrcpy(np->name, p->name, sizeof(p->name));
	np->tmask = p->tmask;

	int pid = np->pid;

	__enter_proc_cs 
	// init timer 
	np->timer = TIMER_NORMAL;
	// set runnable 
	__insert_runnable(PRIORITY_NORMAL, np);
	__leave_proc_cs 

	__debug_info("clone", "leave from fork\n");

	return pid;
}

static void wakeup_no_lock(void *chan) {
	struct proc *p = proc_sleep;
	while (NULL != p) {
		struct proc *next = p->next;
		if ((uint64)chan == (uint64)p->chan) {
			__remove(p);
			p->timer = TIMER_IRQ;
			__insert_runnable(PRIORITY_IRQ, p);
		}
		p = next;
	}
}

void wakeup(void *chan) {
	__enter_proc_cs 
	wakeup_no_lock(chan);
	__leave_proc_cs 
}

void exit(int xstate) {
	struct proc *p = myproc();

	extern struct proc *initproc;
	__debug_assert("exit", initproc != p, "init exiting\n");
	__debug_assert("exit", NULL != p, "NULL proc\n");

	__debug_info("exit", "pid %d %s\n", p->pid, p->name);

	// close all open files 
	dropfdtable(&p->fds);
	iput(p->cwd);

	// DO NOT REMOVE ITSELF FROM PARENT'S CHILD LIST!
	// IN CASE THAT PARENT WAS KILLED BEFORE ABLE TO WAIT 

	// write in xstate 
	p->xstate = xstate;

	// re-parent all it's children to `initproc`
	__enter_proc_cs 
	if (NULL != p->child) {
		struct proc *tmp = p->child;
		struct proc **last_prev;

		// re-parent every children to `initproc`
		tmp->parent = initproc;
		while (NULL != tmp->sibling_next) {
			tmp = tmp->sibling_next;
			tmp->parent = initproc;
		}
		last_prev = &(tmp->sibling_next);

		// insert `child` list into initproc 
		p->child->sibling_prev = &(initproc->child);
		*last_prev = initproc->child;
		if (NULL != initproc->child) {
			initproc->child->sibling_prev = last_prev;
		}
		initproc->child = p->child;
	}

	__remove(p);		// remove p from its list 
	__insert_zombie(p);	// re-insert p into `proc_zombie`

	// after insert the proc into proc_zombie, 
	// no interrupts should be allowed 

	// wakeup parent after the proc is inserted into zombie list 
	wakeup_no_lock(p->parent);
	// wakeup initproc means no bad 
	wakeup_no_lock(initproc);

	sched();
	__debug_error("exit", "zombie exit\n");
	panic("panic!\n");
}

#define WAIT_OPTIONS_WNOHANG 		1
#define WAIT_OPTIONS_WUNTRACED 		2
#define WAIT_OPTIONS_WCONTINUED 	8
static inline int is_child_no_lock(int pid, struct proc *np) {
	return (-1 == pid || pid == np->pid) && ZOMBIE == np->state;
}
int wait4(int pid, uint64 status, uint64 options) {
	struct proc *p = myproc();

	__enter_proc_cs 
	if (NULL == p->child) {
		// the child is a lie!
		__leave_proc_cs 
		return -1;
	}
	else while (1) {
		struct proc *np = p->child;
		while (NULL != np) {
			if (is_child_no_lock(pid, np)) {
				// find one! remove it from parent's child list 
				*(np->sibling_prev) = np->sibling_next;
				if (NULL != np->sibling_next) {
					np->sibling_next->sibling_prev = np->sibling_prev;
				}

				// update proc system/user time 
				p->proc_tms.cstime += np->proc_tms.stime + np->proc_tms.cstime;
				p->proc_tms.cutime += np->proc_tms.utime + np->proc_tms.cutime;

				// remove np from proc_zombie 
				__remove(np);

				// copyout xstate 
				if (status && copyout2(status, (char*)&np->xstate, sizeof(np->xstate)) < 0) {
					__debug_error("wait", "fail to copy out xstate\n");
					panic("panic!\n");
				}

				// return child pid 
				int child_pid = np->pid;
				freeproc(np);

				// it's okay to release proc_lock here 
				__leave_proc_cs 

				return child_pid;
			}
			else {
				np = np->sibling_next;
			}
		}
		// no zombie child found
		if (0 == (options & WAIT_OPTIONS_WNOHANG)) {
			// sleep to wait 
			sleep(p, &proc_lock);
		}
		else {
			// return immediately 
			__leave_proc_cs 
			return 0;
		}
	}
}

void proc_tick(void) {
	__enter_proc_cs 

	// runnable 
	for (int i = PRIORITY_IRQ; i < PRIORITY_NUMBER; i ++) {
		struct proc *p = proc_runnable[i];
		while (NULL != p) {
			struct proc *next = p->next;
			if (RUNNING != p->state) {
				p->timer = p->timer - 1;
				if (0 == p->timer) {	// timeout 
					__remove(p);
					__insert_runnable(PRIORITY_TIMEOUT, p);
				}
			}
			else {
				p->proc_tms.utime += 1;
			}
			p->proc_tms.stime += 1;
			p = next;
		}
	} 

	// sleep 
	struct proc *tmp = proc_sleep;
	while (NULL != tmp) {
		tmp->proc_tms.stime += 1;
		tmp = tmp->next;
	}

	__leave_proc_cs 
}

int kill(int pid) {
	struct proc *tmp;

	__enter_proc_cs 

	// search runnable 
	for (int i = 0; i < PRIORITY_NUMBER; i ++) {
		tmp = proc_runnable[i];
		while (NULL != tmp) {
			if (pid == tmp->pid) {
				// found 
				tmp->killed = 1;
				__leave_proc_cs 
				__debug_info("kill", "leave runnable\n");
				return 0;
			}
			else {
				tmp = tmp->next;
			}
		}
	}
	// search sleep 
	tmp = proc_sleep;
	while (NULL != tmp) {
		if (pid == tmp->pid) {
			// found 
			tmp->killed = 1;
			// wakeup the proc 
			__remove(tmp);
			tmp->timer = TIMER_IRQ;
			__insert_runnable(PRIORITY_IRQ, tmp);

			__leave_proc_cs 
			__debug_info("kill", "leave sleep\n");
			return 0;
		}
		else {
			tmp = tmp->next;
		}
	}

	__leave_proc_cs 
	return -1;
}

// turn current proc into `RUNNABLE`
// and switch to scheduler 
void yield(void) {
	struct proc *p = myproc();

	__debug_assert("yield", p != NULL, "p == NULL\n");
	__enter_proc_cs 

	__remove(p);
	p->timer = TIMER_NORMAL;
	__insert_runnable(PRIORITY_NORMAL, p);

	// swtch to scheduler 
	sched();

	__leave_proc_cs 
}

// sleep must be atomic, so lk must be held when going into sleep()
void sleep(void *chan, struct spinlock *lk) {
	struct proc *p = myproc();

	__debug_assert("sleep", NULL != p, "p == NULL\n");
	__debug_assert("sleep", NULL != lk, "lk == NULL\n");

	// __debug_info("sleep", "enter\n");
	// either proc_lock or lk must be held at any time, 
	// so that proc would sleep atomically 
	if (&proc_lock != lk) {
		__enter_proc_cs 
		release(lk);
	}

	p->chan = chan;
	__remove(p);	// remove p from runnable 
	__insert_sleep(p);

	sched();
	p->chan = NULL;

	// release proc_lock first to avoid deadlock 
	// with another call to sleep() with the same lk
	__leave_proc_cs 
	acquire(lk);
}

// get the next runnable proc 
// lock must be ACQUIRED before 
static struct proc *get_runnable_no_lock(void) {
	for (int i = 0; i < PRIORITY_NUMBER; i ++) {
		struct proc *tmp = proc_runnable[i];
		while (NULL != tmp) {
			if (RUNNABLE == tmp->state) {
				return tmp;
			} else {
				tmp = tmp->next;
			}
		}
	}
	return NULL;
}

struct proc const *get_runnable(void) {
	struct proc *ret;
	__enter_proc_cs 
	ret = get_runnable_no_lock();
	__leave_proc_cs 
	return ret;
}

void scheduler(void) {
	struct proc *tmp;
	struct cpu *c = mycpu();

	// it should never returns!
	while (1) {
		intr_on();		// make sure interrupts are available 
		__enter_proc_cs 
		tmp = get_runnable_no_lock();
		if (NULL != tmp) {	// if found one 
			tmp->state = RUNNING;
			c->proc = tmp;

			// switch to user kernel pagetable 
			// will switch to user's pagetable at usertrapret 
			w_satp(MAKE_SATP(tmp->pagetable));
			sfence_vma();
			// swtch context 
			swtch(&c->context, &tmp->context);
			// switch back to kernel pagetable 
			w_satp(MAKE_SATP(kernel_pagetable));
			sfence_vma();
			/*__debug_info("scheduler()", "swtch from %s\n", tmp->name);*/
		}
		c->proc = NULL;
		__leave_proc_cs
	} 

	__debug_error("scheduler", "scheduler returns!\n");
	panic("panic!\n");
}

// sched() is the only way back to scheduler() 
void sched(void) {
	int intena;
	struct proc *p = myproc();

	__debug_assert("sched", NULL != p, "NULL == p\n");
	// __debug_info("sched", "p %s\n", p->name);
	if (!holding(&proc_lock)) 
		panic("sched proc_lock\n");
	if (1 != mycpu()->noff) 
		panic("sched locks\n");
	if (RUNNING == p->state) 
		panic("sched running\n");
	if (intr_get()) 
		panic("sched interruptible\n");

	intena = mycpu()->intena;
	swtch(&p->context, &mycpu()->context);
	mycpu()->intena = intena;
}

void procinit(void) {
	// init pid 
	next_pid = 1;
	initlock(&pid_lock, "pid_lock");

	// init cpus 
	memset(cpus, 0, sizeof(cpus));

	// init list 
	for (int i = 0; i < PRIORITY_NUMBER; i ++) 
		proc_runnable[i] = NULL;
	proc_sleep = NULL;
	proc_zombie = NULL;
	initlock(&proc_lock, "proc_lock");
	__debug_info("procinit", "init\n");
}

// a fork child's very first scheduling by scheduler() 
// will swtch to forkret 
void forkret(void) {
	extern int first;

	__leave_proc_cs 
	if (first) {
		// File system initialization must be run in the context of a 
		// regular process (e.g., because it calls sleep), and thus 
		// cannot be run from main() 
		__debug_info("forkret", "hart%d enter here\n", cpuid());
		first = 0;
		rootfs_init();
		myproc()->cwd = namei("/home");
	}

	usertrapret();
}

// a user program that calls exec("/init")
// od -t xC initcode
uchar initcode[] = {
	0x17, 0x05, 0x00, 0x00, 0x03, 0x35, 0x05, 0x05, 0x97, 0x05, 0x00, 0x00, 0x83, 0xB5, 0x05, 0x05,
	0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00, 0x93, 0x08, 0xD0, 0x05, 0x73, 0x00, 0x00, 0x00,
	0xEF, 0xF0, 0x9F, 0xFF, 0x2E, 0x2F, 0x69, 0x6E, 0x69, 0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/*uchar ostest_code[] = {*/
/*0x93, 0x06, 0x60, 0x1b, 0x13, 0x06, 0x20, 0x00, 0x97, 0x05, 0x00, 0x00, 0x83, 0xb5, 0x85, 0x2e,*/
/*0x13, 0x05, 0xc0, 0xf9, 0x97, 0x00, 0x00, 0x00, 0xe7, 0x80, 0x00, 0x09, 0x13, 0x05, 0x00, 0x00,*/
/*0x97, 0x00, 0x00, 0x00, 0xe7, 0x80, 0x00, 0x09, 0x13, 0x05, 0x00, 0x00, 0x97, 0x00, 0x00, 0x00,*/
/*0xe7, 0x80, 0x40, 0x08, 0x97, 0x04, 0x00, 0x00, 0x93, 0x84, 0x44, 0x1b, 0x17, 0x09, 0x00, 0x00,*/
/*0x13, 0x09, 0xc9, 0x2a, 0x6f, 0x00, 0x80, 0x01, 0x13, 0x05, 0x00, 0x00, 0x97, 0x00, 0x00, 0x00,*/
/*0xe7, 0x80, 0xc0, 0x04, 0x93, 0x84, 0x84, 0x00, 0x63, 0x82, 0x24, 0x03, 0x97, 0x00, 0x00, 0x00,*/
/*0xe7, 0x80, 0x00, 0x03, 0xe3, 0x12, 0x05, 0xfe, 0x93, 0x05, 0x00, 0x00, 0x03, 0xb5, 0x04, 0x00,*/
/*0x97, 0x00, 0x00, 0x00, 0xe7, 0x80, 0x00, 0x01, 0x6f, 0xf0, 0xdf, 0xfd, 0x6f, 0x00, 0x00, 0x00,*/
/*0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00, 0x67, 0x80, 0x00, 0x00, 0x93, 0x08, 0x10, 0x00,*/
/*0x73, 0x00, 0x00, 0x00, 0x67, 0x80, 0x00, 0x00, 0x93, 0x08, 0x30, 0x00, 0x73, 0x00, 0x00, 0x00,*/
/*0x67, 0x80, 0x00, 0x00, 0x93, 0x08, 0x80, 0x03, 0x73, 0x00, 0x00, 0x00, 0x67, 0x80, 0x00, 0x00,*/
/*0x93, 0x08, 0x70, 0x01, 0x73, 0x00, 0x00, 0x00, 0x67, 0x80, 0x00, 0x00, 0x93, 0x08, 0xd0, 0x05,*/
/*0x73, 0x00, 0x00, 0x00, 0x67, 0x80, 0x00, 0x00, 0x2f, 0x64, 0x65, 0x76, 0x2f, 0x63, 0x6f, 0x6e,*/
/*0x73, 0x6f, 0x6c, 0x65, 0x00, 0x00, 0x00, 0x00, 0x62, 0x72, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x63, 0x68, 0x64, 0x69, 0x72, 0x00, 0x00, 0x00, 0x63, 0x6c, 0x6f, 0x6e, 0x65, 0x00, 0x00, 0x00,*/
/*0x63, 0x6c, 0x6f, 0x73, 0x65, 0x00, 0x00, 0x00, 0x64, 0x75, 0x70, 0x32, 0x00, 0x00, 0x00, 0x00,*/
/*0x64, 0x75, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x78, 0x65, 0x63, 0x76, 0x65, 0x00, 0x00,*/
/*0x65, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00, 0x66, 0x6f, 0x72, 0x6b, 0x00, 0x00, 0x00, 0x00,*/
/*0x66, 0x73, 0x74, 0x61, 0x74, 0x00, 0x00, 0x00, 0x67, 0x65, 0x74, 0x63, 0x77, 0x64, 0x00, 0x00,*/
/*0x67, 0x65, 0x74, 0x64, 0x65, 0x6e, 0x74, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x67, 0x65, 0x74, 0x70, 0x69, 0x64, 0x00, 0x00, 0x67, 0x65, 0x74, 0x70, 0x70, 0x69, 0x64, 0x00,*/
/*0x67, 0x65, 0x74, 0x74, 0x69, 0x6d, 0x65, 0x6f, 0x66, 0x64, 0x61, 0x79, 0x00, 0x00, 0x00, 0x00,*/
/*0x6d, 0x6b, 0x64, 0x69, 0x72, 0x5f, 0x00, 0x00, 0x6d, 0x6d, 0x61, 0x70, 0x00, 0x00, 0x00, 0x00,*/
/*0x6d, 0x6f, 0x75, 0x6e, 0x74, 0x00, 0x00, 0x00, 0x6d, 0x75, 0x6e, 0x6d, 0x61, 0x70, 0x00, 0x00,*/
/*0x6f, 0x70, 0x65, 0x6e, 0x61, 0x74, 0x00, 0x00, 0x6f, 0x70, 0x65, 0x6e, 0x00, 0x00, 0x00, 0x00,*/
/*0x70, 0x69, 0x70, 0x65, 0x00, 0x00, 0x00, 0x00, 0x72, 0x65, 0x61, 0x64, 0x00, 0x00, 0x00, 0x00,*/
/*0x74, 0x69, 0x6d, 0x65, 0x73, 0x00, 0x00, 0x00, 0x75, 0x6d, 0x6f, 0x75, 0x6e, 0x74, 0x00, 0x00,*/
/*0x75, 0x6e, 0x61, 0x6d, 0x65, 0x00, 0x00, 0x00, 0x75, 0x6e, 0x6c, 0x69, 0x6e, 0x6b, 0x00, 0x00,*/
/*0x77, 0x61, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00, 0x77, 0x61, 0x69, 0x74, 0x70, 0x69, 0x64, 0x00,*/
/*0x77, 0x72, 0x69, 0x74, 0x65, 0x00, 0x00, 0x00, 0x79, 0x69, 0x65, 0x6c, 0x64, 0x00, 0x00, 0x00,*/
/*0x73, 0x6c, 0x65, 0x65, 0x70, 0x00, 0x00, 0x00, 0xd8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x48, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x60, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x70, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0x90, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0xa0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0xb0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,*/
/*0xd0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    */
/*0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    */
/*0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
/*};*/

int first = 1;
struct proc *initproc = NULL;
// init the first proc `initcode`
// run at kernel init time, and only hart0 should run this 
void userinit(void) {
	struct proc *p;

	p = allocproc();
	first = 1;
	initproc = p;

	p->parent = NULL;
	p->sibling_prev = NULL;
	p->sibling_next = NULL;

	// allocate one user page and copy init's instruction 
	// and data into it 
	uvminit(p->pagetable, initcode, sizeof(initcode));

	// for ostest
	// uvminit(p->pagetable, ostest_code, sizeof(ostest_code));
	
	struct seg *s = kmalloc(sizeof(struct seg));
	s->addr = 0;
	s->sz = PGSIZE;
	s->next = NULL;
	s->type = LOAD;
	p->segment = s;

	// prepare for the very first "return" from kernel to user 
	p->trapframe->epc = 0x0;
	p->trapframe->sp = PGSIZE;

	safestrcpy(p->name, "initcode", sizeof(p->name));

	p->tmask = 0;

	__enter_proc_cs 
	__insert_runnable(PRIORITY_NORMAL, p);
	__leave_proc_cs 

	__debug_info("userinit", "init\n");
}

// Grow or shrink user memory by n bytes. 
// Return 0 on success, -1 on failure 
int growproc(int n) {
  struct proc *p = myproc();

  struct seg *heap = getseg(p->segment, HEAP);
  uint64 boundary = heap->next == NULL ? MAXUVA : heap->next->addr;

  uint64 oldva = heap->addr + heap->sz;
  uint64 newva = oldva + n;

  if(n > 0){
    if (newva > boundary - PGSIZE ||
        uvmalloc(p->pagetable, oldva, newva, PTE_W|PTE_R) == 0) {
      return -1;
    }
  } else if(n < 0){
	if (newva < heap->addr || newva > oldva) {
	  newva = heap->addr;
	}

	uvmdealloc(p->pagetable, newva, oldva, HEAP);
	sfence_vma();
  }
  heap->sz += n;
  return 0;
}

// Copy to either a user address, or kernel address,
// depending on usr_dst.
// Returns 0 on success, -1 on error.
int
either_copyout(int user_dst, uint64 dst, void *src, uint64 len)
{
  // struct proc *p = myproc();
  if(user_dst){
	// return copyout(p->pagetable, dst, src, len);
	return copyout2(dst, src, len);
  } else {
	memmove((char*)dst, src, len);
	return 0;
  }
}

// Copy from either a user address, or kernel address,
// depending on usr_src.
// Returns 0 on success, -1 on error.
int
either_copyin(void *dst, int user_src, uint64 src, uint64 len)
{
  // struct proc *p = myproc();
  if(user_src){
	// return copyin(p->pagetable, dst, src, len);
	return copyin2(dst, src, len);
  } else {
	memmove(dst, (char*)src, len);
	return 0;
  }
}

static inline char const *state_to_str(enum procstate state) {
	switch (state) {
	case RUNNABLE: 		return "runnable"; 
	case RUNNING: 		return "running "; 
	case SLEEPING: 		return "sleeping";
	case ZOMBIE: 		return "zombie  ";
	default: 			return "\e[31;1m????????\e[0m";
	}
}

static void print_list_no_lock(struct proc const *list) {
	while (list) {
		uint64 load = 0, heap = 0;
		for (struct seg const *s = list->segment; s != NULL; s = s->next) {
			if (s->type == LOAD) {
				load += s->sz;
			} else if (s->type == HEAP) {
				heap = s->sz;
			}
		}
		printf("%d\t%d\t%s\t%d\t%s\t%d\t%d\n", 
			list->pid, 
			initproc == list ? 0 : list->parent->pid, 
			state_to_str(list->state),
			list->killed, 
			list->name, 
			load, heap
		);
		list = list->next;
	}
}

// print current processes to console 
void procdump(void) {
	printf("epc = %p\n", r_sepc());
    acquire(&pid_lock);
    printf("pid = %d\n", next_pid);
    release(&pid_lock);
	printf("\nPID\tPPID\tSTATE\tKILLED\tNAME\tMEM_LOAD\tMEM_HEAP\n");
	__enter_proc_cs 

	// display runnable 
	for (int i = 0; i < PRIORITY_NUMBER; i ++) {
		print_list_no_lock(proc_runnable[i]);
	}

	// display sleeping 
	print_list_no_lock(proc_sleep);

	// display zombie 
	print_list_no_lock(proc_zombie);

	__leave_proc_cs 
}

int procnum(void) {
	int num = 0;
	struct proc const *p;

	__enter_proc_cs 

	// search runnable 
	for (int i = 0; i < PRIORITY_NUMBER; i ++) {
		p = proc_runnable[i];
		while (NULL != p) {
			num ++;
			p = p->next;
		}
	}
	// search sleep 
	p = proc_sleep;
	while (NULL != p) {
		num ++;
		p = p->next;
	}
	// search zombie 
	p = proc_zombie;
	while (NULL != p) {
		num ++;
		p = p->next;
	}

	__leave_proc_cs 

	return num;
}

void reg_info(void) {
	printf("register info: {\n");
	printf("sstatus: %p\n", r_sstatus());
	printf("sip: %p\n", r_sip());
	printf("sie: %p\n", r_sie());
	printf("sepc: %p\n", r_sepc());
	printf("stvec: %p\n", r_stvec());
	printf("satp: %p\n", r_satp());
	printf("scause: %p\n", r_scause());
	printf("stval: %p\n", r_stval());
	printf("sp: %p\n", r_sp());
	printf("tp: %p\n", r_tp());
	printf("ra: %p\n", r_ra());
	printf("}\n");
}
