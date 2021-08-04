#ifndef __RESOURCE_H
#define __RESOURCE_H

#include "time.h"

#define	RUSAGE_SELF			0
#define	RUSAGE_CHILDREN		(-1)
#define	RUSAGE_THREAD		1

struct rusage {
	struct timeval	ru_utime;
	struct timeval	ru_stime;
	long			ru_maxrss;
	long			ru_ixrss;
	long			ru_idrss;
	long			ru_isrss;
	long			ru_minflt;
	long			ru_majflt;
	long			ru_nswap;
	long			ru_inblock;
	long			ru_oublock;
	long			ru_msgsnd;
	long			ru_msgrcv;
	long			ru_nsignals;
	long			ru_nvcsw;
	long			ru_nivcsw;
	// long			__reserved[16];
};

#endif
