#ifndef TIME_H__
#define TIME_H__
#define time_t	unsigned int
struct timespec {
   time_t tv_sec;      /* Seconds */
   long   tv_nsec;     /* Nanoseconds [0 .. 999999999] */
};
#define CLOCK_REALTIME	0
#define clockid_t	int
#endif
