#ifndef TIME_H__
#define TIME_H__
#define time_t	unsigned int
struct timespec {
   time_t tv_sec;      /* Seconds */
   long   tv_nsec;     /* Nanoseconds [0 .. 999999999] */
};
#endif
