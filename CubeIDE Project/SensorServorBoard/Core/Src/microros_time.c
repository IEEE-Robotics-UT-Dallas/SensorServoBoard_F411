#ifdef MICRO_ROS_ENABLED

#include <unistd.h>
#include <time.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#define NANOSECONDS_PER_SECOND     (1000000000LL)
#define NANOSECONDS_PER_TICK       (NANOSECONDS_PER_SECOND / configTICK_RATE_HZ)

void UTILS_NanosecondsToTimespec(int64_t llSource,
                                  struct timespec * const pxDestination)
{
    long lCarrySec = 0;

    pxDestination->tv_sec = (time_t)(llSource / NANOSECONDS_PER_SECOND);
    pxDestination->tv_nsec = (long)(llSource % NANOSECONDS_PER_SECOND);

    if (pxDestination->tv_nsec < 0L)
    {
        lCarrySec = (pxDestination->tv_nsec / (long)NANOSECONDS_PER_SECOND) + 1L;
        pxDestination->tv_sec -= (time_t)(lCarrySec);
        pxDestination->tv_nsec += lCarrySec * (long)NANOSECONDS_PER_SECOND;
    }
}

int clock_gettime(int clock_id, struct timespec * tp)
{
    TimeOut_t xCurrentTime = {0};
    uint64_t ullTickCount = 0ULL;

    (void) clock_id;

    vTaskSetTimeOutState(&xCurrentTime);

    ullTickCount = (uint64_t)(xCurrentTime.xOverflowCount) << (sizeof(TickType_t) * 8);
    ullTickCount += xCurrentTime.xTimeOnEntering;

    UTILS_NanosecondsToTimespec((int64_t)ullTickCount * NANOSECONDS_PER_TICK, tp);

    return 0;
}

#endif /* MICRO_ROS_ENABLED */
