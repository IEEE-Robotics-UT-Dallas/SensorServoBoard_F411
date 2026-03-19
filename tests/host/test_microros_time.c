/*
 * test_microros_time.c — Unit tests for microros_time.c
 * Tests UTILS_NanosecondsToTimespec (pure math) and clock_gettime
 * (renamed to microros_clock_gettime via -D to avoid libc conflict).
 */
#include "unity.h"
#include <time.h>
#include <stdint.h>
#include "hal_stubs.h"

/*
 * UTILS_NanosecondsToTimespec is defined in microros_time.c.
 * clock_gettime is renamed to microros_clock_gettime via -Dclock_gettime=...
 */
extern void UTILS_NanosecondsToTimespec(int64_t llSource,
                                         struct timespec * const pxDestination);
extern int microros_clock_gettime(int clock_id, struct timespec *tp);

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

void setUp(void)
{
    hal_stub_set_tick_overflow(0);
    hal_stub_set_tick_entering(0);
}

void tearDown(void) { }

/* ---- UTILS_NanosecondsToTimespec ---- */

void test_nanoseconds_zero(void)
{
    struct timespec ts;
    UTILS_NanosecondsToTimespec(0, &ts);
    TEST_ASSERT_EQUAL(0, ts.tv_sec);
    TEST_ASSERT_EQUAL(0, ts.tv_nsec);
}

void test_nanoseconds_one_second(void)
{
    struct timespec ts;
    UTILS_NanosecondsToTimespec(1000000000LL, &ts);
    TEST_ASSERT_EQUAL(1, ts.tv_sec);
    TEST_ASSERT_EQUAL(0, ts.tv_nsec);
}

void test_nanoseconds_1_5_seconds(void)
{
    struct timespec ts;
    UTILS_NanosecondsToTimespec(1500000000LL, &ts);
    TEST_ASSERT_EQUAL(1, ts.tv_sec);
    TEST_ASSERT_EQUAL(500000000L, ts.tv_nsec);
}

void test_nanoseconds_subsecond(void)
{
    struct timespec ts;
    UTILS_NanosecondsToTimespec(123456789LL, &ts);
    TEST_ASSERT_EQUAL(0, ts.tv_sec);
    TEST_ASSERT_EQUAL(123456789L, ts.tv_nsec);
}

void test_nanoseconds_large_value(void)
{
    /* 5,000,000,000 ns = 5 seconds — exceeds 2^32 */
    struct timespec ts;
    UTILS_NanosecondsToTimespec(5000000000LL, &ts);
    TEST_ASSERT_EQUAL(5, ts.tv_sec);
    TEST_ASSERT_EQUAL(0, ts.tv_nsec);
}

void test_nanoseconds_large_with_remainder(void)
{
    /* 4,294,967,296 ns = 4s + 294,967,296ns (exactly 2^32 ns) */
    struct timespec ts;
    UTILS_NanosecondsToTimespec(4294967296LL, &ts);
    TEST_ASSERT_EQUAL(4, ts.tv_sec);
    TEST_ASSERT_EQUAL(294967296L, ts.tv_nsec);
}

/* ---- clock_gettime (renamed to microros_clock_gettime) ---- */

void test_clock_gettime_zero_ticks(void)
{
    hal_stub_set_tick_overflow(0);
    hal_stub_set_tick_entering(0);

    struct timespec ts;
    int ret = microros_clock_gettime(CLOCK_REALTIME, &ts);
    TEST_ASSERT_EQUAL(0, ret);
    TEST_ASSERT_EQUAL(0, ts.tv_sec);
    TEST_ASSERT_EQUAL(0, ts.tv_nsec);
}

void test_clock_gettime_1000_ticks_equals_1_second(void)
{
    /* configTICK_RATE_HZ=1000 → 1000 ticks = 1 second */
    hal_stub_set_tick_overflow(0);
    hal_stub_set_tick_entering(1000);

    struct timespec ts;
    int ret = microros_clock_gettime(CLOCK_REALTIME, &ts);
    TEST_ASSERT_EQUAL(0, ret);
    TEST_ASSERT_EQUAL(1, ts.tv_sec);
    TEST_ASSERT_EQUAL(0, ts.tv_nsec);
}

void test_clock_gettime_1500_ticks(void)
{
    /* 1500 ticks = 1.5 seconds */
    hal_stub_set_tick_overflow(0);
    hal_stub_set_tick_entering(1500);

    struct timespec ts;
    int ret = microros_clock_gettime(CLOCK_REALTIME, &ts);
    TEST_ASSERT_EQUAL(0, ret);
    TEST_ASSERT_EQUAL(1, ts.tv_sec);
    TEST_ASSERT_EQUAL(500000000L, ts.tv_nsec);
}

void test_clock_gettime_with_overflow(void)
{
    /*
     * overflow=1, entering=0 → ullTickCount = 1 << 32 = 4294967296 ticks
     * ns = 4294967296 * 1000000 = 4294967296000000
     * sec = 4294967296000000 / 1000000000 = 4294967
     * nsec = 4294967296000000 % 1000000000 = 296000000
     */
    hal_stub_set_tick_overflow(1);
    hal_stub_set_tick_entering(0);

    struct timespec ts;
    int ret = microros_clock_gettime(CLOCK_REALTIME, &ts);
    TEST_ASSERT_EQUAL(0, ret);
    TEST_ASSERT_EQUAL((time_t)4294967, ts.tv_sec);
    TEST_ASSERT_EQUAL(296000000L, ts.tv_nsec);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nanoseconds_zero);
    RUN_TEST(test_nanoseconds_one_second);
    RUN_TEST(test_nanoseconds_1_5_seconds);
    RUN_TEST(test_nanoseconds_subsecond);
    RUN_TEST(test_nanoseconds_large_value);
    RUN_TEST(test_nanoseconds_large_with_remainder);
    RUN_TEST(test_clock_gettime_zero_ticks);
    RUN_TEST(test_clock_gettime_1000_ticks_equals_1_second);
    RUN_TEST(test_clock_gettime_1500_ticks);
    RUN_TEST(test_clock_gettime_with_overflow);
    return UNITY_END();
}
