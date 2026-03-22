/*
 * test_microros_allocators.c — Unit tests for microros_allocators.c
 * Tests memory tracking via absoluteUsedMemory / usedMemory counters.
 */
#include "unity.h"
#include <string.h>
#include <stdint.h>

/* Globals defined in microros_allocators.c */
extern int absoluteUsedMemory;
extern int usedMemory;

/* Functions under test (defined in microros_allocators.c) */
extern void *microros_allocate(size_t size, void *state);
extern void  microros_deallocate(void *pointer, void *state);
extern void *microros_reallocate(void *pointer, size_t size, void *state);
extern void *microros_zero_allocate(size_t number_of_elements,
                                    size_t size_of_element, void *state);

/* getBlockSize from custom_memory_manager.c (needed for size verification) */
extern size_t getBlockSize(void *pv);

void setUp(void)
{
    absoluteUsedMemory = 0;
    usedMemory = 0;
}

void tearDown(void) { }

/* ---- microros_allocate ---- */

void test_allocate_increases_both_counters(void)
{
    void *p = microros_allocate(100, NULL);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_INT(100, absoluteUsedMemory);
    TEST_ASSERT_EQUAL_INT(100, usedMemory);
    microros_deallocate(p, NULL);
}

void test_allocate_multiple_accumulates(void)
{
    void *p1 = microros_allocate(50, NULL);
    void *p2 = microros_allocate(30, NULL);
    TEST_ASSERT_EQUAL_INT(80, absoluteUsedMemory);
    TEST_ASSERT_EQUAL_INT(80, usedMemory);
    microros_deallocate(p1, NULL);
    microros_deallocate(p2, NULL);
}

/* ---- microros_deallocate ---- */

void test_deallocate_decreases_usedMemory_not_absoluteUsedMemory(void)
{
    void *p = microros_allocate(64, NULL);
    int abs_after_alloc = absoluteUsedMemory;
    microros_deallocate(p, NULL);

    TEST_ASSERT_EQUAL_INT(abs_after_alloc, absoluteUsedMemory);
    /* usedMemory decreases by getBlockSize(p), which is >= 64 */
    TEST_ASSERT_TRUE(usedMemory <= 0);
}

void test_deallocate_null_is_safe(void)
{
    microros_deallocate(NULL, NULL);
    TEST_ASSERT_EQUAL_INT(0, absoluteUsedMemory);
    TEST_ASSERT_EQUAL_INT(0, usedMemory);
}

/* ---- microros_zero_allocate ---- */

void test_zero_allocate_returns_zeroed_memory(void)
{
    uint8_t *p = (uint8_t *)microros_zero_allocate(10, 8, NULL);
    TEST_ASSERT_NOT_NULL(p);
    for (int i = 0; i < 80; i++) {
        TEST_ASSERT_EQUAL_UINT8(0, p[i]);
    }
    microros_deallocate(p, NULL);
}

void test_zero_allocate_tracks_correctly(void)
{
    void *p = microros_zero_allocate(5, 20, NULL);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_INT(100, absoluteUsedMemory);
    TEST_ASSERT_EQUAL_INT(100, usedMemory);
    microros_deallocate(p, NULL);
}

/* ---- microros_reallocate ---- */

void test_reallocate_null_pointer_tracks_correctly(void)
{
    void *p = microros_reallocate(NULL, 50, NULL);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_INT(50, absoluteUsedMemory);
    TEST_ASSERT_EQUAL_INT(50, usedMemory);
    microros_deallocate(p, NULL);
}

void test_reallocate_existing_pointer_tracks_correctly(void)
{
    void *p = microros_allocate(40, NULL);
    TEST_ASSERT_NOT_NULL(p);
    size_t old_block_size = getBlockSize(p);

    void *q = microros_reallocate(p, 80, NULL);
    TEST_ASSERT_NOT_NULL(q);

    /* absoluteUsedMemory = 40 (alloc) + 80 (realloc) = 120 */
    TEST_ASSERT_EQUAL_INT(120, absoluteUsedMemory);
    /* usedMemory = 40 + 80 - old_block_size */
    TEST_ASSERT_EQUAL_INT((int)(40 + 80 - old_block_size), usedMemory);
    microros_deallocate(q, NULL);
}

/* ---- Alloc/dealloc cycles ---- */

void test_multiple_cycles_absoluteUsedMemory_only_grows(void)
{
    void *p1 = microros_allocate(100, NULL);
    microros_deallocate(p1, NULL);
    int abs_after_first = absoluteUsedMemory;

    void *p2 = microros_allocate(200, NULL);
    microros_deallocate(p2, NULL);
    int abs_after_second = absoluteUsedMemory;

    TEST_ASSERT_EQUAL_INT(100, abs_after_first);
    TEST_ASSERT_EQUAL_INT(300, abs_after_second);
    TEST_ASSERT_GREATER_OR_EQUAL(abs_after_first, abs_after_second);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_allocate_increases_both_counters);
    RUN_TEST(test_allocate_multiple_accumulates);
    RUN_TEST(test_deallocate_decreases_usedMemory_not_absoluteUsedMemory);
    RUN_TEST(test_deallocate_null_is_safe);
    RUN_TEST(test_zero_allocate_returns_zeroed_memory);
    RUN_TEST(test_zero_allocate_tracks_correctly);
    RUN_TEST(test_reallocate_null_pointer_tracks_correctly);
    RUN_TEST(test_reallocate_existing_pointer_tracks_correctly);
    RUN_TEST(test_multiple_cycles_absoluteUsedMemory_only_grows);
    return UNITY_END();
}
