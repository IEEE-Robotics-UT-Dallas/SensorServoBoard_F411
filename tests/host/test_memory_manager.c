/*
 * test_memory_manager.c — Unit tests for custom_memory_manager.c
 * Tests pvPortMallocMicroROS, vPortFreeMicroROS, pvPortReallocMicroROS,
 * pvPortCallocMicroROS, and getBlockSize.
 */
#include "unity.h"
#include <string.h>
#include <stdint.h>

/* Functions under test (defined in custom_memory_manager.c) */
extern void  *pvPortMallocMicroROS(size_t xWantedSize);
extern void   vPortFreeMicroROS(void *pv);
extern void  *pvPortReallocMicroROS(void *pv, size_t xWantedSize);
extern void  *pvPortCallocMicroROS(size_t num, size_t xWantedSize);
extern size_t getBlockSize(void *pv);

void setUp(void) { }
void tearDown(void) { }

/* ---- pvPortMallocMicroROS / vPortFreeMicroROS ---- */

void test_malloc_returns_non_null(void)
{
    void *p = pvPortMallocMicroROS(64);
    TEST_ASSERT_NOT_NULL(p);
    vPortFreeMicroROS(p);
}

void test_malloc_and_free_no_crash(void)
{
    void *p = pvPortMallocMicroROS(128);
    TEST_ASSERT_NOT_NULL(p);
    memset(p, 0xAB, 128); /* write to verify it's usable */
    vPortFreeMicroROS(p);
}

/* ---- getBlockSize ---- */

void test_getBlockSize_returns_at_least_requested(void)
{
    void *p = pvPortMallocMicroROS(100);
    TEST_ASSERT_NOT_NULL(p);
    size_t bs = getBlockSize(p);
    TEST_ASSERT_GREATER_OR_EQUAL(100, bs);
    vPortFreeMicroROS(p);
}

/* ---- pvPortReallocMicroROS ---- */

void test_realloc_null_ptr_acts_as_malloc(void)
{
    void *p = pvPortReallocMicroROS(NULL, 32);
    TEST_ASSERT_NOT_NULL(p);
    vPortFreeMicroROS(p);
}

void test_realloc_size_zero_frees_and_returns_null(void)
{
    void *p = pvPortMallocMicroROS(64);
    TEST_ASSERT_NOT_NULL(p);
    void *result = pvPortReallocMicroROS(p, 0);
    TEST_ASSERT_NULL(result);
    /* p is freed inside realloc; no double-free */
}

void test_realloc_copies_data(void)
{
    uint8_t *p = (uint8_t *)pvPortMallocMicroROS(16);
    TEST_ASSERT_NOT_NULL(p);
    for (int i = 0; i < 16; i++) p[i] = (uint8_t)i;

    uint8_t *q = (uint8_t *)pvPortReallocMicroROS(p, 32);
    TEST_ASSERT_NOT_NULL(q);
    for (int i = 0; i < 16; i++) {
        TEST_ASSERT_EQUAL_UINT8(i, q[i]);
    }
    vPortFreeMicroROS(q);
}

void test_realloc_to_smaller_preserves_prefix(void)
{
    uint8_t *p = (uint8_t *)pvPortMallocMicroROS(64);
    TEST_ASSERT_NOT_NULL(p);
    for (int i = 0; i < 64; i++) p[i] = (uint8_t)(i + 10);

    uint8_t *q = (uint8_t *)pvPortReallocMicroROS(p, 16);
    TEST_ASSERT_NOT_NULL(q);
    for (int i = 0; i < 16; i++) {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(i + 10), q[i]);
    }
    vPortFreeMicroROS(q);
}

/* ---- pvPortCallocMicroROS ---- */

void test_calloc_returns_zeroed_memory(void)
{
    uint8_t *p = (uint8_t *)pvPortCallocMicroROS(10, 8);
    TEST_ASSERT_NOT_NULL(p);
    for (int i = 0; i < 80; i++) {
        TEST_ASSERT_EQUAL_UINT8(0, p[i]);
    }
    vPortFreeMicroROS(p);
}

void test_calloc_zero_count(void)
{
    void *p = pvPortCallocMicroROS(0, 16);
    /* Implementation: total = 0*16 = 0, malloc(0) is impl-defined */
    if (p != NULL)
        vPortFreeMicroROS(p);
}

void test_calloc_zero_size(void)
{
    void *p = pvPortCallocMicroROS(5, 0);
    /* total = 5*0 = 0, malloc(0) is impl-defined */
    if (p != NULL)
        vPortFreeMicroROS(p);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_malloc_returns_non_null);
    RUN_TEST(test_malloc_and_free_no_crash);
    RUN_TEST(test_getBlockSize_returns_at_least_requested);
    RUN_TEST(test_realloc_null_ptr_acts_as_malloc);
    RUN_TEST(test_realloc_size_zero_frees_and_returns_null);
    RUN_TEST(test_realloc_copies_data);
    RUN_TEST(test_realloc_to_smaller_preserves_prefix);
    RUN_TEST(test_calloc_returns_zeroed_memory);
    RUN_TEST(test_calloc_zero_count);
    RUN_TEST(test_calloc_zero_size);
    return UNITY_END();
}
