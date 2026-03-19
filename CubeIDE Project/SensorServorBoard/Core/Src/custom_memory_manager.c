/*
 * Unified micro-ROS memory manager — delegates to the standard FreeRTOS heap
 * (heap_4.c) instead of maintaining a separate 40KB pool.
 * Provides realloc and calloc wrappers that FreeRTOS heap_4 lacks.
 */

#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ---- helpers for block-size introspection (heap_4 internals) ---- */

typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;
    size_t xBlockSize;
} BlockLink_t;

static const size_t xHeapStructSize =
    (sizeof(BlockLink_t) + ((size_t)(portBYTE_ALIGNMENT - 1)))
    & ~((size_t)portBYTE_ALIGNMENT_MASK);

static const size_t xBlockAllocatedBit =
    ((size_t)1) << ((sizeof(size_t) * 8) - 1);

/* Return usable payload size of an allocated block (excl. header). */
size_t getBlockSize(void *pv)
{
    uint8_t *puc = (uint8_t *)pv - xHeapStructSize;
    BlockLink_t *pxLink = (BlockLink_t *)puc;
    return (pxLink->xBlockSize & ~xBlockAllocatedBit) - xHeapStructSize;
}

/* ---- micro-ROS allocation API ---- */

void *pvPortMallocMicroROS(size_t xWantedSize)
{
    return pvPortMalloc(xWantedSize);
}

void vPortFreeMicroROS(void *pv)
{
    vPortFree(pv);
}

void *pvPortReallocMicroROS(void *pv, size_t xWantedSize)
{
    if (pv == NULL)
        return pvPortMalloc(xWantedSize);

    if (xWantedSize == 0) {
        vPortFree(pv);
        return NULL;
    }

    void *newmem = pvPortMalloc(xWantedSize);
    if (newmem != NULL) {
        size_t oldSize = getBlockSize(pv);
        memcpy(newmem, pv, (xWantedSize < oldSize) ? xWantedSize : oldSize);
        vPortFree(pv);
    }
    return newmem;
}

void *pvPortCallocMicroROS(size_t num, size_t xWantedSize)
{
    size_t total = num * xWantedSize;
    void *mem = pvPortMalloc(total);
    if (mem != NULL)
        memset(mem, 0, total);
    return mem;
}
