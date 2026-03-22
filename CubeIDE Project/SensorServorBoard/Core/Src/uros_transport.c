/**
 * @file uros_transport.c
 * @brief F411 micro-ROS UART/DMA custom transport implementation.
 *
 * Uses a 512-byte circular DMA receive buffer with __HAL_DMA_GET_COUNTER
 * for zero-copy tail tracking.  Transmit is DMA-backed with busy-wait
 * on completion via osDelay.
 *
 * KEY FIX: The STM32F4 HAL treats ANY UART error (ORE/FE/NE) as a
 * "blocking error" when DMA RX is active (see HAL_UART_IRQHandler
 * line ~2416).  It calls UART_EndRxTransfer + HAL_DMA_Abort_IT, which
 * permanently kills the circular DMA RX — no more data will ever be
 * received.  We disable the UART error interrupt (EIE) after starting
 * DMA so that byte-level errors are silently dropped; the XRCE-DDS CRC
 * catches any resulting corruption at the protocol layer.
 */
#include "uros_transport.h"

#ifdef MICRO_ROS_ENABLED
#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

#include "main.h"
#include "cmsis_os.h"
#include <string.h>

#define TRANSPORT_WRITE_TIMEOUT_MS  1000

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0;
static size_t dma_tail = 0;

/* Restart DMA RX circular transfer if it was stopped (e.g. by a HAL
 * error handler that ran before we could disable EIE, or by an
 * explicit DMAStop elsewhere). */
static void ensure_dma_rx_running(UART_HandleTypeDef * uart)
{
    if (uart->RxState == HAL_UART_STATE_READY)
    {
        /* Clear any pending error flags (read SR then DR) */
        volatile uint32_t sr = uart->Instance->SR;
        volatile uint32_t dr = uart->Instance->DR;
        (void)sr;
        (void)dr;
        uart->ErrorCode = HAL_UART_ERROR_NONE;

        dma_head = 0;
        dma_tail = 0;
        HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
        __HAL_UART_DISABLE_IT(uart, UART_IT_ERR);
    }
}

bool cubemx_transport_open(struct uxrCustomTransport * transport)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef *) transport->args;

    dma_head = 0;
    dma_tail = 0;

    /* Clear any pending error flags before starting DMA */
    volatile uint32_t sr = uart->Instance->SR;
    volatile uint32_t dr = uart->Instance->DR;
    (void)sr;
    (void)dr;
    uart->ErrorCode = HAL_UART_ERROR_NONE;

    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);

    /* Disable UART Error Interrupt (EIE in CR3).  The STM32F4 HAL
     * treats ORE/FE/NE as "blocking errors" in DMA mode and aborts
     * the entire DMA RX transfer — fatal for a circular transport.
     * With EIE off, the DMA continues even if a byte is corrupted. */
    __HAL_UART_DISABLE_IT(uart, UART_IT_ERR);

    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef *) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport * transport,
                              const uint8_t * buf, size_t len,
                              uint8_t * err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef *) transport->args;

    if (uart->gState != HAL_UART_STATE_READY)
    {
        return 0;
    }

    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(uart, (uint8_t *) buf, len);
    if (ret != HAL_OK)
    {
        return 0;
    }

    int ms_waited = 0;
    while (uart->gState != HAL_UART_STATE_READY
           && ms_waited < TRANSPORT_WRITE_TIMEOUT_MS)
    {
        osDelay(1);
        ms_waited++;
    }

    if (uart->gState != HAL_UART_STATE_READY)
    {
        HAL_UART_DMAStop(uart);
        return 0;
    }

    return len;
}

size_t cubemx_transport_read(struct uxrCustomTransport * transport,
                             uint8_t * buf, size_t len,
                             int timeout, uint8_t * err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef *) transport->args;

    /* Auto-recover DMA RX if the HAL error handler killed it */
    ensure_dma_rx_running(uart);

    int ms_used = 0;

    while (ms_used < timeout)
    {
        __disable_irq();
        dma_tail = UART_DMA_BUFFER_SIZE
                   - __HAL_DMA_GET_COUNTER(uart->hdmarx);
        __enable_irq();

        if (dma_head != dma_tail)
            break;

        osDelay(1);
        ms_used++;
    }

    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len))
    {
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }

    return wrote;
}

#endif /* RMW_UXRCE_TRANSPORT_CUSTOM */
#endif /* MICRO_ROS_ENABLED */
