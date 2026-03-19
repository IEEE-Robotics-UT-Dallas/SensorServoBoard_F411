/**
 * @file uros_transport.h
 * @brief Board-agnostic micro-ROS custom transport API.
 *
 * Declares the uxrCustomTransport callback functions used by micro-XRCE-DDS.
 * The implementation is board-specific (F411: DMA circular buffer over UART).
 */
#ifndef UROS_TRANSPORT_H
#define UROS_TRANSPORT_H

#ifdef MICRO_ROS_ENABLED

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

#define UART_DMA_BUFFER_SIZE 512

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);

size_t cubemx_transport_write(struct uxrCustomTransport * transport,
                              const uint8_t * buf, size_t len,
                              uint8_t * err);

size_t cubemx_transport_read(struct uxrCustomTransport * transport,
                             uint8_t * buf, size_t len,
                             int timeout, uint8_t * err);

#endif /* RMW_UXRCE_TRANSPORT_CUSTOM */
#endif /* MICRO_ROS_ENABLED */
#endif /* UROS_TRANSPORT_H */
