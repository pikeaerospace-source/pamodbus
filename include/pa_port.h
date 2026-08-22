/*
 * pa_port.h — Generic RS485 / serial transport interface for pamodbus
 *
 * MIT License — see LICENSE file for details.
 *
 * A minimal, board- and OS-independent transport abstraction used by the
 * pamodbus discovery (and later generic Modbus) feature tests. A "port" is a
 * small struct of function pointers. The callback signatures are chosen to
 * match the pamodbus I/O callback typedefs ({pa_send_fn, pa_recv_fn,
 * pa_ticks_fn}) so a pa_port implementation can be handed *directly* to
 * pa_modbus_set_send_cb / pa_modbus_set_recv_cb / pa_modbus_set_ticks_cb.
 *
 * Two concrete providers ship alongside this header:
 *   - a POSIX termios implementation for host testing: host/pa_serial.c
 *   - an embedded board-provider glue to be supplied per target
 *
 * Disable the unused "set_dir" hook on full-duplex transports by leaving it
 * NULL; the discovery feature only calls it when non-NULL.
 */

#ifndef PA_PORT_H
#define PA_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Open/initialise the transport. May be NULL if already open by the caller. */
typedef int (*pa_port_open_fn)(void *userdata);

/** Send raw bytes. Returns 0 on success, negative on error. */
typedef int (*pa_port_send_fn)(const uint8_t *data, size_t len, void *userdata);

/**
 * Non-blocking receive: copy whatever is currently available (up to max_len)
 * into data and return the number of bytes copied; 0 means "no data right now".
 * Negative on error.
 */
typedef int (*pa_port_recv_fn)(uint8_t *data, size_t max_len, void *userdata);

/** Monotonic tick counter in milliseconds (drives idle framing/timeouts). */
typedef uint32_t (*pa_port_ticks_fn)(void *userdata);

/**
 * Optional half-duplex direction control (RS485 DE/RE / direction GPIO).
 * May be NULL on full-duplex transports. tx=true => transmit mode.
 */
typedef int (*pa_port_set_dir_fn)(void *userdata, bool tx);

/** Discard all pending received bytes. */
typedef void (*pa_port_flush_fn)(void *userdata);

/** Blocking delay in milliseconds. May be NULL if the consumer never needs it. */
typedef void (*pa_port_delay_ms_fn)(uint32_t ms, void *userdata);

/** A concrete RS485 / serial transport. */
typedef struct pa_port {
    void                *userdata;   /**< Opaque provider state, passed to every fn. */
    pa_port_open_fn      open;       /**< Optional. */
    pa_port_send_fn      send;       /**< Required. */
    pa_port_recv_fn      recv;       /**< Required, non-blocking. */
    pa_port_ticks_fn     ticks;      /**< Required (monotonic ms). */
    pa_port_set_dir_fn   set_dir;    /**< Optional (RS485 half-duplex). */
    pa_port_flush_fn     flush;      /**< May be NULL. */
    pa_port_delay_ms_fn  delay_ms;   /**< May be NULL. */
} pa_port_t;

#ifdef __cplusplus
}
#endif

#endif /* PA_PORT_H */