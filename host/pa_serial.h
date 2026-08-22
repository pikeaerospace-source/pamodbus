/*
 * pa_serial.h — POSIX (termios) RS485/serial provider implementing pa_port_t
 *
 * A concrete `pa_port_t` implementation for host-side testing over a real
 * serial/RS485 device (e.g. a USB-RS485 adapter at /dev/ttyUSB0). Pairs with
 * master_disco.c / slave_disco.c in this directory.
 *
 * Usage:
 *     pa_serial_t *s = pa_serial_open("/dev/ttyUSB0", 115200);
 *     pa_port_t port;
 *     pa_serial_build_port(s, &port);   // fills port.{send,recv,ticks,flush,...}
 *     ... hand &port to pamodbus / pamodbus-disco ...
 *     pa_serial_close(s);
 */

#ifndef PA_SERIAL_H
#define PA_SERIAL_H

#include <stdint.h>
#include "pa_port.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pa_serial pa_serial_t;

/**
 * Open and configure the serial device in raw 8N1 mode at the given baud.
 * @param device  e.g. "/dev/ttyUSB0"
 * @param baud    e.g. 115200
 * @return opaque handle, or NULL on error.
 */
pa_serial_t *pa_serial_open(const char *device, uint32_t baud);

/** Close the device and free the handle. */
void pa_serial_close(pa_serial_t *h);

/**
 * Fill `port` with a pa_port_t backed by this serial handle. The port's
 * userdata points at the handle. `port` is a caller-owned struct and must
 * outlive use; it is safe to reuse a single pa_serial_t to build many ports.
 */
void pa_serial_build_port(pa_serial_t *h, pa_port_t *port);

/** Discard pending input (tcsflush). */
void pa_serial_flush_input(pa_serial_t *h);

#ifdef __cplusplus
}
#endif

#endif /* PA_SERIAL_H */