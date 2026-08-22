/*
 * master_disco.c — Host-side pamodbus discovery MASTER test.
 *
 * Talks to one or more pamodbus discovery slaves over a real RS485 link using
 * the termios provider in pa_serial.c. Run it together with a slave on the
 * same bus, either the host slave_disco tool or any embedded slave/disco
 * target firmware.
 *
 * Usage:
 *   ./master_disco <device> <baud> [cycles]
 *   ./master_disco /dev/ttyUSB0 115200
 *
 * Exits 0 if a discovery cycle found at least one slave, 1 otherwise.
 */

/* Enable usleep() under strict -std=c99. */
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pamodbus.h"
#include "pamodbus_internal.h"       /* concrete pa_modbus_t */
#include "pamodbus-disco.h"
#include "pamodbus-disco-internal.h" /* concrete pa_disco_master_t */
#include "pa_port.h"
#include "pa_serial.h"

/* ------------------------------------------------------------------------- */

static const pa_disco_settings_t settings = {
    .window_time        = 250,   /* ms reply window  */
    .refresh_period     = 0,     /* 0 = no periodic refresh */
    .repeat_cycles      = 3,
    .window_guard_time  = 50,    /* ms guard time    */
    .reset_repeat_cycles= 2,
    .verify_repeat_cycles= 4,
};

static pa_modbus_t       pam;
static uint8_t           tx_buf[256];
static uint8_t           rx_buf[256];
static pa_disco_master_t disco;

static int found = 0;

/* disco get_ticks: the disco userdata is the pa_port_t* */
static uint32_t port_ticks(void *userdata)
{
    pa_port_t *port = (pa_port_t *)userdata;
    return port->ticks(port->userdata);
}

static void disco_flush(void *userdata)
{
    pa_port_t *port = (pa_port_t *)userdata;
    if (port && port->flush) port->flush(port->userdata);
}

static void notify(pa_disco_list_t *list, pa_disco_slave_t *slave, void *userdata)
{
    (void)userdata;
    if (slave == NULL) {
        printf("[master] discovery cycle complete: %d slave(s)\n",
               pa_disco_list_count(list));
        fflush(stdout);
        return;
    }
    /* A slave was verified and added to the list — this is the success event. */
    printf("[master] discovered slave id=%u\n", pa_disco_slave_get_id(slave));
    fflush(stdout);
    found = 1;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s <device> <baud> [cycles]\n", argv[0]);
        return 2;
    }
    const char *device = argv[1];
    uint32_t baud = (uint32_t)atoi(argv[2]);
    /* argv[3] = scan window in seconds (default 12). */

    pa_serial_t *s = pa_serial_open(device, baud);
    if (s == NULL) return 1;

    pa_port_t port;
    pa_serial_build_port(s, &port);

    pa_modbus_init(&pam);
    pa_modbus_set_framer(&pam, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&pam, tx_buf, sizeof tx_buf);
    pa_modbus_set_rxbuf(&pam, rx_buf, sizeof rx_buf);
    pa_modbus_set_slave(&pam, 0x01u);
    pa_modbus_set_send_cb(&pam, port.send, port.userdata);
    pa_modbus_set_recv_cb(&pam, port.recv, port.userdata);
    pa_modbus_set_ticks_cb(&pam, port.ticks, port.userdata);
    /* Raise idle timeout above the service cadence so a partially-arrived
     * verify response is not discarded as "stale" between retries. */
    pa_modbus_set_rx_idle_timeout(&pam, 300);
    pa_modbus_set_mode(&pam, PA_MODE_MASTER);

    pa_disco_master_init(&disco, &pam);
    pa_disco_master_set_settings(&disco, &settings);
    pa_disco_master_set_callbacks(&disco,
        port_ticks, disco_flush,
        /* lock/unlock/trylock no-ops (single host master) */ NULL, NULL, NULL,
        notify, &port);

    printf("[master] %s @ %u baud - scanning...\n", device, baud);
    fflush(stdout);

    /* Time-boxed scan: keep serving the discovery state machine for a fixed
     * wall-clock window, stopping early as soon as a slave is confirmed.
     * The service cadence acts like an OS yield: the disco master's verify
     * step consumes one non-blocking recv per attempt, so it needs the slave
     * (and real serial transport) time to reply between calls. */
    uint32_t start = port.ticks(port.userdata);
    const uint32_t max_ms = (argc > 3) ? (uint32_t)atoi(argv[3]) * 1000u : 12000u;
    while (!found && (port.ticks(port.userdata) - start) < max_ms) {
        pa_disco_master_service(&disco);
        usleep(20000);  /* ~20 ms "yield" between discovery state-machine steps */
    }

    printf("[master] result: %s\n", found ? "PASS" : "FAIL");
    fflush(stdout);
    pa_serial_close(s);
    return found ? 0 : 1;
}