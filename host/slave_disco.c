/*
 * slave_disco.c — Host-side pamodbus discovery SLAVE test.
 *
 * Talks to a pamodbus discovery master over a real RS485 link using the
 * termios provider in pa_serial.c. Pairs with the host master_disco tool or
 * any embedded master/disco target firmware.
 *
 * Usage:
 *   ./slave_disco <device> <baud> [timeout_s]
 *   ./slave_disco /dev/ttyUSB0 115200
 *
 * Exits 0 once a master has discovered and assigned this slave an ID;
 * exits 1 on timeout.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pamodbus.h"
#include "pamodbus_internal.h"       /* concrete pa_modbus_t */
#include "pamodbus-disco.h"
#include "pamodbus-disco-internal.h" /* concrete pa_disco_slave_dev_t / map */
#include "pa_port.h"
#include "pa_serial.h"

#define SERIALNO_WORDS  6

/* ------------------------------------------------------------------------- */

static pa_modbus_t            pam;
static uint8_t                tx_buf[256];
static uint8_t                rx_buf[256];
static pa_disco_slave_dev_t   disco;
static pa_disco_register_map_t map;
static uint16_t               holding[PA_DISCO_HOLDING_NREGS];
static uint16_t               serialno[SERIALNO_WORDS];
static uint8_t                assigned_id = 0;

/* disco get_ticks: the disco userdata is the pa_port_t* */
static uint32_t port_ticks(void *userdata)
{
    pa_port_t *port = (pa_port_t *)userdata;
    return port->ticks(port->userdata);
}
static void slave_flush(void *userdata)
{
    pa_port_t *port = (pa_port_t *)userdata;
    if (port && port->flush) port->flush(port->userdata);
}
static void slave_delay(uint32_t ms, void *userdata)
{
    pa_port_t *port = (pa_port_t *)userdata;
    if (port && port->delay_ms) port->delay_ms(ms, port->userdata);
}

/* Simple deterministic LCG for response-window jitter. */
static uint32_t s_rand = 0x2468ACE0u;
static uint32_t slave_random(uint32_t min, uint32_t max, void *userdata)
{
    (void)userdata;
    if (max <= min) return min;
    s_rand = s_rand * 1664525u + 1013904223u;
    return min + (s_rand % (max - min + 1u));
}

/* Register-map request callback: forward discovery ops to the disco slave. */
static int req_cb(void *arg)
{
    (void)arg;
    uint8_t  fn  = pa_modbus_slave_function(&pam);
    uint16_t reg = pa_modbus_slave_addr(&pam);
    uint16_t cnt = pa_modbus_slave_count(&pam);
    int do_reply = 1;
    switch (fn) {
        case 0x03: do_reply = pa_disco_slave_read_cb(&disco, (int)reg, cnt); break;
        case 0x06: do_reply = pa_disco_slave_write_cb(&disco, (int)reg, 1);  break;
        case 0x10: do_reply = pa_disco_slave_write_cb(&disco, (int)reg, cnt); break;
        default:   break;
    }
    return do_reply;
}

static void notify(uint8_t slave_id, void *userdata)
{
    (void)userdata;
    assigned_id = slave_id;
    printf("[slave] discovery complete: assigned id=%u\n", slave_id);
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s <device> <baud> [timeout_s]\n", argv[0]);
        return 2;
    }
    const char *device = argv[1];
    uint32_t baud = (uint32_t)atoi(argv[2]);
    int timeout_s = (argc > 3) ? atoi(argv[3]) : 30;

    pa_serial_t *s = pa_serial_open(device, baud);
    if (s == NULL) return 1;

    pa_port_t port;
    pa_serial_build_port(s, &port);

    pa_modbus_init(&pam);
    pa_modbus_set_framer(&pam, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&pam, tx_buf, sizeof tx_buf);
    pa_modbus_set_rxbuf(&pam, rx_buf, sizeof rx_buf);
    pa_modbus_set_slave(&pam, 0x00u);                    /* unassigned */
    pa_modbus_set_discovery_addr(&pam, PA_DISCO_ADDR);
    pa_modbus_set_send_cb(&pam, port.send, port.userdata);
    pa_modbus_set_recv_cb(&pam, port.recv, port.userdata);
    pa_modbus_set_ticks_cb(&pam, port.ticks, port.userdata);
    pa_modbus_set_mode(&pam, PA_MODE_SLAVE);

    pa_disco_register_map_init(&map, &pam);
    /* Holding window starts at 0 so the verify register (0) is serviced; the
     * discovery regs (PA_DISCO_REG_START..) fall inside it. */
    pa_disco_register_map_set_holding(&map, holding, 0, PA_DISCO_HOLDING_NREGS);
    /* Populate the discovery identity (id + serial) inside holding[]: */
    holding[PA_DISCO_REG_START] = 0;                                  /* AFX_ID */
    for (int i = 0; i < SERIALNO_WORDS; i++)
        holding[PA_DISCO_REG_START + 1 + i] = serialno[i];
    pa_disco_register_map_set_req_cb(&map, req_cb, NULL);

    for (int i = 0; i < SERIALNO_WORDS; i++) serialno[i] = (uint16_t)(i + 1u);
    pa_disco_slave_init(&disco, &pam);
    pa_disco_slave_set_callbacks(&disco,
        port_ticks /*get_ticks*/, slave_delay, slave_flush, slave_random, notify, &port);
    pa_disco_slave_set_window(&disco, 10u, 60u);  /* response window (ms) */
    pa_disco_slave_set_serialno(&disco, serialno);

    printf("[slave] %s @ %u baud — waiting to be discovered...\n", device, baud);

    uint32_t start = port.ticks(port.userdata);
    uint32_t assigned_start = 0;     /* time of assignment, 0 until assigned */
    int rc = 1;
    for (;;) {
        if (pa_disco_slave_service(&disco)) {
            pa_disco_register_map_service(&map);
        }
        uint32_t now = port.ticks(port.userdata);

        /* Keep serving the register map for a short window after assignment so
         * the master can complete its verify read-back, then report PASS. */
        if (assigned_id != 0u && assigned_start == 0u) {
            assigned_start = now;
        }
        if (assigned_start != 0u) {
            if ((now - assigned_start) >= 2000u) {
                rc = 0;
                break;
            }
            continue;
        }

        if ((now - start) >= (uint32_t)timeout_s * 1000u) {
            printf("[slave] timeout waiting for discovery\n");
            break;
        }
    }

    printf("[slave] result: %s (assigned=%u)\n", rc ? "FAIL" : "PASS", assigned_id);
    pa_serial_close(s);
    return rc;
}