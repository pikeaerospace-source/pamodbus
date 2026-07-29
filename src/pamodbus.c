/*
 * pamodbus — core API: init, configuration, callbacks, I/O helpers
 * MIT License — see LICENSE file for details.
 */

#include "pamodbus_internal.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * Initialization
 * ------------------------------------------------------------------------- */

void pa_modbus_init(pa_modbus_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->framer_type = PA_FRAMER_RTU;
    ctx->framer = &pa_framer_rtu_ops;
    ctx->slave = 0xFF; /* Respond to all by default */
    ctx->discovery_addr = 0; /* Disabled by default */
    ctx->last_error = PA_OK;
}

/* ---------------------------------------------------------------------------
 * Framer configuration
 * ------------------------------------------------------------------------- */

void pa_modbus_set_framer(pa_modbus_t *ctx, pa_framer_t framer)
{
    ctx->framer_type = framer;
    switch (framer) {
        case PA_FRAMER_RTU:
            ctx->framer = &pa_framer_rtu_ops;
            break;
        case PA_FRAMER_TCP:
            ctx->framer = &pa_framer_tcp_ops;
            break;
    }
}

pa_framer_t pa_modbus_get_framer(const pa_modbus_t *ctx)
{
    return ctx->framer_type;
}

/* ---------------------------------------------------------------------------
 * Buffer configuration
 * ------------------------------------------------------------------------- */

void pa_modbus_set_txbuf(pa_modbus_t *ctx, uint8_t *buf, size_t size)
{
    ctx->txbuf = buf;
    ctx->txbuf_size = size;
}

void pa_modbus_set_rxbuf(pa_modbus_t *ctx, uint8_t *buf, size_t size)
{
    ctx->rxbuf = buf;
    ctx->rxbuf_size = size;
}

/* ---------------------------------------------------------------------------
 * Slave / unit identifier
 * ------------------------------------------------------------------------- */

void pa_modbus_set_slave(pa_modbus_t *ctx, uint8_t slave)
{
    ctx->slave = slave;
}

uint8_t pa_modbus_get_slave(const pa_modbus_t *ctx)
{
    return ctx->slave;
}

/* ---------------------------------------------------------------------------
 * Discovery address
 * ------------------------------------------------------------------------- */

void pa_modbus_set_discovery_addr(pa_modbus_t *ctx, uint8_t addr)
{
    ctx->discovery_addr = addr;
}

uint8_t pa_modbus_get_discovery_addr(const pa_modbus_t *ctx)
{
    return ctx->discovery_addr;
}

/* ---------------------------------------------------------------------------
 * I/O callback registration
 * ------------------------------------------------------------------------- */

void pa_modbus_set_send_cb(pa_modbus_t *ctx, pa_send_fn send, void *userdata)
{
    ctx->send_cb = send;
    ctx->send_userdata = userdata;
}

void pa_modbus_set_recv_cb(pa_modbus_t *ctx, pa_recv_fn recv, void *userdata)
{
    ctx->recv_cb = recv;
    ctx->recv_userdata = userdata;
}

/* ---------------------------------------------------------------------------
 * Register callback registration (slave mode)
 * ------------------------------------------------------------------------- */

void pa_modbus_set_read_coils_cb(pa_modbus_t *ctx, pa_read_coils_fn cb, void *userdata)
{
    ctx->read_coils_cb = cb;
    ctx->read_coils_userdata = userdata;
}

void pa_modbus_set_read_discrete_inputs_cb(pa_modbus_t *ctx, pa_read_discrete_inputs_fn cb, void *userdata)
{
    ctx->read_discrete_inputs_cb = cb;
    ctx->read_discrete_inputs_userdata = userdata;
}

void pa_modbus_set_read_holding_registers_cb(pa_modbus_t *ctx, pa_read_holding_registers_fn cb, void *userdata)
{
    ctx->read_holding_registers_cb = cb;
    ctx->read_holding_registers_userdata = userdata;
}

void pa_modbus_set_read_input_registers_cb(pa_modbus_t *ctx, pa_read_input_registers_fn cb, void *userdata)
{
    ctx->read_input_registers_cb = cb;
    ctx->read_input_registers_userdata = userdata;
}

void pa_modbus_set_write_single_coil_cb(pa_modbus_t *ctx, pa_write_single_coil_fn cb, void *userdata)
{
    ctx->write_single_coil_cb = cb;
    ctx->write_single_coil_userdata = userdata;
}

void pa_modbus_set_write_single_register_cb(pa_modbus_t *ctx, pa_write_single_register_fn cb, void *userdata)
{
    ctx->write_single_register_cb = cb;
    ctx->write_single_register_userdata = userdata;
}

void pa_modbus_set_write_multiple_coils_cb(pa_modbus_t *ctx, pa_write_multiple_coils_fn cb, void *userdata)
{
    ctx->write_multiple_coils_cb = cb;
    ctx->write_multiple_coils_userdata = userdata;
}

void pa_modbus_set_write_multiple_registers_cb(pa_modbus_t *ctx, pa_write_multiple_registers_fn cb, void *userdata)
{
    ctx->write_multiple_registers_cb = cb;
    ctx->write_multiple_registers_userdata = userdata;
}

/* ---------------------------------------------------------------------------
 * TX buffer accessors
 * ------------------------------------------------------------------------- */

const uint8_t *pa_modbus_tx_buf(const pa_modbus_t *ctx)
{
    return ctx->txbuf;
}

size_t pa_modbus_tx_len(const pa_modbus_t *ctx)
{
    return ctx->tx_len;
}

/* ---------------------------------------------------------------------------
 * Master response data accessors
 * ------------------------------------------------------------------------- */

pa_error_t pa_modbus_get_error(const pa_modbus_t *ctx)
{
    return ctx->last_error;
}

uint8_t pa_modbus_get_exception(const pa_modbus_t *ctx)
{
    return ctx->last_exception;
}

uint8_t pa_modbus_get_coil(const pa_modbus_t *ctx, uint16_t idx)
{
    if (idx >= ctx->last_coil_count) return 0;
    return (ctx->last_coil_data[idx / 8] >> (idx % 8)) & 1;
}

uint16_t pa_modbus_get_register(const pa_modbus_t *ctx, uint16_t idx)
{
    if (idx >= ctx->last_reg_count) return 0;
    return ctx->last_reg_data[idx];
}

/* ---------------------------------------------------------------------------
 * Slave request data accessors
 * ------------------------------------------------------------------------- */

uint8_t pa_modbus_slave_function(const pa_modbus_t *ctx)
{
    return ctx->last_function;
}

uint16_t pa_modbus_slave_addr(const pa_modbus_t *ctx)
{
    return ctx->slave_addr;
}

uint16_t pa_modbus_slave_count(const pa_modbus_t *ctx)
{
    return ctx->slave_count;
}

const uint8_t *pa_modbus_slave_coil_data(const pa_modbus_t *ctx)
{
    return ctx->slave_coil_data;
}

const uint16_t *pa_modbus_slave_reg_data(const pa_modbus_t *ctx)
{
    return ctx->slave_reg_data;
}

/* ---------------------------------------------------------------------------
 * Convenience I/O helpers
 * ------------------------------------------------------------------------- */

int pa_modbus_send(pa_modbus_t *ctx)
{
    if (!ctx->send_cb)
        return PA_ERR_STATE;

    int ret = ctx->send_cb(ctx->txbuf, ctx->tx_len, ctx->send_userdata);
    return (ret == 0) ? PA_OK : PA_ERR_CALLBACK;
}

int pa_modbus_recv(pa_modbus_t *ctx)
{
    if (!ctx->recv_cb)
        return PA_ERR_STATE;

    int n = ctx->recv_cb(ctx->rxbuf, ctx->rxbuf_size, ctx->recv_userdata);
    if (n < 0)
        return PA_ERR_CALLBACK;
    if (n == 0)
        return PA_ERR_TIMEOUT;

    /* Feed to the appropriate parser.
     * We determine mode based on whether slave callbacks are registered
     * as a heuristic. A more robust approach would require the user to
     * explicitly set the mode, but this keeps the API simple.
     *
     * If any write callback is registered, assume slave mode.
     * Otherwise, assume master mode.
     */
    if (ctx->write_single_coil_cb || ctx->write_single_register_cb ||
        ctx->write_multiple_coils_cb || ctx->write_multiple_registers_cb) {
        return pa_modbus_slave_feed(ctx, ctx->rxbuf, (size_t)n);
    } else {
        return pa_modbus_master_feed(ctx, ctx->rxbuf, (size_t)n);
    }
}