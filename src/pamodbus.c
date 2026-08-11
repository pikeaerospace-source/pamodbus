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

    /* Receive state defaults. mode is left unset (legacy heuristic active)
     * until pa_modbus_set_mode() is called. */
    ctx->mode = PA_MODE_MASTER;
    ctx->mode_set = 0;
    ctx->ticks_cb = NULL;
    ctx->ticks_userdata = NULL;
    ctx->idle_timeout_ticks = 10; /* Only used once ticks_cb is set. */
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
 * Receive role / timing
 * ------------------------------------------------------------------------- */

void pa_modbus_set_mode(pa_modbus_t *ctx, pa_modbus_mode_t mode)
{
    ctx->mode = mode;
    ctx->mode_set = 1;
}

pa_modbus_mode_t pa_modbus_get_mode(const pa_modbus_t *ctx)
{
    return ctx->mode;
}

void pa_modbus_set_ticks_cb(pa_modbus_t *ctx, pa_ticks_fn ticks, void *userdata)
{
    ctx->ticks_cb = ticks;
    ctx->ticks_userdata = userdata;
}

void pa_modbus_set_rx_idle_timeout(pa_modbus_t *ctx, uint32_t ticks)
{
    ctx->idle_timeout_ticks = ticks;
}

void pa_modbus_rx_reset(pa_modbus_t *ctx)
{
    ctx->rx_len = 0;
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

/* Select the receive/parse role: explicit mode if set, else a legacy
 * heuristic (write callbacks registered => slave, else master). */
static pa_modbus_mode_t pa_rx_mode(const pa_modbus_t *ctx)
{
    if (ctx->mode_set)
        return ctx->mode;
    if (ctx->write_single_coil_cb || ctx->write_single_register_cb ||
        ctx->write_multiple_coils_cb || ctx->write_multiple_registers_cb)
        return PA_MODE_SLAVE;
    return PA_MODE_MASTER;
}

/* Scan the accumulator for a complete, validated framed message.
 * On success returns PA_OK and reports where it starts (start) and its total
 * length including framing (frame_len). Returns PA_WAIT if no complete frame
 * is present yet. Scanning from every offset makes the receiver robust to a
 * garbage/stale prefix before a real frame (resync). */
static int pa_rx_scan_frame(pa_modbus_t *ctx, size_t *start, size_t *frame_len)
{
    for (size_t off = 0; off < ctx->rx_len; off++) {
        const uint8_t *pdu;
        size_t pdu_len;
        int r = ctx->framer->unwrap(ctx, ctx->rxbuf + off, ctx->rx_len - off,
                                    &pdu, &pdu_len);
        if (r == PA_OK) {
            *start = off;
            /* Full frame length = bytes before the PDU + PDU + trailing CRC (RTU). */
            *frame_len = (size_t)(pdu - (ctx->rxbuf + off)) + pdu_len;
            if (ctx->framer_type == PA_FRAMER_RTU)
                *frame_len += 2; /* trailing CRC-16 */
            return PA_OK;
        }
        /* unwrap returned "need more" (r > 0) or an error (r < 0): not a
         * complete frame at this offset; keep scanning. */
    }
    return PA_WAIT;
}

int pa_modbus_recv(pa_modbus_t *ctx)
{
    if (!ctx->recv_cb || !ctx->rxbuf || ctx->rxbuf_size == 0)
        return PA_ERR_STATE;

    uint32_t now = ctx->ticks_cb ? ctx->ticks_cb(ctx->ticks_userdata) : 0;

    /* 1. Non-blocking append of whatever is currently available. */
    int n = ctx->recv_cb(ctx->rxbuf + ctx->rx_len,
                         ctx->rxbuf_size - ctx->rx_len,
                         ctx->recv_userdata);
    if (n < 0)
        return PA_ERR_CALLBACK;
    if (n > 0) {
        ctx->rx_len += (size_t)n;
        ctx->last_rx_tick = now;
    }

    /* 2. Try to finalize a complete, validated frame from the accumulator. */
    if (ctx->rx_len > 0) {
        size_t start, frame_len;
        if (pa_rx_scan_frame(ctx, &start, &frame_len) == PA_OK) {
            /* Parse and dispatch now that the full frame is present. */
            int ret = (pa_rx_mode(ctx) == PA_MODE_SLAVE)
                ? pa_modbus_slave_feed(ctx, ctx->rxbuf + start, frame_len)
                : pa_modbus_master_feed(ctx, ctx->rxbuf + start, frame_len);

            /* Consume the framed message; keep any trailing bytes so a
             * back-to-back frame in the same drain is handled next poll. */
            size_t consumed = start + frame_len;
            size_t remaining = ctx->rx_len - consumed;
            if (remaining > 0) {
                memmove(ctx->rxbuf, ctx->rxbuf + consumed, remaining);
                ctx->last_rx_tick = now;
            }
            ctx->rx_len = remaining;
            return ret;
        }

        /* Partial bytes present but no complete frame yet: guard overflow. */
        if (ctx->rx_len >= ctx->rxbuf_size) {
            ctx->rx_len = 0;
            return PA_ERR_BUFFER_FULL;
        }
    }

    /* 3. Idle timeout: enough silence that the partial bytes must be a
     *    stale/garbage frame; discard them and resync. */
    if (ctx->ticks_cb && ctx->rx_len > 0 &&
        (uint32_t)(now - ctx->last_rx_tick) >= ctx->idle_timeout_ticks) {
        ctx->rx_len = 0;
        return PA_ERR_TIMEOUT;
    }

    return PA_WAIT; /* keep polling, nothing complete yet */
}

/* ---------------------------------------------------------------------------
 * Raw I/O helpers — for custom protocols (e.g. discovery)
 * ------------------------------------------------------------------------- */

int pa_modbus_send_raw(pa_modbus_t *ctx, const uint8_t *pdu, size_t pdu_len)
{
    if (!ctx->send_cb)
        return PA_ERR_STATE;
    if (!ctx->txbuf)
        return PA_ERR_STATE;
    if (pdu_len > ctx->txbuf_size - PA_FRAMER_MAX_OVERHEAD)
        return PA_ERR_BUFFER_FULL;

    /* Copy the raw PDU into the TX buffer */
    memcpy(ctx->txbuf, pdu, pdu_len);

    /* Apply framing (adds slave address + CRC for RTU, MBAP for TCP) */
    int framed;
    int ret = ctx->framer->wrap(ctx, (int)pdu_len, &framed);
    if (ret != PA_OK)
        return ret;

    ctx->tx_len = (size_t)framed;

    /* Send the framed frame */
    ret = ctx->send_cb(ctx->txbuf, ctx->tx_len, ctx->send_userdata);
    return (ret == 0) ? (int)ctx->tx_len : PA_ERR_CALLBACK;
}

int pa_modbus_recv_raw(pa_modbus_t *ctx, uint8_t *pdu_buf, size_t *pdu_len)
{
    if (!ctx->recv_cb)
        return PA_ERR_STATE;
    if (!ctx->rxbuf || ctx->rxbuf_size == 0 || !pdu_buf || !pdu_len)
        return PA_ERR_BAD_PARAM;

    uint32_t now = ctx->ticks_cb ? ctx->ticks_cb(ctx->ticks_userdata) : 0;

    /* Non-blocking append of whatever is currently available. */
    int n = ctx->recv_cb(ctx->rxbuf + ctx->rx_len,
                         ctx->rxbuf_size - ctx->rx_len,
                         ctx->recv_userdata);
    if (n < 0)
        return PA_ERR_CALLBACK;
    if (n > 0) {
        ctx->rx_len += (size_t)n;
        ctx->last_rx_tick = now;
    }

    /* Custom protocol frames don't follow standard MODBUS length rules, so we
     * locate them by scanning for a CRC-valid frame length. The frame starts at
     * rxbuf[0] (frame: slave(1) + PDU(N) + crc(2)). */
    if (ctx->rx_len >= 4) {
        for (size_t frame_len = 4; frame_len <= ctx->rx_len; frame_len++) {
            uint16_t crc_received = (uint16_t)(ctx->rxbuf[frame_len - 2] |
                                               ((uint16_t)ctx->rxbuf[frame_len - 1] << 8));
            if (pa_crc16(ctx->rxbuf, frame_len - 2) == crc_received) {
                size_t pdu_len_out = frame_len - 3; /* subtract slave(1) + crc(2) */
                if (pdu_len_out > *pdu_len)
                    return PA_ERR_BUFFER_FULL;
                memcpy(pdu_buf, ctx->rxbuf + 1, pdu_len_out);
                *pdu_len = pdu_len_out;
                ctx->rx_len = 0; /* consume the frame */
                return PA_OK;
            }
        }
    }

    /* Accumulated bytes but no valid frame yet: guard overflow. */
    if (ctx->rx_len >= ctx->rxbuf_size) {
        ctx->rx_len = 0;
        return PA_ERR_BUFFER_FULL;
    }

    /* Idle timeout: discard a stale/partial frame and resync. */
    if (ctx->ticks_cb && ctx->rx_len > 0 &&
        (uint32_t)(now - ctx->last_rx_tick) >= ctx->idle_timeout_ticks) {
        ctx->rx_len = 0;
        return PA_ERR_TIMEOUT;
    }

    return PA_WAIT; /* keep polling */
}
