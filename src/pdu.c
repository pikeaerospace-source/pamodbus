/*
 * pamodbus — PDU core: build/parse function code + data payload
 * MIT License — see LICENSE file for details.
 *
 * Handles all standard MODBUS function codes (FC01–FC18).
 */

#include "pamodbus_internal.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

/** Write a 16-bit value in big-endian (MODBUS network byte order). */
static void put16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)((val >> 8) & 0xFF);
    buf[1] = (uint8_t)(val & 0xFF);
}

/** Read a 16-bit big-endian value. */
static uint16_t get16(const uint8_t *buf)
{
    return (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

/* ---------------------------------------------------------------------------
 * Master mode — building requests
 *
 * Each build function writes the PDU (function code + data) into ctx->txbuf
 * starting at offset 0, then calls the active framer's wrap() to add
 * transport-specific framing. Returns the total framed length on success,
 * or a negative pa_error_t on failure.
 * ------------------------------------------------------------------------- */

int pa_modbus_build_read_coils(pa_modbus_t *ctx, uint16_t addr, uint16_t count)
{
    if (count < 1 || count > 2000) return PA_ERR_BAD_PARAM;
    ctx->txbuf[0] = 0x01;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, count);
    int framed;
    int ret = ctx->framer->wrap(ctx, 5, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_read_discrete_inputs(pa_modbus_t *ctx, uint16_t addr, uint16_t count)
{
    if (count < 1 || count > 2000) return PA_ERR_BAD_PARAM;
    ctx->txbuf[0] = 0x02;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, count);
    int framed;
    int ret = ctx->framer->wrap(ctx, 5, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_read_holding_registers(pa_modbus_t *ctx, uint16_t addr, uint16_t count)
{
    if (count < 1 || count > 125) return PA_ERR_BAD_PARAM;
    ctx->txbuf[0] = 0x03;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, count);
    int framed;
    int ret = ctx->framer->wrap(ctx, 5, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_read_input_registers(pa_modbus_t *ctx, uint16_t addr, uint16_t count)
{
    if (count < 1 || count > 125) return PA_ERR_BAD_PARAM;
    ctx->txbuf[0] = 0x04;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, count);
    int framed;
    int ret = ctx->framer->wrap(ctx, 5, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_write_single_coil(pa_modbus_t *ctx, uint16_t addr, uint8_t value)
{
    ctx->txbuf[0] = 0x05;
    put16(ctx->txbuf + 1, addr);
    /* MODBUS: 0xFF00 = ON, 0x0000 = OFF */
    put16(ctx->txbuf + 3, value ? 0xFF00 : 0x0000);
    int framed;
    int ret = ctx->framer->wrap(ctx, 5, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_write_single_register(pa_modbus_t *ctx, uint16_t addr, uint16_t value)
{
    ctx->txbuf[0] = 0x06;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, value);
    int framed;
    int ret = ctx->framer->wrap(ctx, 5, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_write_multiple_coils(pa_modbus_t *ctx, uint16_t addr,
                                         const uint8_t *values, uint16_t count)
{
    if (count < 1 || count > 1968) return PA_ERR_BAD_PARAM;
    uint16_t byte_count = (uint16_t)((count + 7) / 8);
    size_t pdu_len = (size_t)(6 + byte_count);
    if (pdu_len > ctx->txbuf_size) return PA_ERR_BUFFER_FULL;

    ctx->txbuf[0] = 0x0F;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, count);
    ctx->txbuf[5] = (uint8_t)byte_count;
    memset(ctx->txbuf + 6, 0, byte_count);
    for (uint16_t i = 0; i < count; i++) {
        if (values[i])
            ctx->txbuf[6 + i / 8] |= (uint8_t)(1 << (i % 8));
    }
    int framed;
    int ret = ctx->framer->wrap(ctx, (int)pdu_len, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_report_slave_id(pa_modbus_t *ctx)
{
    ctx->txbuf[0] = 0x11;
    int framed;
    int ret = ctx->framer->wrap(ctx, 1, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_mask_write_register(pa_modbus_t *ctx, uint16_t addr, uint16_t and_mask, uint16_t or_mask)
{
    ctx->txbuf[0] = 0x16;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, and_mask);
    put16(ctx->txbuf + 5, or_mask);
    int framed;
    int ret = ctx->framer->wrap(ctx, 7, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_read_write_registers(pa_modbus_t *ctx, uint16_t read_addr, uint16_t read_count,
                                         uint16_t write_addr, const uint16_t *write_values, uint16_t write_count)
{
    if (read_count < 1 || read_count > 125) return PA_ERR_BAD_PARAM;
    if (write_count < 1 || write_count > 121) return PA_ERR_BAD_PARAM;
    if (!write_values) return PA_ERR_BAD_PARAM;

    uint16_t byte_count = (uint16_t)(write_count * 2);
    size_t pdu_len = (size_t)(10 + byte_count);
    if (pdu_len > ctx->txbuf_size) return PA_ERR_BUFFER_FULL;

    ctx->txbuf[0] = 0x17;
    put16(ctx->txbuf + 1, read_addr);
    put16(ctx->txbuf + 3, read_count);
    put16(ctx->txbuf + 5, write_addr);
    put16(ctx->txbuf + 7, write_count);
    ctx->txbuf[9] = (uint8_t)byte_count;
    for (uint16_t i = 0; i < write_count; i++)
        put16(ctx->txbuf + 10 + i * 2, write_values[i]);

    int framed;
    int ret = ctx->framer->wrap(ctx, (int)pdu_len, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_build_write_multiple_registers(pa_modbus_t *ctx, uint16_t addr,
                                             const uint16_t *values, uint16_t count)
{
    if (count < 1 || count > 123) return PA_ERR_BAD_PARAM;
    uint16_t byte_count = (uint16_t)(count * 2);
    size_t pdu_len = (size_t)(6 + byte_count);
    if (pdu_len > ctx->txbuf_size) return PA_ERR_BUFFER_FULL;

    ctx->txbuf[0] = 0x10;
    put16(ctx->txbuf + 1, addr);
    put16(ctx->txbuf + 3, count);
    ctx->txbuf[5] = (uint8_t)byte_count;
    for (uint16_t i = 0; i < count; i++)
        put16(ctx->txbuf + 6 + i * 2, values[i]);

    int framed;
    int ret = ctx->framer->wrap(ctx, (int)pdu_len, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

/* ---------------------------------------------------------------------------
 * Master mode — parsing responses
 * ------------------------------------------------------------------------- */

/**
 * Parse a PDU as a master response. Stores results in ctx->last_* fields.
 * Returns PA_OK on success, or a negative error code.
 */
static int parse_master_response(pa_modbus_t *ctx, const uint8_t *pdu, size_t pdu_len)
{
    if (pdu_len < 1) return PA_ERR_PROTOCOL;

    uint8_t fc = pdu[0];
    ctx->last_function = fc;

    /* Check for exception response (MSB of function code set) */
    if (fc & 0x80) {
        if (pdu_len < 2) return PA_ERR_PROTOCOL;
        ctx->last_exception = pdu[1];
        ctx->last_error = PA_ERR_EXCEPTION;
        return PA_ERR_EXCEPTION;
    }

    ctx->last_error = PA_OK;
    ctx->last_exception = 0;
    ctx->last_coil_count = 0;
    ctx->last_reg_count = 0;

    switch (fc) {
        case 0x01: /* Read Coils */
        case 0x02: /* Read Discrete Inputs */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                if (byte_count > sizeof(ctx->last_coil_data)) return PA_ERR_PROTOCOL;
                memcpy(ctx->last_coil_data, pdu + 2, byte_count);
                ctx->last_coil_count = (uint16_t)(byte_count * 8);
            }
            break;

        case 0x03: /* Read Holding Registers */
        case 0x04: /* Read Input Registers */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                if (byte_count / 2 > sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]))
                    return PA_ERR_PROTOCOL;
                ctx->last_reg_count = (uint16_t)(byte_count / 2);
                for (uint16_t i = 0; i < ctx->last_reg_count; i++)
                    ctx->last_reg_data[i] = get16(pdu + 2 + i * 2);
            }
            break;

        case 0x05: /* Write Single Coil */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            /* Echo: addr(2) + value(2). No data to extract. */
            break;

        case 0x06: /* Write Single Register */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            /* Echo: addr(2) + value(2). No data to extract. */
            break;

        case 0x07: /* Read Exception Status */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            /* Single byte status — store as first coil */
            ctx->last_coil_data[0] = pdu[1];
            ctx->last_coil_count = 8;
            break;

        case 0x08: /* Diagnostic */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            /* Echo: sub-function(2) + data(2). Store data as first register. */
            ctx->last_reg_data[0] = get16(pdu + 3);
            ctx->last_reg_count = 1;
            break;

        case 0x0B: /* Get Com Event Counter */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            ctx->last_reg_data[0] = get16(pdu + 1); /* status */
            ctx->last_reg_data[1] = get16(pdu + 3); /* event count */
            ctx->last_reg_count = 2;
            break;

        case 0x0C: /* Get Com Event Log */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                if (byte_count >= 4) {
                    ctx->last_reg_data[0] = get16(pdu + 2); /* status */
                    ctx->last_reg_data[1] = get16(pdu + 4); /* event count */
                    ctx->last_reg_count = 2;
                    /* Remaining bytes are the event log — store as coils */
                    size_t log_bytes = (size_t)(byte_count - 4);
                    if (log_bytes > sizeof(ctx->last_coil_data))
                        log_bytes = sizeof(ctx->last_coil_data);
                    memcpy(ctx->last_coil_data, pdu + 6, log_bytes);
                    ctx->last_coil_count = (uint16_t)(log_bytes * 8);
                }
            }
            break;

        case 0x0F: /* Write Multiple Coils */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            /* Echo: addr(2) + count(2). No data to extract. */
            break;

        case 0x10: /* Write Multiple Registers */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            /* Echo: addr(2) + count(2). No data to extract. */
            break;

        case 0x11: /* Report Server ID */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                /* Store raw data as registers (padded) */
                uint16_t reg_count = (uint16_t)((byte_count + 1) / 2);
                if (reg_count > sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]))
                    reg_count = (uint16_t)(sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]));
                ctx->last_reg_count = reg_count;
                for (uint16_t i = 0; i < reg_count; i++) {
                    uint16_t val = (uint16_t)pdu[2 + i * 2];
                    if (i * 2 + 1 < byte_count)
                        val = (uint16_t)((val << 8) | pdu[2 + i * 2 + 1]);
                    ctx->last_reg_data[i] = val;
                }
            }
            break;

        case 0x14: /* Read File Record */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                uint16_t reg_count = (uint16_t)(byte_count / 2);
                if (reg_count > sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]))
                    reg_count = (uint16_t)(sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]));
                ctx->last_reg_count = reg_count;
                for (uint16_t i = 0; i < reg_count; i++)
                    ctx->last_reg_data[i] = get16(pdu + 2 + i * 2);
            }
            break;

        case 0x15: /* Write File Record */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                /* Echo response — no data to extract beyond validation */
            }
            break;

        case 0x16: /* Mask Write Register */
            if (pdu_len < 7) return PA_ERR_PROTOCOL;
            /* Echo: addr(2) + and_mask(2) + or_mask(2) */
            ctx->last_reg_data[0] = get16(pdu + 1); /* address */
            ctx->last_reg_data[1] = get16(pdu + 3); /* AND mask */
            ctx->last_reg_data[2] = get16(pdu + 5); /* OR mask */
            ctx->last_reg_count = 3;
            break;

        case 0x17: /* Read/Write Multiple Registers */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                uint16_t reg_count = (uint16_t)(byte_count / 2);
                if (reg_count > sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]))
                    reg_count = (uint16_t)(sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]));
                ctx->last_reg_count = reg_count;
                for (uint16_t i = 0; i < reg_count; i++)
                    ctx->last_reg_data[i] = get16(pdu + 2 + i * 2);
            }
            break;

        case 0x18: /* Read FIFO Queue */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint16_t byte_count = get16(pdu + 1);
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                uint16_t fifo_count = byte_count / 2;
                if (fifo_count > sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]))
                    fifo_count = (uint16_t)(sizeof(ctx->last_reg_data) / sizeof(ctx->last_reg_data[0]));
                ctx->last_reg_count = fifo_count;
                for (uint16_t i = 0; i < fifo_count; i++)
                    ctx->last_reg_data[i] = get16(pdu + 2 + i * 2);
            }
            break;

        default:
            return PA_ERR_PROTOCOL;
    }

    return PA_OK;
}

/* ---------------------------------------------------------------------------
 * Slave mode — parsing requests
 * ------------------------------------------------------------------------- */

/**
 * Parse a PDU as a slave request. Stores request info in ctx->slave_* fields.
 * Returns PA_OK on success, or a negative error code.
 */
static int parse_slave_request(pa_modbus_t *ctx, const uint8_t *pdu, size_t pdu_len)
{
    if (pdu_len < 1) return PA_ERR_PROTOCOL;

    uint8_t fc = pdu[0];
    ctx->last_function = fc;
    ctx->slave_data_valid = 0;

    /* Exception responses should not arrive as requests */
    if (fc & 0x80)
        return PA_ERR_PROTOCOL;

    switch (fc) {
        case 0x01: /* Read Coils */
        case 0x02: /* Read Discrete Inputs */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = get16(pdu + 3);
            if (ctx->slave_count < 1 || ctx->slave_count > 2000) return PA_ERR_BAD_PARAM;
            break;

        case 0x03: /* Read Holding Registers */
        case 0x04: /* Read Input Registers */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = get16(pdu + 3);
            if (ctx->slave_count < 1 || ctx->slave_count > 125) return PA_ERR_BAD_PARAM;
            break;

        case 0x05: /* Write Single Coil */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = 1;
            ctx->slave_coil_data[0] = (get16(pdu + 3) == 0xFF00) ? 1 : 0;
            ctx->slave_coil_count = 1;
            ctx->slave_data_valid = 1;
            break;

        case 0x06: /* Write Single Register */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = 1;
            ctx->slave_reg_data[0] = get16(pdu + 3);
            ctx->slave_reg_count = 1;
            ctx->slave_data_valid = 1;
            break;

        case 0x07: /* Read Exception Status */
            if (pdu_len < 1) return PA_ERR_PROTOCOL;
            ctx->slave_addr = 0;
            ctx->slave_count = 0;
            break;

        case 0x08: /* Diagnostic */
            if (pdu_len < 5) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1); /* sub-function */
            ctx->slave_count = 0;
            ctx->slave_reg_data[0] = get16(pdu + 3); /* data */
            ctx->slave_reg_count = 1;
            ctx->slave_data_valid = 1;
            break;

        case 0x0B: /* Get Com Event Counter */
            if (pdu_len < 1) return PA_ERR_PROTOCOL;
            ctx->slave_addr = 0;
            ctx->slave_count = 0;
            break;

        case 0x0C: /* Get Com Event Log */
            if (pdu_len < 1) return PA_ERR_PROTOCOL;
            ctx->slave_addr = 0;
            ctx->slave_count = 0;
            break;

        case 0x0F: /* Write Multiple Coils */
            if (pdu_len < 6) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = get16(pdu + 3);
            if (ctx->slave_count < 1 || ctx->slave_count > 1968) return PA_ERR_BAD_PARAM;
            {
                uint8_t byte_count = pdu[5];
                if (pdu_len < (size_t)(6 + byte_count)) return PA_ERR_PROTOCOL;
                if (byte_count > sizeof(ctx->slave_coil_data))
                    return PA_ERR_PROTOCOL;
                memcpy(ctx->slave_coil_data, pdu + 6, byte_count);
                ctx->slave_coil_count = ctx->slave_count;
                ctx->slave_data_valid = 1;
            }
            break;

        case 0x10: /* Write Multiple Registers */
            if (pdu_len < 6) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = get16(pdu + 3);
            if (ctx->slave_count < 1 || ctx->slave_count > 123) return PA_ERR_BAD_PARAM;
            {
                uint8_t byte_count = pdu[5];
                if (pdu_len < (size_t)(6 + byte_count)) return PA_ERR_PROTOCOL;
                uint16_t reg_count = (uint16_t)(byte_count / 2);
                if (reg_count > sizeof(ctx->slave_reg_data) / sizeof(ctx->slave_reg_data[0]))
                    return PA_ERR_PROTOCOL;
                ctx->slave_reg_count = reg_count;
                for (uint16_t i = 0; i < reg_count; i++)
                    ctx->slave_reg_data[i] = get16(pdu + 6 + i * 2);
                ctx->slave_data_valid = 1;
            }
            break;

        case 0x11: /* Report Server ID */
            if (pdu_len < 1) return PA_ERR_PROTOCOL;
            ctx->slave_addr = 0;
            ctx->slave_count = 0;
            break;

        case 0x14: /* Read File Record */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                ctx->slave_addr = 0;
                ctx->slave_count = 0;
                /* Store raw request data for custom handling */
                uint16_t reg_count = (uint16_t)(byte_count / 2);
                if (reg_count > sizeof(ctx->slave_reg_data) / sizeof(ctx->slave_reg_data[0]))
                    return PA_ERR_PROTOCOL;
                ctx->slave_reg_count = reg_count;
                for (uint16_t i = 0; i < reg_count; i++)
                    ctx->slave_reg_data[i] = get16(pdu + 2 + i * 2);
                ctx->slave_data_valid = 1;
            }
            break;

        case 0x15: /* Write File Record */
            if (pdu_len < 2) return PA_ERR_PROTOCOL;
            {
                uint8_t byte_count = pdu[1];
                if (pdu_len < (size_t)(2 + byte_count)) return PA_ERR_PROTOCOL;
                ctx->slave_addr = 0;
                ctx->slave_count = 0;
                uint16_t reg_count = (uint16_t)(byte_count / 2);
                if (reg_count > sizeof(ctx->slave_reg_data) / sizeof(ctx->slave_reg_data[0]))
                    return PA_ERR_PROTOCOL;
                ctx->slave_reg_count = reg_count;
                for (uint16_t i = 0; i < reg_count; i++)
                    ctx->slave_reg_data[i] = get16(pdu + 2 + i * 2);
                ctx->slave_data_valid = 1;
            }
            break;

        case 0x16: /* Mask Write Register */
            if (pdu_len < 7) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = 1;
            ctx->slave_reg_data[0] = get16(pdu + 3); /* AND mask */
            ctx->slave_reg_data[1] = get16(pdu + 5); /* OR mask */
            ctx->slave_reg_count = 2;
            ctx->slave_data_valid = 1;
            break;

        case 0x17: /* Read/Write Multiple Registers */
            if (pdu_len < 11) return PA_ERR_PROTOCOL;
            {
                uint16_t read_addr  = get16(pdu + 1);
                uint16_t read_count = get16(pdu + 3);
                uint16_t write_addr = get16(pdu + 5);
                uint16_t write_count = get16(pdu + 7);
                if (read_count < 1 || read_count > 125) return PA_ERR_BAD_PARAM;
                if (write_count < 1 || write_count > 121) return PA_ERR_BAD_PARAM;
                uint8_t byte_count = pdu[9];
                if (pdu_len < (size_t)(10 + byte_count)) return PA_ERR_PROTOCOL;
                ctx->slave_addr = read_addr;
                ctx->slave_count = read_count;
                /* Store write data separately */
                uint16_t wc = (uint16_t)(byte_count / 2);
                if (wc > sizeof(ctx->slave_reg_data) / sizeof(ctx->slave_reg_data[0]))
                    return PA_ERR_PROTOCOL;
                /* We store write_addr in slave_coil_data[0..1] and write_count in slave_coil_data[2..3] */
                ctx->slave_coil_data[0] = (uint8_t)((write_addr >> 8) & 0xFF);
                ctx->slave_coil_data[1] = (uint8_t)(write_addr & 0xFF);
                ctx->slave_coil_data[2] = (uint8_t)((write_count >> 8) & 0xFF);
                ctx->slave_coil_data[3] = (uint8_t)(write_count & 0xFF);
                ctx->slave_reg_count = wc;
                for (uint16_t i = 0; i < wc; i++)
                    ctx->slave_reg_data[i] = get16(pdu + 10 + i * 2);
                ctx->slave_data_valid = 1;
            }
            break;

        case 0x18: /* Read FIFO Queue */
            if (pdu_len < 3) return PA_ERR_PROTOCOL;
            ctx->slave_addr = get16(pdu + 1);
            ctx->slave_count = 0;
            break;

        default:
            return PA_ERR_PROTOCOL;
    }

    return PA_OK;
}

/* ---------------------------------------------------------------------------
 * Slave mode — building responses
 * ------------------------------------------------------------------------- */

int pa_modbus_slave_respond(pa_modbus_t *ctx)
{
    uint8_t fc = ctx->last_function;
    int pdu_len = 0;
    int ret;

    switch (fc) {
        case 0x01: /* Read Coils */
        case 0x02: /* Read Discrete Inputs */
            {
                uint16_t count = ctx->slave_count;
                uint16_t byte_count = (uint16_t)((count + 7) / 8);
                uint8_t coil_data[256 / 8 + 1];
                memset(coil_data, 0, sizeof(coil_data));

                pa_read_coils_fn cb = (fc == 0x01) ? ctx->read_coils_cb : (pa_read_coils_fn)ctx->read_discrete_inputs_cb;
                void *ud = (fc == 0x01) ? ctx->read_coils_userdata : ctx->read_discrete_inputs_userdata;

                if (!cb) return PA_ERR_STATE;
                ret = cb(ctx->slave_addr, count, coil_data, ud);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                ctx->txbuf[1] = (uint8_t)byte_count;
                memcpy(ctx->txbuf + 2, coil_data, byte_count);
                pdu_len = 2 + (int)byte_count;
            }
            break;

        case 0x03: /* Read Holding Registers */
        case 0x04: /* Read Input Registers */
            {
                uint16_t count = ctx->slave_count;
                uint16_t byte_count = (uint16_t)(count * 2);
                uint16_t reg_data[128];

                pa_read_holding_registers_fn cb = (fc == 0x03) ? ctx->read_holding_registers_cb : (pa_read_holding_registers_fn)ctx->read_input_registers_cb;
                void *ud = (fc == 0x03) ? ctx->read_holding_registers_userdata : ctx->read_input_registers_userdata;

                if (!cb) return PA_ERR_STATE;
                ret = cb(ctx->slave_addr, count, reg_data, ud);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                ctx->txbuf[1] = (uint8_t)byte_count;
                for (uint16_t i = 0; i < count; i++)
                    put16(ctx->txbuf + 2 + i * 2, reg_data[i]);
                pdu_len = 2 + (int)byte_count;
            }
            break;

        case 0x05: /* Write Single Coil */
            {
                pa_write_single_coil_fn cb = ctx->write_single_coil_cb;
                if (!cb) return PA_ERR_STATE;
                ret = cb(ctx->slave_addr, ctx->slave_coil_data[0], ctx->write_single_coil_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, ctx->slave_addr);
                put16(ctx->txbuf + 3, ctx->slave_coil_data[0] ? 0xFF00 : 0x0000);
                pdu_len = 5;
            }
            break;

        case 0x06: /* Write Single Register */
            {
                pa_write_single_register_fn cb = ctx->write_single_register_cb;
                if (!cb) return PA_ERR_STATE;
                ret = cb(ctx->slave_addr, ctx->slave_reg_data[0], ctx->write_single_register_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, ctx->slave_addr);
                put16(ctx->txbuf + 3, ctx->slave_reg_data[0]);
                pdu_len = 5;
            }
            break;

        case 0x07: /* Read Exception Status */
            {
                /* Return a single byte status. We use read_coils_cb to get 8 bits. */
                pa_read_coils_fn cb = ctx->read_coils_cb;
                if (!cb) return PA_ERR_STATE;
                uint8_t status[1];
                ret = cb(0, 8, status, ctx->read_coils_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                ctx->txbuf[1] = status[0];
                pdu_len = 2;
            }
            break;

        case 0x08: /* Diagnostic */
            {
                /* Echo back the sub-function and data */
                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, ctx->slave_addr); /* sub-function */
                put16(ctx->txbuf + 3, ctx->slave_reg_data[0]);
                pdu_len = 5;
            }
            break;

        case 0x0B: /* Get Com Event Counter */
            {
                /* Return status=0x0000 and event count. Use read_holding_registers to get count. */
                pa_read_holding_registers_fn cb = ctx->read_holding_registers_cb;
                if (!cb) return PA_ERR_STATE;
                uint16_t regs[2];
                ret = cb(0xFFFE, 2, regs, ctx->read_holding_registers_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, regs[0]); /* status */
                put16(ctx->txbuf + 3, regs[1]); /* event count */
                pdu_len = 5;
            }
            break;

        case 0x0C: /* Get Com Event Log */
            {
                /* Return status(2) + event count(2) + message(up to 6 bytes) */
                pa_read_holding_registers_fn cb = ctx->read_holding_registers_cb;
                if (!cb) return PA_ERR_STATE;
                uint16_t regs[5];
                ret = cb(0xFFFC, 5, regs, ctx->read_holding_registers_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                uint8_t byte_count = 10; /* 2 status + 2 count + 6 message */
                ctx->txbuf[0] = fc;
                ctx->txbuf[1] = byte_count;
                put16(ctx->txbuf + 2, regs[0]); /* status */
                put16(ctx->txbuf + 4, regs[1]); /* event count */
                put16(ctx->txbuf + 6, regs[2]); /* message[0..1] */
                put16(ctx->txbuf + 8, regs[3]); /* message[2..3] */
                put16(ctx->txbuf + 10, regs[4]); /* message[4..5] */
                pdu_len = 2 + (int)byte_count;
            }
            break;

        case 0x0F: /* Write Multiple Coils */
            {
                pa_write_multiple_coils_fn cb = ctx->write_multiple_coils_cb;
                if (!cb) return PA_ERR_STATE;
                ret = cb(ctx->slave_addr, ctx->slave_count, ctx->slave_coil_data,
                         ctx->write_multiple_coils_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, ctx->slave_addr);
                put16(ctx->txbuf + 3, ctx->slave_count);
                pdu_len = 5;
            }
            break;

        case 0x10: /* Write Multiple Registers */
            {
                pa_write_multiple_registers_fn cb = ctx->write_multiple_registers_cb;
                if (!cb) return PA_ERR_STATE;
                ret = cb(ctx->slave_addr, ctx->slave_count, ctx->slave_reg_data,
                         ctx->write_multiple_registers_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, ctx->slave_addr);
                put16(ctx->txbuf + 3, ctx->slave_count);
                pdu_len = 5;
            }
            break;

        case 0x11: /* Report Server ID */
            {
                /* Use read_holding_registers to get server ID data */
                pa_read_holding_registers_fn cb = ctx->read_holding_registers_cb;
                if (!cb) return PA_ERR_STATE;
                uint16_t regs[64];
                ret = cb(0xFFF0, 64, regs, ctx->read_holding_registers_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                /* Build response: first reg[0] = byte count, then data */
                uint8_t byte_count = (uint8_t)(regs[0] & 0xFF);
                if (byte_count > 126) byte_count = 126;
                ctx->txbuf[0] = fc;
                ctx->txbuf[1] = byte_count;
                for (uint16_t i = 0; i < (uint16_t)(byte_count + 1) / 2 && i < 63; i++)
                    put16(ctx->txbuf + 2 + i * 2, regs[i + 1]);
                pdu_len = 2 + (int)byte_count;
            }
            break;

        case 0x14: /* Read File Record */
            {
                /* Echo back the request structure with data filled in.
                 * Use read_holding_registers for each sub-request. */
                pa_read_holding_registers_fn cb = ctx->read_holding_registers_cb;
                if (!cb) return PA_ERR_STATE;

                /* Build response by echoing request with data */
                /* For simplicity, we copy the request and fill in data.
                 * The request format is: ref_type(1) + file_num(2) + record_num(2) + record_len(2) */
                uint8_t req_byte_count = (uint8_t)(ctx->slave_reg_count * 2); (void)req_byte_count;
                uint8_t resp_byte_count = 0;

                ctx->txbuf[0] = fc;
                size_t offset = 2; /* start after fc + byte_count placeholder */

                /* Parse sub-requests from slave_reg_data */
                size_t req_offset = 0;
                while (req_offset < (size_t)ctx->slave_reg_count * 2) {
                    if (req_offset + 7 > (size_t)ctx->slave_reg_count * 2) break;
                    uint8_t  ref_type  = (uint8_t)(ctx->slave_reg_data[req_offset / 2] >> 8);
                    uint16_t file_num  = ctx->slave_reg_data[req_offset / 2 + 1];
                    uint16_t rec_num   = ctx->slave_reg_data[req_offset / 2 + 2];
                    uint16_t rec_len   = ctx->slave_reg_data[req_offset / 2 + 3];
                    (void)file_num;
                    (void)rec_num;

                    uint16_t data_regs[64];
                    if (rec_len > 64) rec_len = 64;
                    ret = cb(rec_num, rec_len, data_regs, ctx->read_holding_registers_userdata);
                    if (ret != 0) return PA_ERR_CALLBACK;

                    uint8_t sub_byte_count = (uint8_t)(rec_len * 2 + 1);
                    if (offset + 1 + sub_byte_count > ctx->txbuf_size) break;

                    ctx->txbuf[offset++] = sub_byte_count;
                    ctx->txbuf[offset++] = ref_type;
                    for (uint16_t i = 0; i < rec_len; i++) {
                        put16(ctx->txbuf + offset, data_regs[i]);
                        offset += 2;
                    }
                    resp_byte_count += (uint8_t)(1 + sub_byte_count);
                    req_offset += 7;
                }

                ctx->txbuf[1] = resp_byte_count;
                pdu_len = 2 + (int)resp_byte_count;
            }
            break;

        case 0x15: /* Write File Record */
            {
                /* Echo back the request on success */
                pa_write_multiple_registers_fn cb = ctx->write_multiple_registers_cb;
                if (!cb) return PA_ERR_STATE;

                /* Parse sub-requests from slave_reg_data */
                size_t offset = 0;
                while (offset < (size_t)ctx->slave_reg_count * 2) {
                    if (offset + 7 > (size_t)ctx->slave_reg_count * 2) break;
                    uint8_t  ref_type  = (uint8_t)(ctx->slave_reg_data[offset / 2] >> 8);
                    uint16_t file_num  = ctx->slave_reg_data[offset / 2 + 1];
                    uint16_t rec_num   = ctx->slave_reg_data[offset / 2 + 2];
                    uint16_t rec_len   = ctx->slave_reg_data[offset / 2 + 3];
                    (void)ref_type;
                    (void)file_num;

                    uint16_t data_regs[64];
                    if (rec_len > 64) rec_len = 64;
                    for (uint16_t i = 0; i < rec_len && (offset / 2 + 4 + i) < (size_t)ctx->slave_reg_count; i++)
                        data_regs[i] = ctx->slave_reg_data[offset / 2 + 4 + i];

                    ret = cb(rec_num, rec_len, data_regs, ctx->write_multiple_registers_userdata);
                    if (ret != 0) return PA_ERR_CALLBACK;

                    offset += 7 + (size_t)rec_len * 2;
                }

                /* Echo the original request as response */
                uint8_t req_byte_count = (uint8_t)(ctx->slave_reg_count * 2);
                ctx->txbuf[0] = fc;
                ctx->txbuf[1] = req_byte_count;
                for (uint16_t i = 0; i < ctx->slave_reg_count; i++)
                    put16(ctx->txbuf + 2 + i * 2, ctx->slave_reg_data[i]);
                pdu_len = 2 + (int)req_byte_count;
            }
            break;

        case 0x16: /* Mask Write Register */
            {
                pa_write_single_register_fn cb = ctx->write_single_register_cb;
                if (!cb) return PA_ERR_STATE;

                /* Read current value, apply mask, write back */
                pa_read_holding_registers_fn rcb = ctx->read_holding_registers_cb;
                if (rcb) {
                    uint16_t cur_val;
                    ret = rcb(ctx->slave_addr, 1, &cur_val, ctx->read_holding_registers_userdata);
                    if (ret == 0) {
                        uint16_t and_mask = ctx->slave_reg_data[0];
                        uint16_t or_mask  = ctx->slave_reg_data[1];
                        uint16_t new_val  = (cur_val & and_mask) | or_mask;
                        ret = cb(ctx->slave_addr, new_val, ctx->write_single_register_userdata);
                        if (ret != 0) return PA_ERR_CALLBACK;
                    }
                }

                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, ctx->slave_addr);
                put16(ctx->txbuf + 3, ctx->slave_reg_data[0]); /* AND mask */
                put16(ctx->txbuf + 5, ctx->slave_reg_data[1]); /* OR mask */
                pdu_len = 7;
            }
            break;

        case 0x17: /* Read/Write Multiple Registers */
            {
                pa_read_holding_registers_fn rcb = ctx->read_holding_registers_cb;
                pa_write_multiple_registers_fn wcb = ctx->write_multiple_registers_cb;
                if (!rcb || !wcb) return PA_ERR_STATE;

                /* Extract write info from slave_coil_data */
                uint16_t write_addr  = (uint16_t)(((uint16_t)ctx->slave_coil_data[0] << 8) | ctx->slave_coil_data[1]);
                uint16_t write_count = (uint16_t)(((uint16_t)ctx->slave_coil_data[2] << 8) | ctx->slave_coil_data[3]);

                /* First write */
                ret = wcb(write_addr, write_count, ctx->slave_reg_data, ctx->write_multiple_registers_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                /* Then read */
                uint16_t read_count = ctx->slave_count;
                uint16_t byte_count = (uint16_t)(read_count * 2);
                uint16_t reg_data[128];
                ret = rcb(ctx->slave_addr, read_count, reg_data, ctx->read_holding_registers_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                ctx->txbuf[0] = fc;
                ctx->txbuf[1] = (uint8_t)byte_count;
                for (uint16_t i = 0; i < read_count; i++)
                    put16(ctx->txbuf + 2 + i * 2, reg_data[i]);
                pdu_len = 2 + (int)byte_count;
            }
            break;

        case 0x18: /* Read FIFO Queue */
            {
                pa_read_holding_registers_fn cb = ctx->read_holding_registers_cb;
                if (!cb) return PA_ERR_STATE;

                uint16_t fifo_data[128];
                ret = cb(ctx->slave_addr, 128, fifo_data, ctx->read_holding_registers_userdata);
                if (ret != 0) return PA_ERR_CALLBACK;

                /* First register is the FIFO count */
                uint16_t fifo_count = fifo_data[0];
                if (fifo_count > 127) fifo_count = 127;
                uint16_t byte_count = (uint16_t)(fifo_count * 2 + 2); /* +2 for count field */

                ctx->txbuf[0] = fc;
                put16(ctx->txbuf + 1, byte_count);
                put16(ctx->txbuf + 3, fifo_count);
                for (uint16_t i = 0; i < fifo_count; i++)
                    put16(ctx->txbuf + 5 + i * 2, fifo_data[i + 1]);
                pdu_len = 2 + (int)byte_count;
            }
            break;

        default:
            return PA_ERR_PROTOCOL;
    }

    if (pdu_len < 0) return pdu_len;

    int framed;
    ret = ctx->framer->wrap(ctx, pdu_len, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

int pa_modbus_slave_respond_error(pa_modbus_t *ctx, uint8_t exception_code)
{
    ctx->txbuf[0] = (uint8_t)(ctx->last_function | 0x80);
    ctx->txbuf[1] = exception_code;

    int framed;
    int ret = ctx->framer->wrap(ctx, 2, &framed);
    if (ret == PA_OK) ctx->tx_len = (size_t)framed;
    return ret == PA_OK ? framed : ret;
}

/* ---------------------------------------------------------------------------
 * Public feed functions — dispatch through framer then parser
 * ------------------------------------------------------------------------- */

int pa_modbus_master_feed(pa_modbus_t *ctx, const uint8_t *data, size_t len)
{
    const uint8_t *pdu;
    size_t pdu_len;
    int ret = ctx->framer->unwrap(ctx, data, len, &pdu, &pdu_len);
    if (ret < 0) {
        ctx->last_error = (pa_error_t)ret;
        return ret;
    }
    if (ret > 0)
        return ret; /* Still waiting for more data */

    return parse_master_response(ctx, pdu, pdu_len);
}

int pa_modbus_slave_feed(pa_modbus_t *ctx, const uint8_t *data, size_t len)
{
    const uint8_t *pdu;
    size_t pdu_len;
    int ret = ctx->framer->unwrap(ctx, data, len, &pdu, &pdu_len);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return ret; /* Still waiting for more data */

    return parse_slave_request(ctx, pdu, pdu_len);
}