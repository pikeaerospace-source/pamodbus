/*
 * pamodbus — TCP framer
 * MIT License — see LICENSE file for details.
 *
 * TCP framing: [MBAP Header: 7B] [PDU]
 * MBAP: Transaction ID(2) + Protocol ID(2) + Length(2) + Unit ID(1)
 */

#include "pamodbus_internal.h"
#include <string.h>

/* Simple transaction ID counter */
static uint16_t next_transaction_id = 1;

int pa_framer_tcp_overhead(void)
{
    return 7; /* MBAP header: 2+2+2+1 */
}

int pa_framer_tcp_wrap(pa_modbus_t *ctx, int pdu_len, int *framed_len)
{
    /* Check buffer space: MBAP(7) + pdu */
    if ((size_t)(7 + pdu_len) > ctx->txbuf_size)
        return PA_ERR_BUFFER_FULL;

    /* Shift PDU right by 7 to make room for MBAP header at the front */
    memmove(ctx->txbuf + 7, ctx->txbuf, (size_t)pdu_len);

    /* MBAP Header */
    uint16_t trans_id = next_transaction_id++;
    uint16_t proto_id = 0;
    uint16_t length   = (uint16_t)(pdu_len + 1); /* +1 for unit ID */

    ctx->txbuf[0] = (uint8_t)((trans_id >> 8) & 0xFF);  /* Transaction ID hi */
    ctx->txbuf[1] = (uint8_t)(trans_id & 0xFF);          /* Transaction ID lo */
    ctx->txbuf[2] = (uint8_t)((proto_id >> 8) & 0xFF);   /* Protocol ID hi */
    ctx->txbuf[3] = (uint8_t)(proto_id & 0xFF);           /* Protocol ID lo */
    ctx->txbuf[4] = (uint8_t)((length >> 8) & 0xFF);     /* Length hi */
    ctx->txbuf[5] = (uint8_t)(length & 0xFF);             /* Length lo */
    ctx->txbuf[6] = ctx->slave;                           /* Unit ID */

    *framed_len = 7 + pdu_len;
    return PA_OK;
}

int pa_framer_tcp_unwrap(pa_modbus_t *ctx, const uint8_t *data, size_t len,
                         const uint8_t **pdu, size_t *pdu_len)
{
    (void)ctx;

    /* Minimum: MBAP(7) + FC(1) = 8 bytes */
    if (len < 8)
        return (int)(8 - len);

    /* Parse MBAP header */
    uint16_t length = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);

    /* Length field includes the unit ID byte, so PDU length = length - 1 */
    size_t total_frame = (size_t)(7 + length); /* MBAP(7) + length field value */

    if (len < total_frame)
        return (int)(total_frame - len);

    /* Verify protocol ID is 0 (MODBUS) */
    uint16_t proto_id = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
    if (proto_id != 0)
        return PA_ERR_PROTOCOL;

    /* Verify unit ID matches (only if not broadcast 0xFF) */
    if (ctx->slave != 0xFF && data[6] != ctx->slave)
        return PA_ERR_INVALID_SLAVE;

    /* Return PDU (after MBAP header) */
    *pdu = data + 7;
    *pdu_len = (size_t)(length - 1); /* length includes unit ID, subtract it */
    return PA_OK;
}

const pa_framer_ops_t pa_framer_tcp_ops = {
    pa_framer_tcp_overhead,
    pa_framer_tcp_wrap,
    pa_framer_tcp_unwrap,
};