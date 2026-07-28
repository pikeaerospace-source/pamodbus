/*
 * pamodbus — RTU framer
 * MIT License — see LICENSE file for details.
 *
 * RTU framing: [Slave Addr: 1B] [PDU] [CRC-16: 2B]
 */

#include "pamodbus_internal.h"
#include <string.h>

int pa_framer_rtu_overhead(void)
{
    return 3; /* 1 byte slave addr + 2 bytes CRC */
}

int pa_framer_rtu_wrap(pa_modbus_t *ctx, int pdu_len, int *framed_len)
{
    /* Check buffer space: slave(1) + pdu + crc(2) */
    if ((size_t)(1 + pdu_len + 2) > ctx->txbuf_size)
        return PA_ERR_BUFFER_FULL;

    /* Shift PDU right by 1 to make room for slave addr at the front */
    memmove(ctx->txbuf + 1, ctx->txbuf, (size_t)pdu_len);

    /* Prepend slave address */
    ctx->txbuf[0] = ctx->slave;

    /* Append CRC-16 (little-endian) */
    size_t frame_len = (size_t)(1 + pdu_len);
    uint16_t crc = pa_crc16(ctx->txbuf, frame_len);
    ctx->txbuf[frame_len]     = (uint8_t)(crc & 0xFF);
    ctx->txbuf[frame_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    *framed_len = (int)(frame_len + 2);
    return PA_OK;
}

int pa_framer_rtu_unwrap(pa_modbus_t *ctx, const uint8_t *data, size_t len,
                         const uint8_t **pdu, size_t *pdu_len)
{
    (void)ctx;

    /* Minimum frame: slave(1) + fc(1) + crc(2) = 4 bytes */
    if (len < 4) {
        /* Need more bytes; return how many more */
        return (int)(4 - len);
    }

    /* Minimum PDU is at least 1 byte (function code) */
    size_t min_frame = 4; /* slave + fc + crc */
    size_t pdu_start = 1; /* after slave addr */

    /* The frame length depends on function code and PDU content.
     * For simplicity in the unframe step, we need to know the total
     * frame length. We read the function code to determine:
     *
     * FC 01, 02, 03, 04: request = 5 bytes (slave + fc + addr(2) + count(2) + crc)
     *    But response depends on byte count.
     * FC 05, 06:          always 8 bytes (slave + fc + addr(2) + value(2) + crc)
     * FC 0F, 10:          request varies (slave + fc + addr(2) + count(2) + byte_count + data + crc)
     *
     * We cannot determine frame length until we have the full frame.
     * We'll accumulate in ctx->rxbuf and validate upon receiving data via feed.
     * The unwrap here simply strips the framing if we already have a complete frame.
     *
     * Since the feed function accumulates in rxbuf and passes accumulated data,
     * we check for minimum length and CRC validity when a plausible frame is available.
     */

    /* For the unwrap to work, we need at least 4 bytes */
    uint8_t fc = data[pdu_start];

    /* Determine minimum expected frame length based on function code */
    size_t expected_len = 0;

    switch (fc) {
        /* Read responses (FC 01, 02): slave(1) + fc(1) + byte_count(1) + data(N) + crc(2)
         * Read responses (FC 03, 04): slave(1) + fc(1) + byte_count(1) + data(N) + crc(2)
         * Read requests:              slave(1) + fc(1) + addr(2) + count(2) + crc(2) = 8
         *
         * Responses have byte_count at data[2] (after slave+fc)
         * Requests have addr(2)+count(2) at data[1..4]
         * Distinction: in a response, data[2] is byte_count which is ≤ 250.
         *              in a request, data[1..2] is address and data[3..4] is count.
         *              We can't reliably distinguish. Use the minimum (8) and then
         *              if len >= 8 and data[2] (byte_count) makes sense, use that.
         */
        case 0x01: case 0x02: case 0x03: case 0x04:
            /* Minimum 8 bytes (request format). If we have more, check if it's a response
             * with byte_count that would extend the frame. */
            if (len >= 4) {
                uint8_t bc = data[2]; /* potential byte count field */
                if (bc <= 250 && bc > 0) {
                    /* Looks like a response: slave(1)+fc(1)+bc(1)+data(bc)+crc(2) */
                    expected_len = (size_t)(3 + bc + 2);
                } else {
                    /* Looks like a request or response with no data */
                    expected_len = 8;
                }
            } else {
                expected_len = 8;
            }
            break;

        /* Write single: slave(1) + fc(1) + addr(2) + value(2) + crc(2) = 8 */
        case 0x05: case 0x06:
            expected_len = 8;
            break;

        /* Write multiple request: slave(1) + fc(1) + addr(2) + count(2) + byte_count(1) + data + crc(2)
         * Fixed header = slave(1)+fc(1)+addr(2)+count(2)+byte_count(1) = 7 bytes */
        case 0x0F: case 0x10:
            if (len < 8) return (int)(8 - len);
            /* byte_count is at data[6] (after slave+fc+addr+count) */
            expected_len = (size_t)(7 + data[6] + 2);
            break;

        /* Diagnostic (FC 08): slave(1) + fc(1) + sub(2) + data(2) + crc(2) = 8 */
        case 0x07: case 0x0B: case 0x0C: case 0x08:
            expected_len = 8;
            break;

        /* Report Server ID (FC 11): response variable; request = 4 bytes (slave+fc+crc?) 
         * Actually FC 11 request is just slave + fc + crc = 4 bytes */
        case 0x11:
            expected_len = 4;
            break;

        /* FC 14, 15: Read/Write File Record — more complex. For now require at least 8 */
        case 0x14: case 0x15:
            if (len < 8) return (int)(8 - len);
            expected_len = len; /* Accept what we have for now, actual validation in PDU */
            break;

        /* FC 16: Mask Write Register — slave(1) + fc(1) + addr(2) + and(2) + or(2) + crc(2) = 10 */
        case 0x16:
            expected_len = 10;
            break;

        /* FC 17: Read/Write Multiple Registers
         * Format: slave(1)+fc(1)+read_addr(2)+read_count(2)+write_addr(2)+write_count(2)+byte_count(1)+data+crc(2)
         * Header before data = 10, byte_count at data[9], total = 10 + byte_count + 2 CRC */
        case 0x17:
            if (len < 11) return (int)(11 - len);
            /* byte_count is at data[9] (offset: slave=0, fc=1, read_addr=2-3, read_count=4-5, write_addr=6-7, write_count=8, byte_count=9) */
            expected_len = (size_t)(10 + data[9] + 2);
            break;

        /* FC 18: Read FIFO Queue — slave(1) + fc(1) + fifo_addr(2) + crc(2) = 6 */
        case 0x18:
            expected_len = 6;
            break;

        /* For responses, the FC may have 0x80 set (exception). */
        default:
            if ((fc & 0x80) && (fc & 0x7F) >= 0x01 && (fc & 0x7F) <= 0x18) {
                /* Exception response: slave(1) + 0x80|fc(1) + exc(1) + crc(2) = 5 */
                expected_len = 5;
            } else {
                /* Unknown function code, need more data */
                return PA_ERR_PROTOCOL;
            }
            break;
    }

    if (len < expected_len) {
        return (int)(expected_len - len);
    }

    /* Verify CRC */
    uint16_t crc_received = (uint16_t)(data[expected_len - 2] |
                                       ((uint16_t)data[expected_len - 1] << 8));
    uint16_t crc_calc = pa_crc16(data, expected_len - 2);

    if (crc_calc != crc_received) {
        return PA_ERR_CRC;
    }

    /* Verify slave address matches (only if not broadcast 0xFF) */
    if (ctx->slave != 0xFF && data[0] != ctx->slave) {
        return PA_ERR_INVALID_SLAVE;
    }

    /* Return PDU (after slave addr, before CRC) */
    *pdu = data + pdu_start;
    *pdu_len = expected_len - pdu_start - 2; /* subtract slave and CRC */
    return PA_OK;
}

const pa_framer_ops_t pa_framer_rtu_ops = {
    pa_framer_rtu_overhead,
    pa_framer_rtu_wrap,
    pa_framer_rtu_unwrap,
};