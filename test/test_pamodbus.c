/*
 * pamodbus — unit tests
 * MIT License — see LICENSE file for details.
 *
 * Basic tests for CRC-16, PDU build/parse, and RTU/TCP framing.
 */

#include "pamodbus.h"
#include "pamodbus_internal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * Test helpers
 * ------------------------------------------------------------------------- */

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do {                                 \
    if (!(cond)) {                                                  \
        printf("  FAIL: %s (%s)\n", msg, #cond);                    \
        tests_failed++;                                             \
    } else {                                                        \
        printf("  PASS: %s\n", msg);                                \
        tests_passed++;                                             \
    }                                                               \
} while(0)

/* ---------------------------------------------------------------------------
 * Test: CRC-16
 * ------------------------------------------------------------------------- */

static void test_crc16(void)
{
    printf("\n=== CRC-16 Tests ===\n");

    /* Known MODBUS CRC values verified against standard MODBUS CRC-16:
     * For data {0x01, 0x03, 0x00, 0x00, 0x00, 0x01} CRC = 0x840A
     * For data {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03} CRC = 0x7687
     */
    {
        uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
        uint16_t crc = pa_crc16(data, sizeof(data));
        /* Expected CRC-16 MODBUS = 0x0A84 (low byte 0x84, high byte 0x0A) */
        uint16_t expected = 0x0A84;
        TEST_ASSERT(crc == expected, "CRC for read holding regs request");
    }

    {
        uint8_t data[] = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};
        uint16_t crc = pa_crc16(data, sizeof(data));
        /* Expected CRC-16 MODBUS = 0x8776 */
        uint16_t expected = 0x8776;
        TEST_ASSERT(crc == expected, "CRC for another known frame");
    }

    /* Empty data should give 0xFFFF */
    {
        uint16_t crc = pa_crc16(NULL, 0);
        TEST_ASSERT(crc == 0xFFFF, "CRC of empty data is 0xFFFF");
    }
}

/* ---------------------------------------------------------------------------
 * Test: PDU build (master requests)
 * ------------------------------------------------------------------------- */

static void test_pdu_build(void)
{
    printf("\n=== PDU Build Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];

    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_slave(&mb, 0x01);

    /* Test: Read Holding Registers (FC 03) */
    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 3);
        TEST_ASSERT(len > 0, "build_read_holding_registers returns positive length");
        TEST_ASSERT((size_t)len == pa_modbus_tx_len(&mb), "tx_len matches build return");

        /* RTU frame: slave(1) + fc(1) + addr(2) + count(2) + crc(2) = 8 */
        TEST_ASSERT(len == 8, "RTU frame length is 8 bytes");

        /* Verify PDU content */
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[0] == 0x01, "Slave address = 0x01");
        TEST_ASSERT(buf[1] == 0x03, "Function code = 0x03");
        TEST_ASSERT(buf[2] == 0x00, "Address hi = 0x00");
        TEST_ASSERT(buf[3] == 0x00, "Address lo = 0x00");
        TEST_ASSERT(buf[4] == 0x00, "Count hi = 0x00");
        TEST_ASSERT(buf[5] == 0x03, "Count lo = 0x03");

        /* Verify CRC */
        uint16_t crc_calc = pa_crc16(buf, 6);
        uint16_t crc_frame = (uint16_t)(buf[6] | ((uint16_t)buf[7] << 8));
        TEST_ASSERT(crc_calc == crc_frame, "CRC is correct");
    }

    /* Test: Write Single Coil (FC 05) */
    {
        int len = pa_modbus_build_write_single_coil(&mb, 10, 1);
        TEST_ASSERT(len == 8, "write_single_coil frame length is 8");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[0] == 0x01, "Slave = 0x01");
        TEST_ASSERT(buf[1] == 0x05, "FC = 0x05");
        TEST_ASSERT(buf[2] == 0x00, "Addr hi = 0x00");
        TEST_ASSERT(buf[3] == 0x0A, "Addr lo = 0x0A");
        TEST_ASSERT(buf[4] == 0xFF, "Value hi = 0xFF (ON)");
        TEST_ASSERT(buf[5] == 0x00, "Value lo = 0x00");
    }

    /* Test: Write Single Register (FC 06) */
    {
        int len = pa_modbus_build_write_single_register(&mb, 5, 0x1234);
        TEST_ASSERT(len == 8, "write_single_register frame length is 8");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x06, "FC = 0x06");
        TEST_ASSERT(buf[4] == 0x12, "Value hi = 0x12");
        TEST_ASSERT(buf[5] == 0x34, "Value lo = 0x34");
    }

    /* Test: Write Multiple Registers (FC 10) */
    {
        uint16_t values[] = {0x0001, 0x0002, 0x0003};
        int len = pa_modbus_build_write_multiple_registers(&mb, 0, values, 3);
        /* PDU: fc(1) + addr(2) + count(2) + byte_count(1) + data(6) = 12
         * RTU: + slave(1) + crc(2) = 15
         * Frame layout:
         *   [0]=slave, [1]=FC, [2..3]=addr, [4..5]=count, [6]=byte_count,
         *   [7..8]=val[0], [9..10]=val[1], [11..12]=val[2], [13..14]=CRC */
        TEST_ASSERT(len == 15, "write_multiple_registers frame length is 15");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x10, "FC = 0x10");
        TEST_ASSERT(buf[6] == 0x06, "Byte count = 6");
        TEST_ASSERT(buf[7] == 0x00, "Value[0] hi = 0x00");
        TEST_ASSERT(buf[8] == 0x01, "Value[0] lo = 0x01");
        TEST_ASSERT(buf[11] == 0x00, "Value[2] hi = 0x00");
        TEST_ASSERT(buf[12] == 0x03, "Value[2] lo = 0x03");
    }

    /* Test: Invalid parameter returns error */
    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 0);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "count=0 returns PA_ERR_BAD_PARAM");
    }

    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 200);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "count=200 returns PA_ERR_BAD_PARAM (>125)");
    }
}

/* ---------------------------------------------------------------------------
 * Test: TCP framing
 * ------------------------------------------------------------------------- */

static void test_tcp_framing(void)
{
    printf("\n=== TCP Framing Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];

    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_TCP);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_slave(&mb, 0x01);

    /* Build a request and verify TCP framing */
    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 3);
        /* MBAP(7) + PDU(5) = 12 */
        TEST_ASSERT(len == 12, "TCP frame length is 12 bytes");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        /* MBAP: trans_id(2) + proto_id(2) + length(2) + unit_id(1) */
        TEST_ASSERT(buf[2] == 0x00, "Protocol ID hi = 0x00");
        TEST_ASSERT(buf[3] == 0x00, "Protocol ID lo = 0x00");
        /* Length = PDU(5) + unit_id(1) = 6 */
        TEST_ASSERT(buf[4] == 0x00, "Length hi = 0x00");
        TEST_ASSERT(buf[5] == 0x06, "Length lo = 0x06");
        TEST_ASSERT(buf[6] == 0x01, "Unit ID = 0x01");
        /* PDU starts at offset 7 */
        TEST_ASSERT(buf[7] == 0x03, "FC = 0x03 at PDU offset 0");
    }

    /* Test master_feed with a valid TCP response */
    {
        /* Build a simulated TCP response frame:
         * MBAP: trans_id(2) + proto_id(2) + length(2) + unit_id(1)
         * PDU:  fc(1) + byte_count(1) + data(6)
         * length = PDU(1+1+6) + unit_id(1) = 9
         */
        uint8_t resp[16];
        resp[0] = 0x00; resp[1] = 0x01;  /* Transaction ID */
        resp[2] = 0x00; resp[3] = 0x00;  /* Protocol ID */
        resp[4] = 0x00; resp[5] = 0x09;  /* Length = 9 */
        resp[6] = 0x01;                   /* Unit ID */
        resp[7] = 0x03;                   /* FC 03 */
        resp[8] = 0x06;                   /* Byte count = 6 */
        resp[9] = 0x00; resp[10] = 0x01;  /* Register 0 = 1 */
        resp[11] = 0x00; resp[12] = 0x02; /* Register 1 = 2 */
        resp[13] = 0x00; resp[14] = 0x03; /* Register 2 = 3 */

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_OK, "master_feed returns PA_OK for valid TCP response");

        uint16_t r0 = pa_modbus_get_register(&mb, 0);
        uint16_t r1 = pa_modbus_get_register(&mb, 1);
        uint16_t r2 = pa_modbus_get_register(&mb, 2);
        TEST_ASSERT(r0 == 1, "Register 0 = 1");
        TEST_ASSERT(r1 == 2, "Register 1 = 2");
        TEST_ASSERT(r2 == 3, "Register 2 = 3");
    }

    /* Test: TCP response with wrong unit ID */
    {
        uint8_t resp[16];
        resp[0] = 0x00; resp[1] = 0x02;
        resp[2] = 0x00; resp[3] = 0x00;
        resp[4] = 0x00; resp[5] = 0x09;
        resp[6] = 0x02;                   /* Wrong unit ID */
        resp[7] = 0x03;
        resp[8] = 0x06;
        memset(resp + 9, 0, 6);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_ERR_INVALID_SLAVE, "Wrong unit ID returns PA_ERR_INVALID_SLAVE");
    }
}

/* ---------------------------------------------------------------------------
 * Test: RTU framing with master_feed
 * ------------------------------------------------------------------------- */

static void test_rtu_master_feed(void)
{
    printf("\n=== RTU Master Feed Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];

    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_slave(&mb, 0x01);

    /* Build a request first (to set the slave address context) */
    pa_modbus_build_read_holding_registers(&mb, 0, 3);

    /* Simulate a valid RTU response:
     * slave(1) + fc(1) + byte_count(1) + data(6) + crc(2)
     * CRC over: {0x01, 0x03, 0x06, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03}
     */
    uint8_t resp[] = {
        0x01,                         /* Slave address */
        0x03,                         /* FC 03 */
        0x06,                         /* Byte count = 6 */
        0x00, 0x01,                   /* Register 0 = 1 */
        0x00, 0x02,                   /* Register 1 = 2 */
        0x00, 0x03,                   /* Register 2 = 3 */
        0x00, 0x00                    /* CRC placeholder */
    };

    /* Compute CRC */
    uint16_t crc = pa_crc16(resp, 9);
    resp[9]  = (uint8_t)(crc & 0xFF);
    resp[10] = (uint8_t)((crc >> 8) & 0xFF);

    int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
    TEST_ASSERT(ret == PA_OK, "master_feed returns PA_OK for valid RTU response");

    uint16_t r0 = pa_modbus_get_register(&mb, 0);
    uint16_t r1 = pa_modbus_get_register(&mb, 1);
    uint16_t r2 = pa_modbus_get_register(&mb, 2);
    TEST_ASSERT(r0 == 1, "Register 0 = 1");
    TEST_ASSERT(r1 == 2, "Register 1 = 2");
    TEST_ASSERT(r2 == 3, "Register 2 = 3");

    /* Test: CRC mismatch */
    {
        uint8_t bad_resp[11];
        memcpy(bad_resp, resp, sizeof(bad_resp));
        bad_resp[9] ^= 0xFF; /* Corrupt CRC */

        int ret2 = pa_modbus_master_feed(&mb, bad_resp, sizeof(bad_resp));
        TEST_ASSERT(ret2 == PA_ERR_CRC, "Corrupt CRC returns PA_ERR_CRC");
    }

    /* Test: Wrong slave address */
    {
        uint8_t wrong_slave[11];
        memcpy(wrong_slave, resp, sizeof(wrong_slave));
        wrong_slave[0] = 0x02; /* Wrong slave */
        /* Recompute CRC */
        uint16_t crc2 = pa_crc16(wrong_slave, 9);
        wrong_slave[9]  = (uint8_t)(crc2 & 0xFF);
        wrong_slave[10] = (uint8_t)((crc2 >> 8) & 0xFF);

        int ret2 = pa_modbus_master_feed(&mb, wrong_slave, sizeof(wrong_slave));
        TEST_ASSERT(ret2 == PA_ERR_INVALID_SLAVE, "Wrong slave returns PA_ERR_INVALID_SLAVE");
    }

    /* Test: Incomplete frame (need more data) */
    {
        uint8_t partial[] = {0x01, 0x03, 0x06}; /* Only 3 bytes */
        int ret2 = pa_modbus_master_feed(&mb, partial, sizeof(partial));
        TEST_ASSERT(ret2 > 0, "Partial frame returns >0 (need more data)");
    }
}

/* ---------------------------------------------------------------------------
 * Test: Slave mode
 * ------------------------------------------------------------------------- */

static uint16_t test_holding_regs[100];

static int test_read_holding_cb(uint16_t addr, uint16_t count, uint16_t *values, void *userdata)
{
    (void)userdata;
    if (addr + count > 100) return -1;
    for (uint16_t i = 0; i < count; i++)
        values[i] = test_holding_regs[addr + i];
    return 0;
}

static int test_write_holding_cb(uint16_t addr, uint16_t count, const uint16_t *values, void *userdata)
{
    (void)userdata;
    if (addr + count > 100) return -1;
    for (uint16_t i = 0; i < count; i++)
        test_holding_regs[addr + i] = values[i];
    return 0;
}

static void test_slave_mode(void)
{
    printf("\n=== Slave Mode Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];

    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_slave(&mb, 0x01);

    pa_modbus_set_read_holding_registers_cb(&mb, test_read_holding_cb, NULL);
    pa_modbus_set_write_multiple_registers_cb(&mb, test_write_holding_cb, NULL);

    /* Initialize test registers */
    for (int i = 0; i < 100; i++)
        test_holding_regs[i] = (uint16_t)(i * 10);

    /* Simulate a read request: slave(1) + fc(1) + addr(2) + count(2) + crc(2) */
    {
        uint8_t req[] = {
            0x01,                         /* Slave */
            0x03,                         /* FC 03 */
            0x00, 0x00,                   /* Addr = 0 */
            0x00, 0x03,                   /* Count = 3 */
            0x00, 0x00                    /* CRC */
        };
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "slave_feed returns PA_OK for read request");

        uint8_t fc = pa_modbus_slave_function(&mb);
        uint16_t addr = pa_modbus_slave_addr(&mb);
        uint16_t count = pa_modbus_slave_count(&mb);
        TEST_ASSERT(fc == 0x03, "Function code = 0x03");
        TEST_ASSERT(addr == 0, "Address = 0");
        TEST_ASSERT(count == 3, "Count = 3");

        /* Build response */
        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len > 0, "slave_respond returns positive length");

        /* Verify response: slave(1) + fc(1) + byte_count(1) + data(6) + crc(2) = 11 */
        TEST_ASSERT(len == 11, "Response length is 11");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[0] == 0x01, "Response slave = 0x01");
        TEST_ASSERT(buf[1] == 0x03, "Response FC = 0x03");
        TEST_ASSERT(buf[2] == 0x06, "Byte count = 6");
        TEST_ASSERT(buf[3] == 0x00 && buf[4] == 0x00, "Reg[0] = 0");
        TEST_ASSERT(buf[5] == 0x00 && buf[6] == 0x0A, "Reg[1] = 10");
        TEST_ASSERT(buf[7] == 0x00 && buf[8] == 0x14, "Reg[2] = 20");
    }

    /* Simulate a write request */
    {
        uint8_t req[] = {
            0x01,                         /* Slave */
            0x10,                         /* FC 10 */
            0x00, 0x0A,                   /* Addr = 10 */
            0x00, 0x02,                   /* Count = 2 */
            0x04,                         /* Byte count = 4 */
            0x00, 0x64,                   /* Value[0] = 100 */
            0x00, 0xC8,                   /* Value[1] = 200 */
            0x00, 0x00                    /* CRC */
        };
        uint16_t crc = pa_crc16(req, 11);
        req[11] = (uint8_t)(crc & 0xFF);
        req[12] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "slave_feed returns PA_OK for write request");

        /* Build response */
        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len > 0, "slave_respond returns positive length");

        /* Verify registers were written */
        TEST_ASSERT(test_holding_regs[10] == 100, "Holding reg[10] = 100");
        TEST_ASSERT(test_holding_regs[11] == 200, "Holding reg[11] = 200");

        /* Response should be echo: slave(1) + fc(1) + addr(2) + count(2) + crc(2) = 8 */
        TEST_ASSERT(len == 8, "Write response length is 8");
    }

    /* Test: Exception response */
    {
        /* Feed a request for an unsupported function code (e.g., FC 0x07) */
        uint8_t req[] = {
            0x01, 0x07, 0x00, 0x00, 0x00, 0x00
        };
        uint16_t crc = pa_crc16(req, 6);
        uint8_t full_req[8];
        memcpy(full_req, req, 6);
        full_req[6] = (uint8_t)(crc & 0xFF);
        full_req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, full_req, sizeof(full_req));
        /* FC 07 is valid but we haven't registered a callback for it,
         * so slave_respond will return PA_ERR_STATE */
        if (ret == PA_OK) {
            int len = pa_modbus_slave_respond(&mb);
            TEST_ASSERT(len == PA_ERR_STATE, "slave_respond returns PA_ERR_STATE for unregistered FC 07");
        }

        /* Test explicit exception response */
        pa_modbus_slave_feed(&mb, full_req, sizeof(full_req));
        int len = pa_modbus_slave_respond_error(&mb, PA_EX_ILLEGAL_FUNCTION);
        TEST_ASSERT(len > 0, "slave_respond_error returns positive length");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == (0x07 | 0x80), "Exception FC has MSB set");
        TEST_ASSERT(buf[2] == 0x01, "Exception code = ILLEGAL_FUNCTION");
    }
}

/* ---------------------------------------------------------------------------
 * Test: Coil read/write
 * ------------------------------------------------------------------------- */

static uint8_t test_coils[32]; /* 256 bits */

static int test_read_coils_cb(uint16_t addr, uint16_t count, uint8_t *values, void *userdata)
{
    (void)userdata;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t a = addr + i;
        values[i / 8] &= ~(uint8_t)(1 << (i % 8));
        if (test_coils[a / 8] & (1 << (a % 8)))
            values[i / 8] |= (uint8_t)(1 << (i % 8));
    }
    return 0;
}

static int test_write_single_coil_cb(uint16_t addr, uint8_t value, void *userdata)
{
    (void)userdata;
    if (addr >= 256) return -1;
    if (value)
        test_coils[addr / 8] |= (uint8_t)(1 << (addr % 8));
    else
        test_coils[addr / 8] &= ~(uint8_t)(1 << (addr % 8));
    return 0;
}

static void test_coil_operations(void)
{
    printf("\n=== Coil Operation Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];

    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_slave(&mb, 0x01);

    pa_modbus_set_read_coils_cb(&mb, test_read_coils_cb, NULL);
    pa_modbus_set_write_single_coil_cb(&mb, test_write_single_coil_cb, NULL);

    memset(test_coils, 0, sizeof(test_coils));

    /* Write coil 5 = ON */
    {
        uint8_t req[] = {0x01, 0x05, 0x00, 0x05, 0xFF, 0x00, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Write coil slave_feed OK");

        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len > 0, "Write coil response OK");
        TEST_ASSERT(test_coils[0] & (1 << 5), "Coil 5 is ON");
    }

    /* Read coils 0-7 */
    {
        uint8_t req[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Read coils slave_feed OK");

        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len > 0, "Read coils response OK");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        /* Byte at offset 3 (after slave+fc+byte_count) should have bit 5 set */
        TEST_ASSERT(buf[3] & (1 << 5), "Coil 5 is set in response");
        TEST_ASSERT(!(buf[3] & (1 << 0)), "Coil 0 is clear in response");
    }
}

/* ---------------------------------------------------------------------------
 * Main
 * ------------------------------------------------------------------------- */

int main(void)
{
    printf("pamodbus Unit Tests\n");
    printf("==================\n");

    test_crc16();
    test_pdu_build();
    test_tcp_framing();
    test_rtu_master_feed();
    test_slave_mode();
    test_coil_operations();

    printf("\n==================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}