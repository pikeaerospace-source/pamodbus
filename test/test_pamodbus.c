/*
 * pamodbus — unit tests
 * MIT License — see LICENSE file for details.
 *
 * Tests for CRC-16, PDU build/parse, RTU/TCP framing, slave mode,
 * userdata isolation, chunked receives, broadcast addresses, and reentrancy.
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

    {
        uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
        uint16_t crc = pa_crc16(data, sizeof(data));
        uint16_t expected = 0x0A84;
        TEST_ASSERT(crc == expected, "CRC for read holding regs request");
    }

    {
        uint8_t data[] = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};
        uint16_t crc = pa_crc16(data, sizeof(data));
        uint16_t expected = 0x8776;
        TEST_ASSERT(crc == expected, "CRC for another known frame");
    }

    /* Empty data should give 0xFFFF */
    {
        uint16_t crc = pa_crc16(NULL, 0);
        TEST_ASSERT(crc == 0xFFFF, "CRC of empty data is 0xFFFF");
    }

    /* CRC of single byte */
    {
        uint8_t data[] = {0x00};
        uint16_t crc = pa_crc16(data, 1);
        TEST_ASSERT(crc == 0x40BF, "CRC of single zero byte");
    }

    {
        uint8_t data[] = {0xFF};
        uint16_t crc = pa_crc16(data, 1);
        TEST_ASSERT(crc == 0x00FF, "CRC of single 0xFF byte");
    }
}

/* ---------------------------------------------------------------------------
 * Helper: set up a default MODBUS context for testing
 * ------------------------------------------------------------------------- */

static void setup_default(pa_modbus_t *mb, uint8_t *txbuf, size_t txsz,
                          uint8_t *rxbuf, size_t rxsz, uint8_t slave)
{
    pa_modbus_init(mb);
    pa_modbus_set_framer(mb, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(mb, txbuf, txsz);
    pa_modbus_set_rxbuf(mb, rxbuf, rxsz);
    pa_modbus_set_slave(mb, slave);
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
    setup_default(&mb, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf), 0x01);

    /* Read Holding Registers (FC 03) */
    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 3);
        TEST_ASSERT(len > 0, "build_read_holding_registers returns positive length");
        TEST_ASSERT((size_t)len == pa_modbus_tx_len(&mb), "tx_len matches build return");
        TEST_ASSERT(len == 8, "RTU frame length is 8 bytes");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[0] == 0x01, "Slave address = 0x01");
        TEST_ASSERT(buf[1] == 0x03, "Function code = 0x03");
        TEST_ASSERT(buf[5] == 0x03, "Count lo = 0x03");

        uint16_t crc_calc = pa_crc16(buf, 6);
        uint16_t crc_frame = (uint16_t)(buf[6] | ((uint16_t)buf[7] << 8));
        TEST_ASSERT(crc_calc == crc_frame, "CRC is correct");
    }

    /* Read Coils (FC 01) */
    {
        int len = pa_modbus_build_read_coils(&mb, 10, 8);
        TEST_ASSERT(len == 8, "FC01 RTU frame length is 8");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x01, "FC01 function code");
        TEST_ASSERT(buf[3] == 0x0A, "FC01 addr lo = 10");
        TEST_ASSERT(buf[5] == 0x08, "FC01 count = 8");
    }

    /* Read Discrete Inputs (FC 02) */
    {
        int len = pa_modbus_build_read_discrete_inputs(&mb, 0, 5);
        TEST_ASSERT(len == 8, "FC02 RTU frame length is 8");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x02, "FC02 function code");
    }

    /* Read Input Registers (FC 04) */
    {
        int len = pa_modbus_build_read_input_registers(&mb, 100, 10);
        TEST_ASSERT(len == 8, "FC04 RTU frame length is 8");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x04, "FC04 function code");
        TEST_ASSERT(buf[3] == 0x64, "FC04 addr lo = 100");
        TEST_ASSERT(buf[5] == 0x0A, "FC04 count = 10");
    }

    /* Write Single Coil ON (FC 05) */
    {
        int len = pa_modbus_build_write_single_coil(&mb, 10, 1);
        TEST_ASSERT(len == 8, "FC05 ON frame length is 8");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x05, "FC05 ON = 0x05");
        TEST_ASSERT(buf[4] == 0xFF, "FC05 ON value hi = 0xFF");
        TEST_ASSERT(buf[5] == 0x00, "FC05 ON value lo = 0x00");
    }

    /* Write Single Coil OFF (FC 05) */
    {
        int len = pa_modbus_build_write_single_coil(&mb, 42, 0);
        TEST_ASSERT(len == 8, "FC05 OFF frame length is 8");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x05, "FC05 OFF function code");
        TEST_ASSERT(buf[4] == 0x00, "FC05 OFF value hi = 0x00");
        TEST_ASSERT(buf[5] == 0x00, "FC05 OFF value lo = 0x00");
    }

    /* Write Single Register (FC 06) */
    {
        int len = pa_modbus_build_write_single_register(&mb, 5, 0x1234);
        TEST_ASSERT(len == 8, "FC06 frame length is 8");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x06, "FC06 function code");
        TEST_ASSERT(buf[4] == 0x12, "FC06 value hi = 0x12");
        TEST_ASSERT(buf[5] == 0x34, "FC06 value lo = 0x34");
    }

    /* Write Multiple Coils (FC 0F) */
    {
        uint8_t values[] = {1, 0, 1, 0, 1, 0, 1, 0};
        int len = pa_modbus_build_write_multiple_coils(&mb, 0, values, 8);
        TEST_ASSERT(len == 10, "FC0F RTU frame length is 10");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x0F, "FC0F function code");
        TEST_ASSERT(buf[6] == 0x01, "FC0F byte count = 1");
        TEST_ASSERT(buf[7] == 0x55, "FC0F coil data = 0x55 (01010101)");
    }

    /* Write Multiple Registers (FC 10) */
    {
        uint16_t values[] = {0x0001, 0x0002, 0x0003};
        int len = pa_modbus_build_write_multiple_registers(&mb, 0, values, 3);
        TEST_ASSERT(len == 15, "FC10 RTU frame length is 15");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == 0x10, "FC10 function code");
        TEST_ASSERT(buf[6] == 0x06, "FC10 byte count = 6");
        TEST_ASSERT(buf[7] == 0x00 && buf[8] == 0x01, "FC10 val[0] = 1");
        TEST_ASSERT(buf[11] == 0x00 && buf[12] == 0x03, "FC10 val[2] = 3");
    }

    /* Maximum valid parameters */
    {
        int len = pa_modbus_build_read_coils(&mb, 0, 2000);
        TEST_ASSERT(len == 8, "FC01 max count (2000) OK");
    }
    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 125);
        TEST_ASSERT(len == 8, "FC03 max count (125) OK");
    }

    /* Invalid parameters return error */
    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 0);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "FC03 count=0 returns PA_ERR_BAD_PARAM");
    }
    {
        int len = pa_modbus_build_read_holding_registers(&mb, 0, 200);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "FC03 count=200 returns PA_ERR_BAD_PARAM (>125)");
    }
    {
        int len = pa_modbus_build_read_coils(&mb, 0, 0);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "FC01 count=0 returns error");
    }
    {
        int len = pa_modbus_build_read_coils(&mb, 0, 2001);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "FC01 count=2001 returns error");
    }
    {
        int len = pa_modbus_build_read_input_registers(&mb, 0, 126);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "FC04 count=126 returns error");
    }
    {
        int len = pa_modbus_build_write_multiple_coils(&mb, 0, NULL, 0);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "FC0F count=0 returns error");
    }
    {
        int len = pa_modbus_build_write_multiple_registers(&mb, 0, NULL, 124);
        TEST_ASSERT(len == PA_ERR_BAD_PARAM, "FC10 count=124 returns error");
    }

    /* Buffer overflow with tiny TX buffer */
    {
        pa_modbus_t mb2;
        uint8_t small_tx[16];
        uint8_t small_rx[16];
        setup_default(&mb2, small_tx, 16, small_rx, sizeof(small_rx), 0x01);

        uint8_t values[200];
        memset(values, 0, sizeof(values));
        int len = pa_modbus_build_write_multiple_coils(&mb2, 0, values, 100);
        TEST_ASSERT(len == PA_ERR_BUFFER_FULL, "Buffer overflow returns PA_ERR_BUFFER_FULL");
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
        TEST_ASSERT(len == 12, "TCP frame length is 12 bytes");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[2] == 0x00, "Protocol ID hi = 0x00");
        TEST_ASSERT(buf[3] == 0x00, "Protocol ID lo = 0x00");
        TEST_ASSERT(buf[4] == 0x00, "Length hi = 0x00");
        TEST_ASSERT(buf[5] == 0x06, "Length lo = 0x06 (PDU+unit)");
        TEST_ASSERT(buf[6] == 0x01, "Unit ID = 0x01");
        TEST_ASSERT(buf[7] == 0x03, "FC = 0x03 at PDU offset 0");
    }

    /* Transaction ID increments */
    {
        uint16_t tid1 = (uint16_t)(((uint16_t)pa_modbus_tx_buf(&mb)[0] << 8) | pa_modbus_tx_buf(&mb)[1]);
        pa_modbus_build_read_holding_registers(&mb, 0, 3);
        uint16_t tid2 = (uint16_t)(((uint16_t)pa_modbus_tx_buf(&mb)[0] << 8) | pa_modbus_tx_buf(&mb)[1]);
        TEST_ASSERT(tid2 > tid1, "Transaction ID increments");
    }

    /* Master feed with a valid TCP response */
    {
        uint8_t resp[16] = {0};
        resp[0] = 0x00; resp[1] = 0x01;  /* Transaction ID */
        resp[4] = 0x00; resp[5] = 0x09;  /* Length = 9 */
        resp[6] = 0x01;                   /* Unit ID */
        resp[7] = 0x03;                   /* FC 03 */
        resp[8] = 0x06;                   /* Byte count = 6 */
        resp[9] = 0x00; resp[10] = 0x01;  /* Register 0 = 1 */
        resp[11] = 0x00; resp[12] = 0x02; /* Register 1 = 2 */
        resp[13] = 0x00; resp[14] = 0x03; /* Register 2 = 3 */

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_OK, "TCP master_feed returns PA_OK");
        TEST_ASSERT(pa_modbus_get_register(&mb, 0) == 1, "TCP reg 0 = 1");
        TEST_ASSERT(pa_modbus_get_register(&mb, 2) == 3, "TCP reg 2 = 3");
    }

    /* Wrong unit ID */
    {
        uint8_t resp[16] = {0};
        resp[4] = 0x00; resp[5] = 0x09;
        resp[6] = 0x02;                   /* Wrong unit ID */
        resp[7] = 0x03; resp[8] = 0x06;
        memset(resp + 9, 0, 6);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_ERR_INVALID_SLAVE, "Wrong unit ID returns PA_ERR_INVALID_SLAVE");
    }

    /* Non-zero protocol ID */
    {
        uint8_t resp[16] = {0};
        resp[2] = 0x00; resp[3] = 0x01;  /* Protocol ID = 1 (invalid) */
        resp[4] = 0x00; resp[5] = 0x09;
        resp[6] = 0x01; resp[7] = 0x03; resp[8] = 0x06;

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_ERR_PROTOCOL, "Non-zero protocol ID returns PA_ERR_PROTOCOL");
    }

    /* Incomplete TCP frame (need more data) */
    {
        uint8_t partial[6] = {0}; /* MBAP needs at least 8 bytes */
        int ret = pa_modbus_master_feed(&mb, partial, 6);
        TEST_ASSERT(ret > 0, "Partial TCP frame returns >0");
    }
}

/* ---------------------------------------------------------------------------
 * Test: RTU master feed
 * ------------------------------------------------------------------------- */

static void test_rtu_master_feed(void)
{
    printf("\n=== RTU Master Feed Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];
    setup_default(&mb, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf), 0x01);

    pa_modbus_build_read_holding_registers(&mb, 0, 3);

    /* Valid RTU response */
    {
        uint8_t resp[] = {
            0x01, 0x03, 0x06,
            0x00, 0x01, 0x00, 0x02, 0x00, 0x03,
            0x00, 0x00
        };
        uint16_t crc = pa_crc16(resp, 9);
        resp[9]  = (uint8_t)(crc & 0xFF);
        resp[10] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_OK, "RTU master_feed returns PA_OK");
        TEST_ASSERT(pa_modbus_get_register(&mb, 0) == 1, "RTU reg 0 = 1");
        TEST_ASSERT(pa_modbus_get_register(&mb, 2) == 3, "RTU reg 2 = 3");
    }

    /* CRC mismatch */
    {
        uint8_t bad_resp[11];
        /* build a fresh response with correct CRC inside the mb context */
        uint8_t resp[] = {0x01, 0x03, 0x06, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(resp, 9);
        resp[9]  = (uint8_t)(crc & 0xFF);
        resp[10] = (uint8_t)((crc >> 8) & 0xFF);
        memcpy(bad_resp, resp, sizeof(bad_resp));
        bad_resp[9] ^= 0xFF;
        int ret2 = pa_modbus_master_feed(&mb, bad_resp, sizeof(bad_resp));
        TEST_ASSERT(ret2 == PA_ERR_CRC, "Corrupt CRC returns PA_ERR_CRC");
    }

    /* Wrong slave address */
    {
        uint8_t resp[] = {0x01, 0x03, 0x06, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(resp, 9);
        resp[9]  = (uint8_t)(crc & 0xFF);
        resp[10] = (uint8_t)((crc >> 8) & 0xFF);

        uint8_t wrong_slave[11];
        memcpy(wrong_slave, resp, sizeof(wrong_slave));
        wrong_slave[0] = 0x02;
        uint16_t crc2 = pa_crc16(wrong_slave, 9);
        wrong_slave[9]  = (uint8_t)(crc2 & 0xFF);
        wrong_slave[10] = (uint8_t)((crc2 >> 8) & 0xFF);

        int ret2 = pa_modbus_master_feed(&mb, wrong_slave, sizeof(wrong_slave));
        TEST_ASSERT(ret2 == PA_ERR_INVALID_SLAVE, "Wrong slave returns PA_ERR_INVALID_SLAVE");
    }

    /* Incomplete frame */
    {
        uint8_t partial[] = {0x01, 0x03, 0x06};
        int ret2 = pa_modbus_master_feed(&mb, partial, sizeof(partial));
        TEST_ASSERT(ret2 > 0, "Partial frame returns >0 (need more data)");
    }

    /* FC 01 response: Read Coils */
    {
        pa_modbus_build_read_coils(&mb, 0, 16);
        uint8_t resp[] = {0x01, 0x01, 0x02, 0x55, 0xAA, 0x00, 0x00};
        uint16_t crc = pa_crc16(resp, 5);
        resp[5] = (uint8_t)(crc & 0xFF);
        resp[6] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_OK, "FC01 parse OK");

        TEST_ASSERT(pa_modbus_get_coil(&mb, 0) == 1, "FC01 coil 0 = 1 (0x55 bit 0)");
        TEST_ASSERT(pa_modbus_get_coil(&mb, 1) == 0, "FC01 coil 1 = 0");
        TEST_ASSERT(pa_modbus_get_coil(&mb, 7) == 0, "FC01 coil 7 = 0 (0x55 bit 7 is 0)");
        TEST_ASSERT(pa_modbus_get_coil(&mb, 8) == 0, "FC01 coil 8 = 0 (0xAA bit 0)");
        TEST_ASSERT(pa_modbus_get_coil(&mb, 15) == 1, "FC01 coil 15 = 1");
        /* Out of range should return 0 */
        TEST_ASSERT(pa_modbus_get_coil(&mb, 999) == 0, "FC01 out-of-range coil returns 0");
    }

    /* FC 02 response: Read Discrete Inputs */
    {
        pa_modbus_build_read_discrete_inputs(&mb, 0, 8);
        uint8_t resp[] = {0x01, 0x02, 0x01, 0xF0, 0x00, 0x00};
        uint16_t crc = pa_crc16(resp, 4);
        resp[4] = (uint8_t)(crc & 0xFF);
        resp[5] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_OK, "FC02 parse OK");
        TEST_ASSERT(pa_modbus_get_coil(&mb, 0) == 0, "FC02 input 0 = 0");
        TEST_ASSERT(pa_modbus_get_coil(&mb, 4) == 1, "FC02 input 4 = 1");
    }

    /* FC 04 response: Read Input Registers */
    {
        pa_modbus_build_read_input_registers(&mb, 0, 3);
        uint8_t resp[] = {0x01, 0x04, 0x06, 0x00, 0x0A, 0x00, 0x14, 0x00, 0x1E, 0x00, 0x00};
        uint16_t crc = pa_crc16(resp, 9);
        resp[9]  = (uint8_t)(crc & 0xFF);
        resp[10] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_OK, "FC04 parse OK");
        TEST_ASSERT(pa_modbus_get_register(&mb, 0) == 10, "FC04 reg 0 = 10");
        TEST_ASSERT(pa_modbus_get_register(&mb, 2) == 30, "FC04 reg 2 = 30");
        /* Out of range returns 0 */
        TEST_ASSERT(pa_modbus_get_register(&mb, 999) == 0, "FC04 out-of-range reg returns 0");
    }

    /* Exception response */
    {
        pa_modbus_build_read_holding_registers(&mb, 0, 3);
        uint8_t resp[] = {0x01, 0x83, 0x02, 0x00, 0x00};
        uint16_t crc = pa_crc16(resp, 3);
        resp[3] = (uint8_t)(crc & 0xFF);
        resp[4] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_ERR_EXCEPTION, "Exception returns PA_ERR_EXCEPTION");
        TEST_ASSERT(pa_modbus_get_exception(&mb) == 0x02, "Exception code = 0x02");
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
    setup_default(&mb, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf), 0x01);

    pa_modbus_set_read_holding_registers_cb(&mb, test_read_holding_cb, NULL);
    pa_modbus_set_write_multiple_registers_cb(&mb, test_write_holding_cb, NULL);

    for (int i = 0; i < 100; i++)
        test_holding_regs[i] = (uint16_t)(i * 10);

    /* Read request (FC 03) */
    {
        uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "slave_feed FC03 OK");
        TEST_ASSERT(pa_modbus_slave_function(&mb) == 0x03, "Function = 0x03");
        TEST_ASSERT(pa_modbus_slave_addr(&mb) == 0, "Address = 0");
        TEST_ASSERT(pa_modbus_slave_count(&mb) == 3, "Count = 3");

        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len == 11, "FC03 response length = 11");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[0] == 0x01, "Response slave = 0x01");
        TEST_ASSERT(buf[2] == 0x06, "Byte count = 6");
        TEST_ASSERT(buf[3] == 0x00 && buf[4] == 0x00, "Reg[0] = 0");
        TEST_ASSERT(buf[5] == 0x00 && buf[6] == 0x0A, "Reg[1] = 10");
        TEST_ASSERT(buf[7] == 0x00 && buf[8] == 0x14, "Reg[2] = 20");
    }

    /* Write request (FC 10) */
    {
        uint8_t req[] = {0x01, 0x10, 0x00, 0x0A, 0x00, 0x02, 0x04,
                         0x00, 0x64, 0x00, 0xC8, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 11);
        req[11] = (uint8_t)(crc & 0xFF);
        req[12] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "slave_feed FC10 OK");

        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len > 0, "FC10 response positive");
        TEST_ASSERT(test_holding_regs[10] == 100, "Holding reg[10] = 100");
        TEST_ASSERT(test_holding_regs[11] == 200, "Holding reg[11] = 200");
        TEST_ASSERT(len == 8, "FC10 echo response length = 8");
    }

    /* Exception response */
    {
        uint8_t req[] = {0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        pa_modbus_slave_feed(&mb, req, sizeof(req));
        int len = pa_modbus_slave_respond_error(&mb, PA_EX_ILLEGAL_FUNCTION);
        TEST_ASSERT(len > 0, "slave_respond_error positive");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[1] == (0x07 | 0x80), "Exception FC has MSB set");
        TEST_ASSERT(buf[2] == 0x01, "Exception code = ILLEGAL_FUNCTION");
    }
}

/* ---------------------------------------------------------------------------
 * Test: Coil read/write
 * ------------------------------------------------------------------------- */

static uint8_t test_coils[32];

static int test_read_coils_cb(uint16_t addr, uint16_t count, uint8_t *values, void *userdata)
{
    (void)userdata;
    size_t byte_count = (size_t)((count + 7) / 8);
    memset(values, 0, byte_count);
    for (uint16_t i = 0; i < count; i++) {
        uint16_t a = addr + i;
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

static int test_write_multiple_coils_cb(uint16_t addr, uint16_t count, const uint8_t *values, void *userdata)
{
    (void)userdata;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t a = addr + i;
        if (a >= 256) return -1;
        if (values[i / 8] & (1 << (i % 8)))
            test_coils[a / 8] |= (uint8_t)(1 << (a % 8));
        else
            test_coils[a / 8] &= ~(uint8_t)(1 << (a % 8));
    }
    return 0;
}

static void test_coil_operations(void)
{
    printf("\n=== Coil Operation Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];
    setup_default(&mb, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf), 0x01);

    pa_modbus_set_read_coils_cb(&mb, test_read_coils_cb, NULL);
    pa_modbus_set_write_single_coil_cb(&mb, test_write_single_coil_cb, NULL);
    pa_modbus_set_write_multiple_coils_cb(&mb, test_write_multiple_coils_cb, NULL);

    memset(test_coils, 0, sizeof(test_coils));

    /* Write single coil ON (FC 05) */
    {
        uint8_t req[] = {0x01, 0x05, 0x00, 0x05, 0xFF, 0x00, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Write single coil ON feed OK");
        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len > 0, "Write single coil ON response OK");
        TEST_ASSERT(test_coils[0] & (1 << 5), "Coil 5 is ON");
    }

    /* Write single coil OFF (FC 05) */
    {
        uint8_t req[] = {0x01, 0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Write single coil OFF feed OK");
        pa_modbus_slave_respond(&mb);
        TEST_ASSERT(!(test_coils[0] & (1 << 5)), "Coil 5 is OFF");
    }

    /* Write multiple coils (FC 0F) */
    {
        uint8_t req[] = {0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0xAA, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 8);
        req[8] = (uint8_t)(crc & 0xFF);
        req[9] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Write multiple coils feed OK");
        pa_modbus_slave_respond(&mb);

        TEST_ASSERT(test_coils[0] & (1 << 1), "Coil 1 is ON (0xAA bit 1)");
        TEST_ASSERT(!(test_coils[0] & (1 << 0)), "Coil 0 is OFF (0xAA bit 0)");
        TEST_ASSERT(test_coils[0] & (1 << 3), "Coil 3 is ON");
        TEST_ASSERT(!(test_coils[0] & (1 << 2)), "Coil 2 is OFF");
    }

    /* Read coils (FC 01) */
    {
        uint8_t req[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Read coils feed OK");

        int len = pa_modbus_slave_respond(&mb);
        TEST_ASSERT(len > 0, "Read coils response OK");

        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[3] & (1 << 1), "Coil 1 is set in response");
        TEST_ASSERT(!(buf[3] & (1 << 0)), "Coil 0 is clear in response");
    }
}

/* ---------------------------------------------------------------------------
 * Test: Userdata isolation with multiple contexts
 * ------------------------------------------------------------------------- */

static int read_holding_ud_cb(uint16_t addr, uint16_t count, uint16_t *values, void *userdata)
{
    uint16_t *store = (uint16_t *)userdata;
    for (uint16_t i = 0; i < count; i++)
        values[i] = store[addr + i];
    return 0;
}

static void test_userdata_isolation(void)
{
    printf("\n=== Userdata Isolation Tests ===\n");

    uint16_t store_a[100];
    uint16_t store_b[100];
    for (int i = 0; i < 100; i++) {
        store_a[i] = (uint16_t)(i);
        store_b[i] = (uint16_t)(i * 100);
    }

    pa_modbus_t mb_a, mb_b;
    uint8_t tx_a[256], rx_a[256], tx_b[256], rx_b[256];

    setup_default(&mb_a, tx_a, sizeof(tx_a), rx_a, sizeof(rx_a), 0x01);
    pa_modbus_set_read_holding_registers_cb(&mb_a, read_holding_ud_cb, store_a);

    setup_default(&mb_b, tx_b, sizeof(tx_b), rx_b, sizeof(rx_b), 0x02);
    pa_modbus_set_read_holding_registers_cb(&mb_b, read_holding_ud_cb, store_b);

    /* Same request, different contexts */
    uint8_t req[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
    uint16_t crc = pa_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)((crc >> 8) & 0xFF);

    /* Context A uses slave 0x01 — this request matches */
    TEST_ASSERT(pa_modbus_slave_feed(&mb_a, req, sizeof(req)) == PA_OK, "Context A feed OK");
    pa_modbus_slave_respond(&mb_a);
    const uint8_t *buf_a = pa_modbus_tx_buf(&mb_a);
    TEST_ASSERT(buf_a[0] == 0x01, "Context A slave addr = 0x01");
    TEST_ASSERT(buf_a[3] == 0x00 && buf_a[4] == 0x00, "Context A reg[0] = 0");
    TEST_ASSERT(buf_a[5] == 0x00 && buf_a[6] == 0x01, "Context A reg[1] = 1");

    /* Context B uses slave 0x02 — request with slave 0x01 will be rejected */
    int ret_b = pa_modbus_slave_feed(&mb_b, req, sizeof(req));
    TEST_ASSERT(ret_b == PA_ERR_INVALID_SLAVE, "Context B rejects slave 0x01 request");

    /* Now send a request with slave 0x02 */
    uint8_t req_b[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
    uint16_t crc_b = pa_crc16(req_b, 6);
    req_b[6] = (uint8_t)(crc_b & 0xFF);
    req_b[7] = (uint8_t)((crc_b >> 8) & 0xFF);

    TEST_ASSERT(pa_modbus_slave_feed(&mb_b, req_b, sizeof(req_b)) == PA_OK, "Context B feed with slave 0x02 OK");
    pa_modbus_slave_respond(&mb_b);
    const uint8_t *buf_b = pa_modbus_tx_buf(&mb_b);
    TEST_ASSERT(buf_b[0] == 0x02, "Context B slave addr = 0x02");
    TEST_ASSERT(buf_b[3] == 0x00 && buf_b[4] == 0x00, "Context B reg[0] = 0");
    TEST_ASSERT(buf_b[5] == 0x00 && buf_b[6] == 0x64, "Context B reg[1] = 100 = 0x64");
}

/* ---------------------------------------------------------------------------
 * Test: Broadcast address (slave = 0xFF)
 * ------------------------------------------------------------------------- */

static void test_broadcast_address(void)
{
    printf("\n=== Broadcast Address Tests ===\n");

    pa_modbus_t mb_master, mb_slave;
    uint8_t tx_m[256], rx_m[256], tx_s[256], rx_s[256];

    /* Master with slave=0xFF (broadcast) — should accept any response slave */
    setup_default(&mb_master, tx_m, sizeof(tx_m), rx_m, sizeof(rx_m), 0xFF);

    /* Slave with slave=0xFF (respond to all) */
    setup_default(&mb_slave, tx_s, sizeof(tx_s), rx_s, sizeof(rx_s), 0xFF);
    pa_modbus_set_read_holding_registers_cb(&mb_slave, test_read_holding_cb, NULL);

    /* Master builds a request with broadcast slave */
    pa_modbus_build_read_holding_registers(&mb_master, 0, 3);

    /* Simulate response with a different slave address — should be accepted */
    uint8_t resp[] = {0x05, 0x03, 0x06, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00};
    uint16_t crc = pa_crc16(resp, 9);
    resp[9]  = (uint8_t)(crc & 0xFF);
    resp[10] = (uint8_t)((crc >> 8) & 0xFF);

    int ret = pa_modbus_master_feed(&mb_master, resp, sizeof(resp));
    TEST_ASSERT(ret == PA_OK, "Broadcast master accepts any slave address");
    TEST_ASSERT(pa_modbus_get_register(&mb_master, 0) == 1, "Broadcast master reg 0 = 1");

    /* Slave with 0xFF accepts any request */
    uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
    uint16_t crc_r = pa_crc16(req, 6);
    req[6] = (uint8_t)(crc_r & 0xFF);
    req[7] = (uint8_t)((crc_r >> 8) & 0xFF);

    ret = pa_modbus_slave_feed(&mb_slave, req, sizeof(req));
    TEST_ASSERT(ret == PA_OK, "Broadcast slave accepts request");
}

/* ---------------------------------------------------------------------------
 * Test: Framer get/set and mode switching
 * ------------------------------------------------------------------------- */

static void test_framer_switch(void)
{
    printf("\n=== Framer Switch Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];

    pa_modbus_init(&mb);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_slave(&mb, 0x01);

    /* Default is RTU */
    TEST_ASSERT(pa_modbus_get_framer(&mb) == PA_FRAMER_RTU, "Default framer is RTU");

    /* Switch to TCP */
    pa_modbus_set_framer(&mb, PA_FRAMER_TCP);
    TEST_ASSERT(pa_modbus_get_framer(&mb) == PA_FRAMER_TCP, "Framer switched to TCP");

    /* Build with TCP */
    int len = pa_modbus_build_read_holding_registers(&mb, 0, 3);
    TEST_ASSERT(len == 12, "TCP frame after switch = 12 bytes");

    /* Switch back to RTU */
    pa_modbus_set_framer(&mb, PA_FRAMER_RTU);
    TEST_ASSERT(pa_modbus_get_framer(&mb) == PA_FRAMER_RTU, "Framer switched back to RTU");

    len = pa_modbus_build_read_holding_registers(&mb, 0, 3);
    TEST_ASSERT(len == 8, "RTU frame after switch back = 8 bytes");
}

/* ---------------------------------------------------------------------------
 * Test: Discovery address
 * ------------------------------------------------------------------------- */

static void test_discovery_address(void)
{
    printf("\n=== Discovery Address Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];

    /* Slave with primary address 0x01 and discovery address 0xFF */
    setup_default(&mb, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf), 0x01);
    pa_modbus_set_read_holding_registers_cb(&mb, test_read_holding_cb, NULL);
    pa_modbus_set_discovery_addr(&mb, 0xFF);

    /* Default is disabled */
    TEST_ASSERT(pa_modbus_get_discovery_addr(&mb) == 0xFF, "Discovery addr = 0xFF after set");

    /* Request to primary address (0x01) should be accepted */
    {
        uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Primary address (0x01) accepted");
    }

    /* Request to discovery address (0xFF) should be accepted */
    {
        uint8_t req[] = {0xFF, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Discovery address (0xFF) accepted");
    }

    /* Request to unknown address (0x02) should be rejected */
    {
        uint8_t req[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_ERR_INVALID_SLAVE, "Unknown address (0x02) rejected");
    }

    /* Disable discovery address — 0xFF should now be rejected */
    pa_modbus_set_discovery_addr(&mb, 0);
    TEST_ASSERT(pa_modbus_get_discovery_addr(&mb) == 0, "Discovery addr disabled");

    {
        uint8_t req[] = {0xFF, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_ERR_INVALID_SLAVE, "0xFF rejected after discovery disabled");
    }

    /* Primary address still works after discovery disabled */
    {
        uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00};
        uint16_t crc = pa_crc16(req, 6);
        req[6] = (uint8_t)(crc & 0xFF);
        req[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_slave_feed(&mb, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "Primary address still works after discovery disabled");
    }

    /* Discovery address with TCP framer */
    {
        pa_modbus_t mb_tcp;
        uint8_t tcp_tx[256], tcp_rx[256];
        pa_modbus_init(&mb_tcp);
        pa_modbus_set_framer(&mb_tcp, PA_FRAMER_TCP);
        pa_modbus_set_txbuf(&mb_tcp, tcp_tx, sizeof(tcp_tx));
        pa_modbus_set_rxbuf(&mb_tcp, tcp_rx, sizeof(tcp_rx));
        pa_modbus_set_slave(&mb_tcp, 0x01);
        pa_modbus_set_discovery_addr(&mb_tcp, 0xFF);
        pa_modbus_set_read_holding_registers_cb(&mb_tcp, test_read_holding_cb, NULL);

        /* TCP request to discovery address (unit ID 0xFF) 
         * MBAP(7): trans_id(2) + proto_id(2) + length(2) + unit_id(1)
         * PDU: fc(1) + addr(2) + count(2) = 5
         * MBAP length field = PDU(5) + unit_id(1) = 6 */
        uint8_t req[13] = {0};
        req[4] = 0x00; req[5] = 0x06;  /* Length = 6 */
        req[6] = 0xFF;                   /* Unit ID = 0xFF (discovery) */
        req[7] = 0x03;                   /* FC 03 */
        req[8] = 0x00; req[9] = 0x00;   /* Addr = 0 */
        req[10] = 0x00; req[11] = 0x03; /* Count = 3 */

        int ret = pa_modbus_slave_feed(&mb_tcp, req, sizeof(req));
        TEST_ASSERT(ret == PA_OK, "TCP discovery address accepted via slave_feed");
    }
}

/* ---------------------------------------------------------------------------
 * Test: Raw I/O helpers (pa_modbus_send_raw / pa_modbus_recv_raw)
 * ------------------------------------------------------------------------- */

/* Simple in-memory loopback for testing raw I/O */
static uint8_t loopback_tx[256];
static size_t loopback_tx_len = 0;
static uint8_t loopback_rx[256];
static size_t loopback_rx_len = 0;
static size_t loopback_rx_pos = 0;

static int loopback_send_cb(const uint8_t *data, size_t len, void *userdata)
{
    (void)userdata;
    if (len > sizeof(loopback_tx)) return -1;
    memcpy(loopback_tx, data, len);
    loopback_tx_len = len;
    return 0;
}

static int loopback_recv_cb(uint8_t *data, size_t max_len, void *userdata)
{
    (void)userdata;
    size_t remaining = loopback_rx_len - loopback_rx_pos;
    if (remaining == 0) return 0;
    size_t n = remaining < max_len ? remaining : max_len;
    memcpy(data, loopback_rx + loopback_rx_pos, n);
    loopback_rx_pos += n;
    return (int)n;
}

static void test_raw_io(void)
{
    printf("\n=== Raw I/O Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];
    setup_default(&mb, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf), 0x01);
    pa_modbus_set_send_cb(&mb, loopback_send_cb, NULL);
    pa_modbus_set_recv_cb(&mb, loopback_recv_cb, NULL);

    /* Send raw PDU with RTU framing */
    {
        uint8_t pdu[] = {0x11, 0xAA, 0xBB}; /* Custom discovery frame */
        int ret = pa_modbus_send_raw(&mb, pdu, sizeof(pdu));
        TEST_ASSERT(ret > 0, "send_raw returns positive framed length");
        TEST_ASSERT((size_t)ret == loopback_tx_len, "send_raw sent correct length");
        TEST_ASSERT(loopback_tx[0] == 0x01, "RTU slave addr = 0x01");
        TEST_ASSERT(loopback_tx[1] == 0x11, "PDU byte 0 = 0x11");
        TEST_ASSERT(loopback_tx[2] == 0xAA, "PDU byte 1 = 0xAA");
        TEST_ASSERT(loopback_tx[3] == 0xBB, "PDU byte 2 = 0xBB");

        /* Verify CRC */
        uint16_t crc_calc = pa_crc16(loopback_tx, 4);
        uint16_t crc_frame = (uint16_t)(loopback_tx[4] | ((uint16_t)loopback_tx[5] << 8));
        TEST_ASSERT(crc_calc == crc_frame, "send_raw CRC is correct");
    }

    /* Send raw with buffer overflow */
    {
        uint8_t big_pdu[300];
        memset(big_pdu, 0, sizeof(big_pdu));
        int ret = pa_modbus_send_raw(&mb, big_pdu, sizeof(big_pdu));
        TEST_ASSERT(ret == PA_ERR_BUFFER_FULL, "send_raw buffer overflow returns PA_ERR_BUFFER_FULL");
    }

    /* Send raw without send callback */
    {
        pa_modbus_t mb2;
        uint8_t tx2[64], rx2[64];
        setup_default(&mb2, tx2, sizeof(tx2), rx2, sizeof(rx2), 0x01);
        uint8_t pdu[] = {0x11};
        int ret = pa_modbus_send_raw(&mb2, pdu, sizeof(pdu));
        TEST_ASSERT(ret == PA_ERR_STATE, "send_raw without callback returns PA_ERR_STATE");
    }

    /* Receive raw with RTU framing */
    {
        /* Build a framed response: slave=0x01, PDU={0x11, 0x01, 0x02}, CRC */
        uint8_t frame[6];
        frame[0] = 0x01;
        frame[1] = 0x11;
        frame[2] = 0x01;
        frame[3] = 0x02;
        uint16_t crc = pa_crc16(frame, 4);
        frame[4] = (uint8_t)(crc & 0xFF);
        frame[5] = (uint8_t)((crc >> 8) & 0xFF);

        memcpy(loopback_rx, frame, sizeof(frame));
        loopback_rx_len = sizeof(frame);
        loopback_rx_pos = 0;

        uint8_t pdu_buf[64];
        size_t pdu_len = sizeof(pdu_buf);
        int ret = pa_modbus_recv_raw(&mb, pdu_buf, &pdu_len);
        TEST_ASSERT(ret == PA_OK, "recv_raw returns PA_OK");
        TEST_ASSERT(pdu_len == 3, "recv_raw PDU length = 3");
        TEST_ASSERT(pdu_buf[0] == 0x11, "recv_raw PDU[0] = 0x11");
        TEST_ASSERT(pdu_buf[1] == 0x01, "recv_raw PDU[1] = 0x01");
        TEST_ASSERT(pdu_buf[2] == 0x02, "recv_raw PDU[2] = 0x02");
    }

    /* Receive raw with timeout (no data) */
    {
        loopback_rx_len = 0;
        loopback_rx_pos = 0;
        uint8_t pdu_buf[64];
        size_t pdu_len = sizeof(pdu_buf);
        int ret = pa_modbus_recv_raw(&mb, pdu_buf, &pdu_len);
        TEST_ASSERT(ret == PA_ERR_TIMEOUT, "recv_raw timeout returns PA_ERR_TIMEOUT");
    }

    /* Receive raw with bad CRC */
    {
        uint8_t frame[6];
        frame[0] = 0x01;
        frame[1] = 0x11;
        frame[2] = 0x01;
        frame[3] = 0x02;
        frame[4] = 0xFF; /* Bad CRC */
        frame[5] = 0xFF;

        memcpy(loopback_rx, frame, sizeof(frame));
        loopback_rx_len = sizeof(frame);
        loopback_rx_pos = 0;

        uint8_t pdu_buf[64];
        size_t pdu_len = sizeof(pdu_buf);
        int ret = pa_modbus_recv_raw(&mb, pdu_buf, &pdu_len);
        TEST_ASSERT(ret == PA_ERR_TIMEOUT, "recv_raw bad CRC returns PA_ERR_TIMEOUT (no valid frame found)");
    }
}

/* ---------------------------------------------------------------------------
 * Test: FC 0x11 (Report Slave ID)
 * ------------------------------------------------------------------------- */

static void test_report_slave_id(void)
{
    printf("\n=== Report Slave ID (FC 0x11) Tests ===\n");

    pa_modbus_t mb;
    uint8_t txbuf[256];
    uint8_t rxbuf[256];
    setup_default(&mb, txbuf, sizeof(txbuf), rxbuf, sizeof(rxbuf), 0x01);

    /* Build FC 0x11 request */
    {
        int len = pa_modbus_build_report_slave_id(&mb);
        TEST_ASSERT(len == 4, "FC11 RTU frame length = 4");
        const uint8_t *buf = pa_modbus_tx_buf(&mb);
        TEST_ASSERT(buf[0] == 0x01, "FC11 slave addr = 0x01");
        TEST_ASSERT(buf[1] == 0x11, "FC11 function code = 0x11");

        uint16_t crc_calc = pa_crc16(buf, 2);
        uint16_t crc_frame = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
        TEST_ASSERT(crc_calc == crc_frame, "FC11 CRC is correct");
    }

    /* Parse FC 0x11 response */
    {
        /* Response: slave=0x01, FC=0x11, byte_count=3, data={0x01, 0xAA, 0xBB} */
        uint8_t resp[] = {0x01, 0x11, 0x03, 0x01, 0xAA, 0xBB, 0x00, 0x00};
        uint16_t crc = pa_crc16(resp, 6);
        resp[6] = (uint8_t)(crc & 0xFF);
        resp[7] = (uint8_t)((crc >> 8) & 0xFF);

        int ret = pa_modbus_master_feed(&mb, resp, sizeof(resp));
        TEST_ASSERT(ret == PA_OK, "FC11 response parse OK");
        TEST_ASSERT(pa_modbus_get_register(&mb, 0) == 0x01AA, "FC11 data[0..1] = 0x01AA");
        TEST_ASSERT(pa_modbus_get_register(&mb, 1) == 0x00BB, "FC11 data[2] padded = 0x00BB");
    }
}

/* ---------------------------------------------------------------------------
 * Test: Discovery handshake (master + slave loopback)
 * ------------------------------------------------------------------------- */

/* Slave-side discovery handler: responds to DISCO_REQ with its serial number */
static uint8_t slave_serial[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
static uint8_t slave_assigned_id = 0x00;

static int disco_slave_send_cb(const uint8_t *data, size_t len, void *userdata)
{
    (void)userdata;
    if (len > sizeof(loopback_tx)) return -1;
    memcpy(loopback_tx, data, len);
    loopback_tx_len = len;
    return 0;
}

static int disco_slave_recv_cb(uint8_t *data, size_t max_len, void *userdata)
{
    (void)userdata;
    size_t remaining = loopback_rx_len - loopback_rx_pos;
    if (remaining == 0) return 0;
    size_t n = remaining < max_len ? remaining : max_len;
    memcpy(data, loopback_rx + loopback_rx_pos, n);
    loopback_rx_pos += n;
    return (int)n;
}

static void test_discovery_handshake(void)
{
    printf("\n=== Discovery Handshake Tests ===\n");

    /* Master context */
    pa_modbus_t master;
    uint8_t master_tx[256], master_rx[256];
    setup_default(&master, master_tx, sizeof(master_tx), master_rx, sizeof(master_rx), 0xFF);
    pa_modbus_set_send_cb(&master, loopback_send_cb, NULL);
    pa_modbus_set_recv_cb(&master, loopback_recv_cb, NULL);

    /* Slave context with discovery address 0xFF */
    pa_modbus_t slave;
    uint8_t slave_tx[256], slave_rx[256];
    setup_default(&slave, slave_tx, sizeof(slave_tx), slave_rx, sizeof(slave_rx), 0x00);
    pa_modbus_set_discovery_addr(&slave, 0xFF);
    pa_modbus_set_send_cb(&slave, disco_slave_send_cb, NULL);
    pa_modbus_set_recv_cb(&slave, disco_slave_recv_cb, NULL);

    /* Step 1: Master sends DISCO_REQ to 0xFF via raw I/O */
    {
        /* DISCO_REQ PDU: {0x11, 0x01} — FC 0x11 (Report Slave ID) as discovery probe */
        uint8_t disco_req[] = {0x11, 0x01};
        int ret = pa_modbus_send_raw(&master, disco_req, sizeof(disco_req));
        TEST_ASSERT(ret > 0, "Master sends DISCO_REQ via send_raw");

        /* Copy master's framed output to slave's receive buffer */
        memcpy(loopback_rx, loopback_tx, loopback_tx_len);
        loopback_rx_len = loopback_tx_len;
        loopback_rx_pos = 0;

        /* Slave receives and parses the DISCO_REQ */
        uint8_t pdu_buf[64];
        size_t pdu_len = sizeof(pdu_buf);
        ret = pa_modbus_recv_raw(&slave, pdu_buf, &pdu_len);
        TEST_ASSERT(ret == PA_OK, "Slave receives DISCO_REQ via recv_raw");
        TEST_ASSERT(pdu_len == 2, "DISCO_REQ PDU length = 2");
        TEST_ASSERT(pdu_buf[0] == 0x11, "DISCO_REQ FC = 0x11");
        TEST_ASSERT(pdu_buf[1] == 0x01, "DISCO_REQ probe = 0x01");
    }

    /* Step 2: Slave responds with its serial number */
    {
        /* DISCO_RSP PDU: {0x11, 0x02, serial[0..5]} */
        uint8_t disco_rsp[8];
        disco_rsp[0] = 0x11;
        disco_rsp[1] = 0x02;
        memcpy(&disco_rsp[2], slave_serial, 6);

        int ret = pa_modbus_send_raw(&slave, disco_rsp, sizeof(disco_rsp));
        TEST_ASSERT(ret > 0, "Slave sends DISCO_RSP via send_raw");

        /* Copy slave's framed output to master's receive buffer */
        memcpy(loopback_rx, loopback_tx, loopback_tx_len);
        loopback_rx_len = loopback_tx_len;
        loopback_rx_pos = 0;

        /* Master receives and parses the DISCO_RSP */
        uint8_t pdu_buf[64];
        size_t pdu_len = sizeof(pdu_buf);
        ret = pa_modbus_recv_raw(&master, pdu_buf, &pdu_len);
        TEST_ASSERT(ret == PA_OK, "Master receives DISCO_RSP via recv_raw");
        TEST_ASSERT(pdu_len == 8, "DISCO_RSP PDU length = 8");
        TEST_ASSERT(pdu_buf[0] == 0x11, "DISCO_RSP FC = 0x11");
        TEST_ASSERT(pdu_buf[1] == 0x02, "DISCO_RSP type = 0x02");
        TEST_ASSERT(memcmp(&pdu_buf[2], slave_serial, 6) == 0, "DISCO_RSP serial matches");
    }

    /* Step 3: Master assigns slave ID via ASSIGN_REQ */
    {
        /* ASSIGN_REQ PDU: {0x11, 0x03, new_id, serial[0..5]} */
        uint8_t assign_req[9];
        assign_req[0] = 0x11;
        assign_req[1] = 0x03;
        assign_req[2] = 0x07; /* New slave ID */
        memcpy(&assign_req[3], slave_serial, 6);

        int ret = pa_modbus_send_raw(&master, assign_req, sizeof(assign_req));
        TEST_ASSERT(ret > 0, "Master sends ASSIGN_REQ via send_raw");

        /* Copy master's framed output to slave's receive buffer */
        memcpy(loopback_rx, loopback_tx, loopback_tx_len);
        loopback_rx_len = loopback_tx_len;
        loopback_rx_pos = 0;

        /* Slave receives and parses the ASSIGN_REQ */
        uint8_t pdu_buf[64];
        size_t pdu_len = sizeof(pdu_buf);
        ret = pa_modbus_recv_raw(&slave, pdu_buf, &pdu_len);
        TEST_ASSERT(ret == PA_OK, "Slave receives ASSIGN_REQ via recv_raw");
        TEST_ASSERT(pdu_len == 9, "ASSIGN_REQ PDU length = 9");
        TEST_ASSERT(pdu_buf[0] == 0x11, "ASSIGN_REQ FC = 0x11");
        TEST_ASSERT(pdu_buf[1] == 0x03, "ASSIGN_REQ type = 0x03");
        TEST_ASSERT(pdu_buf[2] == 0x07, "ASSIGN_REQ new ID = 0x07");
        TEST_ASSERT(memcmp(&pdu_buf[3], slave_serial, 6) == 0, "ASSIGN_REQ serial matches");

        /* Slave updates its assigned ID */
        slave_assigned_id = pdu_buf[2];
        pa_modbus_set_slave(&slave, slave_assigned_id);
        TEST_ASSERT(pa_modbus_get_slave(&slave) == 0x07, "Slave ID updated to 0x07");
    }

    /* Step 4: Slave confirms assignment */
    {
        /* ASSIGN_RSP PDU: {0x11, 0x04, new_id} */
        uint8_t assign_rsp[3];
        assign_rsp[0] = 0x11;
        assign_rsp[1] = 0x04;
        assign_rsp[2] = slave_assigned_id;

        int ret = pa_modbus_send_raw(&slave, assign_rsp, sizeof(assign_rsp));
        TEST_ASSERT(ret > 0, "Slave sends ASSIGN_RSP via send_raw");

        /* Copy slave's framed output to master's receive buffer */
        memcpy(loopback_rx, loopback_tx, loopback_tx_len);
        loopback_rx_len = loopback_tx_len;
        loopback_rx_pos = 0;

        /* Master receives and parses the ASSIGN_RSP */
        uint8_t pdu_buf[64];
        size_t pdu_len = sizeof(pdu_buf);
        ret = pa_modbus_recv_raw(&master, pdu_buf, &pdu_len);
        TEST_ASSERT(ret == PA_OK, "Master receives ASSIGN_RSP via recv_raw");
        TEST_ASSERT(pdu_len == 3, "ASSIGN_RSP PDU length = 3");
        TEST_ASSERT(pdu_buf[0] == 0x11, "ASSIGN_RSP FC = 0x11");
        TEST_ASSERT(pdu_buf[1] == 0x04, "ASSIGN_RSP type = 0x04");
        TEST_ASSERT(pdu_buf[2] == 0x07, "ASSIGN_RSP confirmed ID = 0x07");
    }

    /* Step 5: Normal Modbus operation on assigned ID */
    {
        /* Master builds a standard FC03 read request to slave 0x07 */
        pa_modbus_set_slave(&master, 0x07);
        int len = pa_modbus_build_read_holding_registers(&master, 0, 3);
        TEST_ASSERT(len == 8, "Master builds FC03 to assigned slave");

        /* Send the built frame through the master's send callback */
        int ret = pa_modbus_send(&master);
        TEST_ASSERT(ret == PA_OK, "Master sends FC03 via pa_modbus_send");

        /* Copy master's framed output to slave's receive buffer */
        memcpy(loopback_rx, loopback_tx, loopback_tx_len);
        loopback_rx_len = loopback_tx_len;
        loopback_rx_pos = 0;

        /* Slave receives the FC03 request */
        ret = pa_modbus_slave_feed(&slave, loopback_rx, loopback_rx_len);
        TEST_ASSERT(ret == PA_OK, "Slave accepts FC03 on assigned ID");
        TEST_ASSERT(pa_modbus_slave_function(&slave) == 0x03, "Slave sees FC03");
    }
}

/* ---------------------------------------------------------------------------
 * Test: Get/set slave address
 * ------------------------------------------------------------------------- */

static void test_slave_address(void)
{
    printf("\n=== Slave Address Tests ===\n");

    pa_modbus_t mb;
    pa_modbus_init(&mb);

    /* Default is 0xFF */
    TEST_ASSERT(pa_modbus_get_slave(&mb) == 0xFF, "Default slave = 0xFF");

    pa_modbus_set_slave(&mb, 0x01);
    TEST_ASSERT(pa_modbus_get_slave(&mb) == 0x01, "Set slave = 0x01");

    pa_modbus_set_slave(&mb, 0xFF);
    TEST_ASSERT(pa_modbus_get_slave(&mb) == 0xFF, "Set slave = 0xFF");
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
    test_userdata_isolation();
    test_broadcast_address();
    test_framer_switch();
    test_discovery_address();
    test_raw_io();
    test_report_slave_id();
    test_discovery_handshake();
    test_slave_address();

    printf("\n==================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}