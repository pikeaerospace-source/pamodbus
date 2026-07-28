/*
 * pamodbus — internal definitions
 * MIT License — see LICENSE file for details.
 */

#ifndef PAMODBUS_INTERNAL_H
#define PAMODBUS_INTERNAL_H

#include "pamodbus.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Framer operations — each framer implements these
 * ------------------------------------------------------------------------- */

/** Maximum framing overhead: RTU = 3, TCP = 7. */
#define PA_FRAMER_MAX_OVERHEAD 7

/**
 * Framer function table.
 *
 * @param ctx       The MODBUS context.
 * @param pdu       Pointer to the PDU start within the TX buffer.
 * @param pdu_len   Length of the PDU (function code + data).
 * @param framed    [out] Set to the length of the complete framed frame
 *                  (including overhead).
 * @param data      Received data (for unwrap).
 * @param len       Length of received data (for unwrap).
 * @return 0 on success, negative on error.
 */
typedef struct {
    /** Compute framing overhead in bytes. */
    int    (*overhead)(void);

    /**
     * Wrap a PDU: insert framing bytes around it in the TX buffer.
     * The framer must place the complete frame starting at
     * ctx->txbuf with ctx->slave inserted appropriately.
     */
    int    (*wrap)(pa_modbus_t *ctx, int pdu_len, int *framed_len);

    /**
     * Unwrap received data: strip framing, return PDU pointer and length.
     * Returns 0 on success if enough data is present, or >0 indicating
     * how many bytes are still needed. Negative on error.
     */
    int    (*unwrap)(pa_modbus_t *ctx, const uint8_t *data, size_t len,
                     const uint8_t **pdu, size_t *pdu_len);
} pa_framer_ops_t;

/* Framer operation tables (defined in framer_rtu.c and framer_tcp.c). */
extern const pa_framer_ops_t pa_framer_rtu_ops;
extern const pa_framer_ops_t pa_framer_tcp_ops;

/* ---------------------------------------------------------------------------
 * MODBUS context structure (opaque to the user)
 * ------------------------------------------------------------------------- */

struct pa_modbus {
    /* --- Buffers --- */
    uint8_t  *txbuf;           /**< Transmit buffer. */
    size_t    txbuf_size;      /**< Transmit buffer capacity. */
    uint8_t  *rxbuf;           /**< Receive buffer. */
    size_t    rxbuf_size;      /**< Receive buffer capacity. */

    /* --- Configuration --- */
    pa_framer_t        framer_type; /**< Active framer enum. */
    const pa_framer_ops_t *framer;  /**< Active framer ops table. */
    uint8_t            slave;       /**< Slave/unit identifier. */

    /* --- I/O callbacks --- */
    pa_send_fn send_cb;
    void      *send_userdata;
    pa_recv_fn recv_cb;
    void      *recv_userdata;

    /* --- Register access callbacks --- */
    pa_read_coils_fn             read_coils_cb;
    void                        *read_coils_userdata;
    pa_read_discrete_inputs_fn   read_discrete_inputs_cb;
    void                        *read_discrete_inputs_userdata;
    pa_read_holding_registers_fn read_holding_registers_cb;
    void                        *read_holding_registers_userdata;
    pa_read_input_registers_fn   read_input_registers_cb;
    void                        *read_input_registers_userdata;
    pa_write_single_coil_fn             write_single_coil_cb;
    void                               *write_single_coil_userdata;
    pa_write_single_register_fn         write_single_register_cb;
    void                               *write_single_register_userdata;
    pa_write_multiple_coils_fn          write_multiple_coils_cb;
    void                               *write_multiple_coils_userdata;
    pa_write_multiple_registers_fn      write_multiple_registers_cb;
    void                               *write_multiple_registers_userdata;

    /* --- Build state --- */
    size_t tx_len;     /**< Length of the framed frame in txbuf. */

    /* --- Parse state --- */
    size_t rx_len;     /**< Number of valid bytes currently in rxbuf. */

    /* --- Last parse result (master responses) --- */
    pa_error_t last_error;
    uint8_t    last_exception;
    uint8_t    last_function;    /**< Function code from parsed frame. */
    uint8_t    last_coil_data[256 / 8 + 1];  /**< Up to 256 coils. */
    uint16_t   last_reg_data[128];            /**< Up to 128 registers. */
    uint16_t   last_coil_count;  /**< Number of coils in last_coil_data. */
    uint16_t   last_reg_count;   /**< Number of registers in last_reg_data. */

    /* --- Slave request info (valid after slave_feed returns PA_OK) --- */
    uint16_t   slave_addr;       /**< Request starting address. */
    uint16_t   slave_count;      /**< Request count. */
    uint8_t    slave_coil_data[256 / 8 + 1];
    uint16_t   slave_reg_data[128];
    uint16_t   slave_coil_count; /* Number of coils in slave_coil_data */
    uint16_t   slave_reg_count;  /* Number of regs in slave_reg_data */
    int        slave_data_valid; /* Non-zero when slave_coil/reg_data is populated */
};

#ifdef __cplusplus
}
#endif

#endif /* PAMODBUS_INTERNAL_H */