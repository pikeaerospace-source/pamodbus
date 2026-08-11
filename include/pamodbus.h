/*
 * pamodbus — Lightweight MODBUS Library for Embedded Systems
 *
 * MIT License — see LICENSE file for details.
 */

#ifndef PAMODBUS_H
#define PAMODBUS_H

#include <stddef.h>  /* size_t */
#include <stdint.h>  /* uint8_t, uint16_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------------- */

/** Opaque MODBUS context struct. */
typedef struct pa_modbus pa_modbus_t;

/* ---------------------------------------------------------------------------
 * Enums
 * ------------------------------------------------------------------------- */

/** Framer type. */
typedef enum {
    PA_FRAMER_RTU,   /**< MODBUS RTU: slave addr + CRC-16 */
    PA_FRAMER_TCP,   /**< MODBUS TCP: MBAP header (7 bytes) */
} pa_framer_t;

/** Error codes. */
typedef enum {
    PA_OK                =  0,  /**< Success */
    PA_WAIT              =  1,  /**< Non-blocking receive: no complete frame yet, keep polling. */
    PA_ERR_CRC           = -1,  /**< CRC mismatch (RTU only) */
    PA_ERR_TIMEOUT       = -2,  /**< Response timeout */
    PA_ERR_INVALID_SLAVE = -3,  /**< Response slave/unit ID mismatch */
    PA_ERR_BAD_PARAM     = -4,  /**< Invalid parameter (address, count, etc.) */
    PA_ERR_BUFFER_FULL   = -5,  /**< TX or RX buffer overflow */
    PA_ERR_EXCEPTION     = -6,  /**< MODBUS exception received */
    PA_ERR_PROTOCOL      = -7,  /**< Malformed frame */
    PA_ERR_STATE         = -8,  /**< Invalid state for requested operation */
    PA_ERR_CALLBACK      = -9,  /**< Callback returned error */
} pa_error_t;

/**
 * Receive/parse role.
 *
 * Determines how received bytes are dispatched once a full frame is assembled.
 * Previously this was inferred from whether write callbacks were registered;
 * it is now explicit and must be set by the application via
 * pa_modbus_set_mode(). A legacy heuristic remains when it is not set.
 */
typedef enum {
    PA_MODE_MASTER = 0,  /**< Parse slave responses (master role). */
    PA_MODE_SLAVE  = 1,  /**< Parse master requests (slave role). */
} pa_modbus_mode_t;

/** MODBUS exception codes. */
typedef enum {
    PA_EX_ILLEGAL_FUNCTION       = 0x01,
    PA_EX_ILLEGAL_DATA_ADDRESS   = 0x02,
    PA_EX_ILLEGAL_DATA_VALUE     = 0x03,
    PA_EX_SLAVE_DEVICE_FAILURE   = 0x04,
    PA_EX_ACKNOWLEDGE            = 0x05,
    PA_EX_SLAVE_DEVICE_BUSY      = 0x06,
    PA_EX_MEMORY_PARITY_ERROR    = 0x08,
    PA_EX_GATEWAY_PATH_UNAVAIL   = 0x0A,
    PA_EX_GATEWAY_TARGET_FAILED  = 0x0B,
} pa_exception_t;

/* ---------------------------------------------------------------------------
 * Callback types — I/O
 * ------------------------------------------------------------------------- */

/**
 * Send raw bytes.
 * @param data   Bytes to send.
 * @param len    Number of bytes.
 * @param userdata  User-supplied pointer (e.g. UART handle, socket fd).
 * @return 0 on success, negative on error.
 */
typedef int (*pa_send_fn)(const uint8_t *data, size_t len, void *userdata);

/** Get a monotonic tick counter (used for receive idle framing/timeout). */
typedef uint32_t (*pa_ticks_fn)(void *userdata);

/**
 * Receive raw bytes.
 *
 * This callback MUST be non-blocking. It should copy whatever is currently
 * available (up to max_len) into data and return the number of bytes copied.
 * A return of 0 means "no data available right now" — the caller must poll
 * again later; it is NOT itself a timeout. Negative on error.
 */
typedef int (*pa_recv_fn)(uint8_t *data, size_t max_len, void *userdata);

/* ---------------------------------------------------------------------------
 * Callback types — register access (slave mode)
 * ------------------------------------------------------------------------- */

/** Read coils/discrete inputs. Fill values[] with count elements. */
typedef int (*pa_read_coils_fn)(           uint16_t addr, uint16_t count, uint8_t  *values, void *userdata);
typedef int (*pa_read_discrete_inputs_fn)(  uint16_t addr, uint16_t count, uint8_t  *values, void *userdata);
typedef int (*pa_read_holding_registers_fn)(uint16_t addr, uint16_t count, uint16_t *values, void *userdata);
typedef int (*pa_read_input_registers_fn)(  uint16_t addr, uint16_t count, uint16_t *values, void *userdata);

/** Write single coil/register. */
typedef int (*pa_write_single_coil_fn)(         uint16_t addr, uint8_t  value, void *userdata);
typedef int (*pa_write_single_register_fn)(     uint16_t addr, uint16_t value, void *userdata);

/** Write multiple coils/registers. */
typedef int (*pa_write_multiple_coils_fn)(      uint16_t addr, uint16_t count, const uint8_t  *values, void *userdata);
typedef int (*pa_write_multiple_registers_fn)(  uint16_t addr, uint16_t count, const uint16_t *values, void *userdata);

/* ---------------------------------------------------------------------------
 * Initialization
 * ------------------------------------------------------------------------- */

/**
 * Initialize a MODBUS context.
 * Must be called before any other function. Default framer is PA_FRAMER_RTU.
 */
void pa_modbus_init(pa_modbus_t *ctx);

/* ---------------------------------------------------------------------------
 * Framer configuration
 * ------------------------------------------------------------------------- */

/** Select the transport framer (default: PA_FRAMER_RTU). */
void pa_modbus_set_framer(pa_modbus_t *ctx, pa_framer_t framer);

/** Get the currently selected framer. */
pa_framer_t pa_modbus_get_framer(const pa_modbus_t *ctx);

/* ---------------------------------------------------------------------------
 * Buffer configuration
 * ------------------------------------------------------------------------- */

/** Set the transmit buffer. */
void pa_modbus_set_txbuf(pa_modbus_t *ctx, uint8_t *buf, size_t size);

/** Set the receive buffer. */
void pa_modbus_set_rxbuf(pa_modbus_t *ctx, uint8_t *buf, size_t size);

/* ---------------------------------------------------------------------------
 * Slave / unit identifier
 * ------------------------------------------------------------------------- */

/** Set the local slave/unit identifier (default: 0xFF, respond to all). */
void   pa_modbus_set_slave(pa_modbus_t *ctx, uint8_t slave);

/** Get the local slave/unit identifier. */
uint8_t pa_modbus_get_slave(const pa_modbus_t *ctx);

/* ---------------------------------------------------------------------------
 * Discovery address (secondary listen address)
 * ------------------------------------------------------------------------- */

/**
 * Set a secondary listen address (non-standard).
 * When non-zero, the slave will also respond to requests addressed to this
 * address in addition to its primary slave address. This is useful for
 * discovery protocols using reserved MODBUS addresses (0xF8-0xFF).
 * Default: 0 (disabled).
 */
void   pa_modbus_set_discovery_addr(pa_modbus_t *ctx, uint8_t addr);

/** Get the secondary listen address. 0 means disabled. */
uint8_t pa_modbus_get_discovery_addr(const pa_modbus_t *ctx);

/* ---------------------------------------------------------------------------
 * I/O callback registration
 * ------------------------------------------------------------------------- */

/** Register the send callback. */
void pa_modbus_set_send_cb(pa_modbus_t *ctx, pa_send_fn send, void *userdata);

/** Register the receive callback. */
void pa_modbus_set_recv_cb(pa_modbus_t *ctx, pa_recv_fn recv, void *userdata);

/* ---------------------------------------------------------------------------
 * Register callback registration (slave mode)
 * ------------------------------------------------------------------------- */

void pa_modbus_set_read_coils_cb(            pa_modbus_t *ctx, pa_read_coils_fn cb,            void *userdata);
void pa_modbus_set_read_discrete_inputs_cb(   pa_modbus_t *ctx, pa_read_discrete_inputs_fn cb,   void *userdata);
void pa_modbus_set_read_holding_registers_cb( pa_modbus_t *ctx, pa_read_holding_registers_fn cb, void *userdata);
void pa_modbus_set_read_input_registers_cb(   pa_modbus_t *ctx, pa_read_input_registers_fn cb,   void *userdata);
void pa_modbus_set_write_single_coil_cb(      pa_modbus_t *ctx, pa_write_single_coil_fn cb,      void *userdata);
void pa_modbus_set_write_single_register_cb(  pa_modbus_t *ctx, pa_write_single_register_fn cb,  void *userdata);
void pa_modbus_set_write_multiple_coils_cb(   pa_modbus_t *ctx, pa_write_multiple_coils_fn cb,   void *userdata);
void pa_modbus_set_write_multiple_registers_cb(pa_modbus_t *ctx, pa_write_multiple_registers_fn cb, void *userdata);

/* ---------------------------------------------------------------------------
 * Master mode — building requests
 * ------------------------------------------------------------------------- */

int pa_modbus_build_read_coils(            pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_read_discrete_inputs(   pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_read_holding_registers( pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_read_input_registers(   pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_write_single_coil(      pa_modbus_t *ctx, uint16_t addr, uint8_t value);
int pa_modbus_build_write_single_register(  pa_modbus_t *ctx, uint16_t addr, uint16_t value);
int pa_modbus_build_write_multiple_coils(   pa_modbus_t *ctx, uint16_t addr, const uint8_t *values, uint16_t count);
int pa_modbus_build_write_multiple_registers(pa_modbus_t *ctx, uint16_t addr, const uint16_t *values, uint16_t count);
int pa_modbus_build_report_slave_id(pa_modbus_t *ctx);
int pa_modbus_build_mask_write_register(pa_modbus_t *ctx, uint16_t addr, uint16_t and_mask, uint16_t or_mask);
int pa_modbus_build_read_write_registers(pa_modbus_t *ctx, uint16_t read_addr, uint16_t read_count,
                                         uint16_t write_addr, const uint16_t *write_values, uint16_t write_count);

/** Get pointer to the framed TX buffer after build. */
const uint8_t *pa_modbus_tx_buf(const pa_modbus_t *ctx);

/** Get length of the framed TX buffer after build. */
size_t         pa_modbus_tx_len(const pa_modbus_t *ctx);

/* ---------------------------------------------------------------------------
 * Master mode — parsing responses
 * ------------------------------------------------------------------------- */

/**
 * Feed received bytes into the response parser.
 * @return PA_OK on complete valid response, >0 if still waiting for data,
 *         negative pa_error_t on error.
 */
int pa_modbus_master_feed(pa_modbus_t *ctx, const uint8_t *data, size_t len);

/** Get the last error code. */
pa_error_t pa_modbus_get_error(const pa_modbus_t *ctx);

/** Get the exception code from the last parsed response. */
uint8_t    pa_modbus_get_exception(const pa_modbus_t *ctx);

/** Get a coil value by index from the last parsed response. */
uint8_t    pa_modbus_get_coil(const pa_modbus_t *ctx, uint16_t idx);

/** Get a register value by index from the last parsed response. */
uint16_t   pa_modbus_get_register(const pa_modbus_t *ctx, uint16_t idx);

/* ---------------------------------------------------------------------------
 * Slave mode — parsing requests
 * ------------------------------------------------------------------------- */

/**
 * Feed received bytes into the request parser.
 * @return PA_OK on complete valid request, >0 if still waiting for data,
 *         negative pa_error_t on error.
 */
int pa_modbus_slave_feed(pa_modbus_t *ctx, const uint8_t *data, size_t len);

/** Get the function code from the parsed request. */
uint8_t        pa_modbus_slave_function(const pa_modbus_t *ctx);

/** Get the starting address from the parsed request. */
uint16_t       pa_modbus_slave_addr(const pa_modbus_t *ctx);

/** Get the count from the parsed request. */
uint16_t       pa_modbus_slave_count(const pa_modbus_t *ctx);

/** Get pointer to coil data from the parsed write request. */
const uint8_t *pa_modbus_slave_coil_data(const pa_modbus_t *ctx);

/** Get pointer to register data from the parsed write request. */
const uint16_t *pa_modbus_slave_reg_data(const pa_modbus_t *ctx);

/* ---------------------------------------------------------------------------
 * Slave mode — building responses
 * ------------------------------------------------------------------------- */

/** Build the response for the parsed request using registered callbacks. */
int pa_modbus_slave_respond(pa_modbus_t *ctx);

/** Build an exception response. */
int pa_modbus_slave_respond_error(pa_modbus_t *ctx, uint8_t exception_code);

/* ---------------------------------------------------------------------------
 * Convenience I/O helpers
 * ------------------------------------------------------------------------- */

 /** Send the TX buffer using the registered send callback. */
int pa_modbus_send(pa_modbus_t *ctx);

/**
 * Non-blocking receive: accumulate bytes into ctx->rxbuf until a complete
 * framed message has been assembled, then parse and dispatch it.
 *
 * The receive callback is drained once per call (it must be non-blocking).
 * Partial frames are retained across calls so a message received in chunks is
 * assembled over successive polls. Only once the full frame is present is it
 * parsed and dispatched to the master/slave parser.
 *
 * @return PA_OK      a complete frame was parsed (call pa_modbus_slave_respond
 *                    in slave mode, or read the response in master mode).
 * @return PA_WAIT    no complete frame yet — keep polling (not an error).
 * @return PA_ERR_TIMEOUT  idle timeout elapsed on a partial/stale frame; the
 *                    accumulator has been discarded and the receiver resynced.
 * @return negative   hard error (CRC, protocol, buffer overflow, etc.).
 *                    The accumulator is reset so the next poll starts fresh.
 */
int pa_modbus_recv(pa_modbus_t *ctx);

/** Reset the receive frame accumulator (e.g. before starting a new transaction). */
void pa_modbus_rx_reset(pa_modbus_t *ctx);

/**
 * Send raw bytes with framing applied.
 * Packs the data as the PDU portion; the framer adds slave address + CRC (RTU)
 * or MBAP header (TCP).
 * @param ctx       The MODBUS context.
 * @param pdu       Raw PDU bytes to send (function code + payload).
 * @param pdu_len   Length of the PDU.
 * @return Framed length on success, or negative pa_error_t on failure.
 */
int pa_modbus_send_raw(pa_modbus_t *ctx, const uint8_t *pdu, size_t pdu_len);

/**
 * Non-blocking raw receive: accumulate bytes until a CRC-valid framed message
 * is present, then strip framing and copy the PDU out.
 *
 * @param ctx       The MODBUS context.
 * @param pdu_buf   Buffer to receive the PDU (framing stripped).
 * @param pdu_len   In: capacity of pdu_buf. Out: actual PDU length received.
 * @return PA_OK on success, PA_WAIT while still assembling (keep polling),
 *         or negative pa_error_t on failure/timeout.
 */
int pa_modbus_recv_raw(pa_modbus_t *ctx, uint8_t *pdu_buf, size_t *pdu_len);

/* ---------------------------------------------------------------------------
 * Receive state control (role, timing, framing)
 * ------------------------------------------------------------------------- */

/** Set the receive/parse role (master or slave). */
void pa_modbus_set_mode(pa_modbus_t *ctx, pa_modbus_mode_t mode);

/** Get the configured receive/parse role. */
pa_modbus_mode_t pa_modbus_get_mode(const pa_modbus_t *ctx);

/**
 * Set a monotonic tick callback used to detect receive idle gaps / timeouts.
 * Without it, partial or stale frames can never time out (they keep being
 * accumulated); a full frame is still detected regardless.
 */
void pa_modbus_set_ticks_cb(pa_modbus_t *ctx, pa_ticks_fn ticks, void *userdata);

/**
 * Set the idle timeout in ticks. If no new bytes arrive for this many ticks
 * while a frame is being assembled, the accumulated bytes are treated as a
 * stale/partial frame, discarded, and pa_modbus_recv()/pa_modbus_recv_raw()
 * return PA_ERR_TIMEOUT. For Modbus RTU this should approximate the 3.5-
 * character inter-frame silent interval (plus margin).
 */
void pa_modbus_set_rx_idle_timeout(pa_modbus_t *ctx, uint32_t ticks);


/* ---------------------------------------------------------------------------
 * CRC-16
 * ------------------------------------------------------------------------- */

/** Compute MODBUS CRC-16 over data. */
uint16_t pa_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* PAMODBUS_H */