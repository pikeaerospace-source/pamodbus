# pamodbus — Lightweight MODBUS Library for Embedded Systems

**pamodbus** is a minimal, portable MODBUS protocol library written in C, designed for non-ASCII MODBUS only. It targets resource-constrained microcontrollers and embedded systems where every byte counts.

## Architecture

The library is structured in three layers:

```
┌──────────────────────────────────────────────┐
│              PDU Core (shared)                │
│  Build/parse function code + data payload     │
│  (FC01–FC18, all standard function codes)     │
├──────────────────────────────────────────────┤
│           Framer Layer (pluggable)            │
│  RTU: [Slave Addr: 1B] [PDU] [CRC-16: 2B]   │
│  TCP: [MBAP Header: 7B] [PDU]                │
├──────────────────────────────────────────────┤
│        User I/O Callbacks (send/recv)         │
│  UART, TCP socket, etc. — library never       │
│  touches hardware, just calls your callbacks  │
└──────────────────────────────────────────────┘
```

- **PDU Core** — Function code + data payload. Identical for RTU and TCP.
- **Framer** — Wraps/unwraps the PDU with transport-specific framing. Pluggable at runtime.
- **I/O Callbacks** — The consumer provides send/receive functions. The library does not know or care whether the transport is UART, TCP socket, SPI, or semaphore-backed shared memory.

## Design Principles

- **Non-ASCII MODBUS only** — RTU framing with CRC-16, or TCP framing with MBAP header. No ASCII mode.
- **Per-connection state** — All state is held in a `pa_modbus_t` struct. Fully reentrant.
- **Callbacks for I/O** — The library never touches hardware directly. Send/receive callbacks bridge the real world.
- **Callbacks for register access** — Slave mode uses callbacks to read/write coils, discrete inputs, holding registers, and input registers. The callbacks enforce address boundaries.
- **User-supplied memory** — The consumer provides the `pa_modbus_t` context, TX buffer, and RX buffer. The library performs no allocations of any kind.
- **Dual role** — Can be built as master-only, slave-only, or both. Controlled by preprocessor defines.
- **Pluggable framer** — Select RTU or TCP framing at runtime. The framer is a lightweight wrapper around the shared PDU core.
- **C99 compatible** — Broad compiler support, from GCC to IAR to ARMCC.
- **Public prefix `pa_`** — Every public function, type, and macro begins with `pa_`.
- **First parameter always `pa_modbus_t *`** — Consistent, predictable API.

## Build Configuration

### Role Selection

Define one of the following at compile time to select the role:

| Define | Role |
|--------|------|
| `PA_MASTER` | Master mode only |
| `PA_SLAVE` | Slave mode only |
| *(neither)* | Both master and slave (default) |

### Framer Selection

The framer is selected at runtime via `pa_modbus_set_framer()`. Both framers are always compiled in by default. To exclude TCP support and save code space:

| Define | Effect |
|--------|--------|
| `PA_NO_FRAMER_TCP` | Exclude TCP framer (RTU only) |
| `PA_NO_FRAMER_RTU` | Exclude RTU framer (TCP only) |
| *(neither)* | Both framers included (default) |

## Types

### `pa_modbus_t`

Opaque connection context. All state is internal. The consumer allocates the struct and initializes it with `pa_modbus_init()`.

```c
typedef struct pa_modbus pa_modbus_t;
```

### `pa_framer_t`

```c
typedef enum {
    PA_FRAMER_RTU,   // MODBUS RTU: slave addr + CRC-16
    PA_FRAMER_TCP,   // MODBUS TCP: MBAP header (7 bytes)
} pa_framer_t;
```

### `pa_error_t`

```c
typedef enum {
    PA_OK                =  0,  // Success
    PA_ERR_CRC           = -1,  // CRC mismatch (RTU only)
    PA_ERR_TIMEOUT       = -2,  // Response timeout
    PA_ERR_INVALID_SLAVE = -3,  // Response slave/unit ID mismatch
    PA_ERR_BAD_PARAM     = -4,  // Invalid parameter (address, count, etc.)
    PA_ERR_BUFFER_FULL   = -5,  // TX or RX buffer overflow
    PA_ERR_EXCEPTION     = -6,  // MODBUS exception received (check exception code)
    PA_ERR_PROTOCOL      = -7,  // Malformed frame
    PA_ERR_STATE         = -8,  // Invalid state for requested operation
    PA_ERR_CALLBACK      = -9,  // Callback returned error
} pa_error_t;
```

### `pa_exception_t`

```c
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
```

### Callback Types

#### I/O Callbacks

```c
// Send raw bytes. Return 0 on success, negative on error.
typedef int (*pa_send_fn)(const uint8_t *data, size_t len, void *userdata);

// Receive raw bytes. Return number of bytes received, 0 on timeout, negative on error.
typedef int (*pa_recv_fn)(uint8_t *data, size_t max_len, void *userdata);
```

#### Register Access Callbacks (Slave Mode)

```c
// Read callbacks: fill values[] array with count elements. Return 0 on success.
typedef int (*pa_read_coils_fn)(            uint16_t addr, uint16_t count, uint8_t  *values, void *userdata);
typedef int (*pa_read_discrete_inputs_fn)(   uint16_t addr, uint16_t count, uint8_t  *values, void *userdata);
typedef int (*pa_read_holding_registers_fn)( uint16_t addr, uint16_t count, uint16_t *values, void *userdata);
typedef int (*pa_read_input_registers_fn)(   uint16_t addr, uint16_t count, uint16_t *values, void *userdata);

// Write callbacks: write count elements from values[]. Return 0 on success.
typedef int (*pa_write_single_coil_fn)(          uint16_t addr, uint8_t  value, void *userdata);
typedef int (*pa_write_single_register_fn)(      uint16_t addr, uint16_t value, void *userdata);
typedef int (*pa_write_multiple_coils_fn)(       uint16_t addr, uint16_t count, const uint8_t  *values, void *userdata);
typedef int (*pa_write_multiple_registers_fn)(   uint16_t addr, uint16_t count, const uint16_t *values, void *userdata);
```

## API Reference

### Initialization

```c
void pa_modbus_init(pa_modbus_t *ctx);
```

Initialize a MODBUS context. The consumer is responsible for allocating the `pa_modbus_t` struct (static, global, stack, or heap — the library does not care). Default framer is `PA_FRAMER_RTU`. Must be called before any other function.

### Framer Configuration

```c
void pa_modbus_set_framer(pa_modbus_t *ctx, pa_framer_t framer);
```

Select the transport framer. Must be called before any build or feed operations. Default: `PA_FRAMER_RTU`.

```c
pa_framer_t pa_modbus_get_framer(const pa_modbus_t *ctx);
```

Get the currently selected framer.

### Buffer Configuration

```c
void pa_modbus_set_txbuf(pa_modbus_t *ctx, uint8_t *buf, size_t size);
void pa_modbus_set_rxbuf(pa_modbus_t *ctx, uint8_t *buf, size_t size);
```

Set the TX and RX buffers. The library references these pointers — it does **not** copy or own the memory. The caller must ensure the buffers remain valid for the lifetime of the context.

Minimum recommended buffer size: 256 bytes (sufficient for most MODBUS frames).

### Slave / Unit Identifier

In RTU mode, this is the slave address. In TCP mode, this is the unit identifier in the MBAP header.

```c
void   pa_modbus_set_slave(pa_modbus_t *ctx, uint8_t slave);
uint8_t pa_modbus_get_slave(const pa_modbus_t *ctx);
```

Set/get the local slave/unit identifier. In master mode, this is the target used when building requests. In slave mode, this is the identifier the slave responds to. Default: `0xFF` (respond to all).

### I/O Callback Registration

```c
void pa_modbus_set_send_cb(pa_modbus_t *ctx, pa_send_fn send, void *userdata);
void pa_modbus_set_recv_cb(pa_modbus_t *ctx, pa_recv_fn recv, void *userdata);
```

Register the send and receive callbacks. The `userdata` pointer is passed through to the callback unchanged — typically a file descriptor, UART handle, socket, or device struct.

### Register Callback Registration (Slave Mode)

```c
void pa_modbus_set_read_coils_cb(            pa_modbus_t *ctx, pa_read_coils_fn cb,            void *userdata);
void pa_modbus_set_read_discrete_inputs_cb(   pa_modbus_t *ctx, pa_read_discrete_inputs_fn cb,   void *userdata);
void pa_modbus_set_read_holding_registers_cb( pa_modbus_t *ctx, pa_read_holding_registers_fn cb, void *userdata);
void pa_modbus_set_read_input_registers_cb(   pa_modbus_t *ctx, pa_read_input_registers_fn cb,   void *userdata);
void pa_modbus_set_write_single_coil_cb(      pa_modbus_t *ctx, pa_write_single_coil_fn cb,      void *userdata);
void pa_modbus_set_write_single_register_cb(  pa_modbus_t *ctx, pa_write_single_register_fn cb,  void *userdata);
void pa_modbus_set_write_multiple_coils_cb(   pa_modbus_t *ctx, pa_write_multiple_coils_fn cb,   void *userdata);
void pa_modbus_set_write_multiple_registers_cb(pa_modbus_t *ctx, pa_write_multiple_registers_fn cb, void *userdata);
```

Each callback receives its own `userdata` pointer, allowing different contexts for different register spaces.

### Master Mode — Building Requests

Each build function constructs a MODBUS PDU (function code + data) in the internal buffer, then the active framer wraps it with the transport-specific framing (RTU: slave addr + CRC, TCP: MBAP header). Returns the total frame length on success, or a negative `pa_error_t` on failure.

```c
int pa_modbus_build_read_coils(            pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_read_discrete_inputs(   pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_read_holding_registers( pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_read_input_registers(   pa_modbus_t *ctx, uint16_t addr, uint16_t count);
int pa_modbus_build_write_single_coil(      pa_modbus_t *ctx, uint16_t addr, uint8_t value);
int pa_modbus_build_write_single_register(  pa_modbus_t *ctx, uint16_t addr, uint16_t value);
int pa_modbus_build_write_multiple_coils(   pa_modbus_t *ctx, uint16_t addr, const uint8_t *values, uint16_t count);
int pa_modbus_build_write_multiple_registers(pa_modbus_t *ctx, uint16_t addr, const uint16_t *values, uint16_t count);
```

After building, retrieve the complete framed bytes for transmission:

```c
const uint8_t *pa_modbus_tx_buf(const pa_modbus_t *ctx);
size_t         pa_modbus_tx_len(const pa_modbus_t *ctx);
```

### Master Mode — Parsing Responses

```c
int pa_modbus_master_feed(pa_modbus_t *ctx, const uint8_t *data, size_t len);
```

Feed received bytes into the response parser. The active framer first strips the transport wrapper, then the PDU parser validates the response. Returns:
- `PA_OK` — Complete and valid response parsed.
- `PA_ERR_CRC` — CRC mismatch (RTU only).
- `PA_ERR_INVALID_SLAVE` — Response slave/unit ID doesn't match request.
- `PA_ERR_EXCEPTION` — MODBUS exception received. Call `pa_modbus_get_exception()` for the code.
- `PA_ERR_PROTOCOL` — Malformed frame.
- `> 0` — Still waiting for more data (number of bytes consumed so far).

After a successful parse, extract the data:

```c
pa_error_t     pa_modbus_get_error(const pa_modbus_t *ctx);
uint8_t        pa_modbus_get_exception(const pa_modbus_t *ctx);
uint8_t        pa_modbus_get_coil(const pa_modbus_t *ctx, uint16_t idx);
uint16_t       pa_modbus_get_register(const pa_modbus_t *ctx, uint16_t idx);
```

### Slave Mode — Parsing Requests

```c
int pa_modbus_slave_feed(pa_modbus_t *ctx, const uint8_t *data, size_t len);
```

Feed received bytes into the request parser. The active framer first strips the transport wrapper, then the PDU parser validates the request. Return values follow the same convention as `pa_modbus_master_feed()`.

On complete request received, examine the request:

```c
uint8_t        pa_modbus_slave_function(const pa_modbus_t *ctx);
uint16_t       pa_modbus_slave_addr(const pa_modbus_t *ctx);
uint16_t       pa_modbus_slave_count(const pa_modbus_t *ctx);
const uint8_t *pa_modbus_slave_coil_data(const pa_modbus_t *ctx);
const uint16_t *pa_modbus_slave_reg_data(const pa_modbus_t *ctx);
```

### Slave Mode — Building Responses

```c
int pa_modbus_slave_respond(pa_modbus_t *ctx);
```

Automatically builds the appropriate response by invoking the registered register access callbacks. The PDU is built, then the active framer wraps it. Returns the total frame length on success, or a negative `pa_error_t`.

```c
int pa_modbus_slave_respond_error(pa_modbus_t *ctx, uint8_t exception_code);
```

Build an exception response. Returns the total frame length on success.

After building, retrieve the response bytes:

```c
const uint8_t *pa_modbus_tx_buf(const pa_modbus_t *ctx);
size_t         pa_modbus_tx_len(const pa_modbus_t *ctx);
```

### Convenience I/O Helpers

```c
int pa_modbus_send(pa_modbus_t *ctx);
```

Send the contents of the TX buffer using the registered send callback. Returns `PA_OK` on success, or `PA_ERR_CALLBACK` if the callback failed.

```c
int pa_modbus_recv(pa_modbus_t *ctx);
```

Receive data using the registered recv callback and feed it to the parser (master or slave depending on mode). Returns the parser result.

## Usage Examples

### Master: Read Holding Registers (RTU over UART)

```c
#include "pamodbus.h"

static pa_modbus_t mb;
static uint8_t txbuf[256];
static uint8_t rxbuf[256];

static int uart_send(const uint8_t *data, size_t len, void *userdata) {
    // Write len bytes to UART
    return 0;
}

static int uart_recv(uint8_t *data, size_t max_len, void *userdata) {
    // Read up to max_len bytes from UART
    return bytes_read;
}

void read_registers_rtu(void) {
    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_send_cb(&mb, uart_send, NULL);
    pa_modbus_set_recv_cb(&mb, uart_recv, NULL);
    pa_modbus_set_slave(&mb, 0x01);

    // Build request: read 3 holding registers starting at address 0
    int len = pa_modbus_build_read_holding_registers(&mb, 0, 3);
    if (len < 0) { /* handle error */ }

    // Send request
    if (pa_modbus_send(&mb) != PA_OK) { /* handle error */ }

    // Receive and parse response
    int ret;
    do {
        ret = pa_modbus_recv(&mb);
    } while (ret > 0);  // Still waiting for more data

    if (ret == PA_OK) {
        uint16_t reg0 = pa_modbus_get_register(&mb, 0);
        uint16_t reg1 = pa_modbus_get_register(&mb, 1);
        uint16_t reg2 = pa_modbus_get_register(&mb, 2);
    }
}
```

### Master: Read Holding Registers (TCP over socket)

```c
#include "pamodbus.h"
#include <sys/socket.h>

static pa_modbus_t mb;
static uint8_t txbuf[256];
static uint8_t rxbuf[256];

static int tcp_send(const uint8_t *data, size_t len, void *userdata) {
    int fd = *(int *)userdata;
    return send(fd, data, len, 0) == (ssize_t)len ? 0 : -1;
}

static int tcp_recv(uint8_t *data, size_t max_len, void *userdata) {
    int fd = *(int *)userdata;
    ssize_t n = recv(fd, data, max_len, 0);
    return (n > 0) ? (int)n : (int)n;  // 0 = closed, -1 = error
}

void read_registers_tcp(int sockfd) {
    // sockfd is already connected to the MODBUS TCP server

    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_TCP);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_send_cb(&mb, tcp_send, &sockfd);
    pa_modbus_set_recv_cb(&mb, tcp_recv, &sockfd);
    pa_modbus_set_slave(&mb, 0x01);  // Unit identifier in MBAP

    // Build request — framer adds MBAP header automatically
    int len = pa_modbus_build_read_holding_registers(&mb, 0, 3);
    if (len < 0) { /* handle error */ }

    // Send request
    if (pa_modbus_send(&mb) != PA_OK) { /* handle error */ }

    // Receive and parse response
    int ret;
    do {
        ret = pa_modbus_recv(&mb);
    } while (ret > 0);

    if (ret == PA_OK) {
        uint16_t reg0 = pa_modbus_get_register(&mb, 0);
        uint16_t reg1 = pa_modbus_get_register(&mb, 1);
        uint16_t reg2 = pa_modbus_get_register(&mb, 2);
    }
}
```

### Slave: Respond to Requests (RTU over UART)

```c
#include "pamodbus.h"

static pa_modbus_t mb;
static uint8_t txbuf[256];
static uint8_t rxbuf[256];

// Backing store
static uint16_t holding_regs[100];

static int read_holding_cb(uint16_t addr, uint16_t count, uint16_t *values, void *userdata) {
    if (addr + count > 100) return -1;  // Enforce boundary
    for (uint16_t i = 0; i < count; i++)
        values[i] = holding_regs[addr + i];
    return 0;
}

static int write_holding_cb(uint16_t addr, uint16_t count, const uint16_t *values, void *userdata) {
    if (addr + count > 100) return -1;  // Enforce boundary
    for (uint16_t i = 0; i < count; i++)
        holding_regs[addr + i] = values[i];
    return 0;
}

void slave_example(void) {
    pa_modbus_init(&mb);
    pa_modbus_set_framer(&mb, PA_FRAMER_RTU);
    pa_modbus_set_txbuf(&mb, txbuf, sizeof(txbuf));
    pa_modbus_set_rxbuf(&mb, rxbuf, sizeof(rxbuf));
    pa_modbus_set_send_cb(&mb, uart_send, NULL);
    pa_modbus_set_recv_cb(&mb, uart_recv, NULL);
    pa_modbus_set_slave(&mb, 0x01);

    pa_modbus_set_read_holding_registers_cb(&mb, read_holding_cb, NULL);
    pa_modbus_set_write_multiple_registers_cb(&mb, write_holding_cb, NULL);

    while (1) {
        int ret = pa_modbus_recv(&mb);
        if (ret == PA_OK) {
            ret = pa_modbus_slave_respond(&mb);
            if (ret > 0)
                pa_modbus_send(&mb);
        }
    }
}
```

## Supported MODBUS Function Codes

| Code | Name | Master | Slave |
|------|------|--------|-------|
| 0x01 | Read Coils | ✓ | ✓ |
| 0x02 | Read Discrete Inputs | ✓ | ✓ |
| 0x03 | Read Holding Registers | ✓ | ✓ |
| 0x04 | Read Input Registers | ✓ | ✓ |
| 0x05 | Write Single Coil | ✓ | ✓ |
| 0x06 | Write Single Register | ✓ | ✓ |
| 0x07 | Read Exception Status | ✓ | ✓ |
| 0x08 | Diagnostic | ✓ | ✓ |
| 0x0B | Get Com Event Counter | ✓ | ✓ |
| 0x0C | Get Com Event Log | ✓ | ✓ |
| 0x0F | Write Multiple Coils | ✓ | ✓ |
| 0x10 | Write Multiple Registers | ✓ | ✓ |
| 0x11 | Report Server ID | ✓ | ✓ |
| 0x14 | Read File Record | ✓ | ✓ |
| 0x15 | Write File Record | ✓ | ✓ |
| 0x16 | Mask Write Register | ✓ | ✓ |
| 0x17 | Read/Write Multiple Registers | ✓ | ✓ |
| 0x18 | Read FIFO Queue | ✓ | ✓ |

## CRC-16

The library provides the MODBUS CRC-16 function for external use if needed:

```c
uint16_t pa_crc16(const uint8_t *data, size_t len);
```

## Implementation Details

### File Structure

```
pamodbus/
├── include/
│   └── pamodbus.h              # Public API header
├── src/
│   ├── pamodbus_internal.h     # Internal structs and helpers
│   ├── pamodbus.c              # Core init, config, callbacks, I/O helpers
│   ├── pdu.c                   # PDU build/parse for all 18 function codes
│   ├── framer_rtu.c            # RTU framing: slave addr + CRC wrapper
│   ├── framer_tcp.c            # TCP framing: MBAP header wrapper
│   └── crc16.c                 # CRC-16 MODBUS implementation
├── test/
│   └── test_pamodbus.c         # Basic unit tests
├── CMakeLists.txt              # Build system
├── LICENSE                     # MIT license
└── README.md                   # This file
```

### Internal Architecture

The framer is implemented via a struct of function pointers (`pa_framer_ops_t`), making it truly pluggable at runtime without `if/else` dispatch on an enum. Each framer (RTU, TCP) exposes:

- **wrap**: Insert framing bytes around a PDU in the TX buffer.
- **unwrap**: Strip framing bytes from received data, returning the PDU.
- **overhead**: Maximum bytes the framer adds (RTU: 3, TCP: 7).

The `pa_modbus_t` internal struct holds:

- TX/RX buffer pointers and lengths
- Active framer type and its function table
- Slave/unit identifier
- Send/recv callbacks with userdata pointers
- Register access callbacks (one per type) with per-callback userdata
- Build state (currently building a request/response)
- Parse state (currently accumulating received bytes for the active framer)
- Last parse result (function code, address, count, coil data, register data, exception code)

### Framing Overhead

| Framer | Overhead | Details |
|--------|----------|---------|
| RTU    | 3 bytes  | 1 byte slave addr + 2 bytes CRC-16 |
| TCP    | 7 bytes  | 2 trans ID + 2 proto ID + 2 length + 1 unit ID |

### Build System

The `CMakeLists.txt` supports the following compile-time definitions:

| Define | Effect |
|--------|--------|
| `PA_MASTER` | Include only master-mode functions |
| `PA_SLAVE` | Include only slave-mode functions |
| (neither) | Both master and slave included (default) |
| `PA_NO_FRAMER_TCP` | Exclude TCP framer (RTU only) |
| `PA_NO_FRAMER_RTU` | Exclude RTU framer (TCP only) |
| (neither) | Both framers included (default) |

The build system automatically passes `-DPA_MASTER`, `-DPA_SLAVE`, etc. to the compiler when the corresponding CMake options are enabled.

## License

MIT — see LICENSE file for details.
