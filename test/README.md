# pamodbus — Unit Test Suite

This directory contains the unit test suite for the **pamodbus** library — a lightweight, portable MODBUS protocol library for embedded systems.

## Test File

- **`test_pamodbus.c`** — The complete test suite

## Test Architecture

The test suite is written in plain C with no external test framework:

- **`TEST_ASSERT` macros** — Simple pass/fail assertions with `tests_passed`/`tests_failed` counters
- **Direct byte-level frame construction** — Raw MODBUS RTU and TCP frames are built manually and fed to the parser via `pa_modbus_master_feed()` and `pa_modbus_slave_feed()`
- **No I/O callbacks needed** — Tests bypass the send/recv callbacks by directly constructing frames and calling feed functions, which is possible because pamodbus is a pure protocol library
- **Real register callbacks** — Slave mode tests register actual read/write callbacks backed by in-memory arrays
- **Standalone host executable** — Compiled with `gcc` on the host, no embedded hardware required

## Test Coverage

### 1. CRC-16 Tests — 5 checks

| Test | Description |
|------|-------------|
| CRC for read holding regs request | Known frame 0x01,0x03,0x00,0x00,0x00,0x01 → 0x0A84 |
| CRC for another known frame | 0x11,0x03,0x00,0x6B,0x00,0x03 → 0x8776 |
| CRC of empty data | Returns 0xFFFF |
| CRC of single zero byte | 0x00 → 0x40BF |
| CRC of single 0xFF byte | 0xFF → 0x00FF |

### 2. PDU Build Tests — 18 checks

| Test | Description |
|------|-------------|
| Read Holding Registers (FC03) | Frame length, slave addr, FC, count, CRC |
| Read Coils (FC01) | Frame length, FC, addr, count |
| Read Discrete Inputs (FC02) | Frame length, FC |
| Read Input Registers (FC04) | Frame length, FC, addr, count |
| Write Single Coil ON (FC05) | Frame length, FC, ON value (0xFF00) |
| Write Single Coil OFF (FC05) | Frame length, FC, OFF value (0x0000) |
| Write Single Register (FC06) | Frame length, FC, value |
| Write Multiple Coils (FC0F) | Frame length, FC, byte count, coil data |
| Write Multiple Registers (FC10) | Frame length, FC, byte count, register data |
| Maximum valid parameters | FC01 count=2000, FC03 count=125 |
| Invalid parameters (count=0) | FC03, FC01 return PA_ERR_BAD_PARAM |
| Invalid parameters (count too high) | FC03 count=200, FC01 count=2001, FC04 count=126 |
| Invalid parameters (write count=0) | FC0F, FC10 return PA_ERR_BAD_PARAM |
| Buffer overflow | Small TX buffer returns PA_ERR_BUFFER_FULL |

### 3. TCP Framing Tests — 7 checks

| Test | Description |
|------|-------------|
| TCP frame length | 12 bytes for FC03 request |
| MBAP header fields | Protocol ID=0, Length=6, Unit ID=1 |
| Transaction ID increments | TID increases after each request |
| Valid TCP response | Master feed parses 3 registers correctly |
| Wrong unit ID | Returns PA_ERR_INVALID_SLAVE |
| Non-zero protocol ID | Returns PA_ERR_PROTOCOL |
| Incomplete TCP frame | Returns >0 (need more data) |

### 4. RTU Master Feed Tests — 9 checks

| Test | Description |
|------|-------------|
| Valid RTU response | 3 registers parsed correctly |
| CRC mismatch | Returns PA_ERR_CRC |
| Wrong slave address | Returns PA_ERR_INVALID_SLAVE |
| Incomplete frame | Returns >0 (need more data) |
| Read Coils (FC01) response | 16 coils parsed, bit positions correct |
| Read Discrete Inputs (FC02) response | 8 inputs parsed |
| Read Input Registers (FC04) response | 3 registers parsed |
| Out-of-range coils/registers | Returns 0 |
| Exception response | Returns PA_ERR_EXCEPTION, code extracted |

### 5. Slave Mode Tests — 5 checks

| Test | Description |
|------|-------------|
| Read request (FC03) | Slave feed, function/addr/count extracted, response built with correct register values |
| Write request (FC10) | Slave feed, registers updated, echo response |
| Exception response | `pa_modbus_slave_respond_error()` with ILLEGAL_FUNCTION |

### 6. Coil Operation Tests — 6 checks

| Test | Description |
|------|-------------|
| Write single coil ON (FC05) | Coil set to 1 |
| Write single coil OFF (FC05) | Coil cleared to 0 |
| Write multiple coils (FC0F) | 8 coils written with pattern 0xAA |
| Read coils (FC01) | Response contains correct coil states |

### 7. Userdata Isolation Tests — 1 check

| Test | Description |
|------|-------------|
| Multiple contexts, separate userdata | Two MODBUS contexts with different register stores, verify correct isolation |

### 8. Broadcast Address Tests — 2 checks

| Test | Description |
|------|-------------|
| Master with slave=0xFF accepts any response | Broadcast slave address accepts any response slave |
| Slave with slave=0xFF accepts any request | Slave responds to any incoming slave address |

### 9. Framer Switch Tests — 1 check

| Test | Description |
|------|-------------|
| RTU → TCP → RTU switching | Default RTU, switch to TCP, build TCP frame, switch back to RTU, build RTU frame |

### 10. Discovery Address Tests — 8 checks

| Test | Description |
|------|-------------|
| Set/get discovery address | `pa_modbus_set_discovery_addr()` / `pa_modbus_get_discovery_addr()` |
| Primary address accepted | Request to 0x01 accepted |
| Discovery address (0xFF) accepted | Request to 0xFF accepted |
| Unknown address (0x02) rejected | Returns PA_ERR_INVALID_SLAVE |
| Discovery address disabled | 0xFF rejected after setting to 0 |
| Primary address still works | After disabling discovery |
| TCP discovery address | Unit ID 0xFF accepted via TCP slave_feed |

### 11. Slave Address Tests — 1 check

| Test | Description |
|------|-------------|
| Get/set slave address | Default 0xFF, set to 0x01, set to 0xFF |

## Build & Run

### Prerequisites

- GCC (or any C99-compatible compiler)
- pamodbus library source files (in `_src/pamodbus/`)

### Compilation

```bash
gcc -o test_pamodbus \
    -I_src/pamodbus/include \
    -I_src/pamodbus/src \
    _src/pamodbus/test/test_pamodbus.c \
    _src/pamodbus/src/pamodbus.c \
    _src/pamodbus/src/pdu.c \
    _src/pamodbus/src/framer_rtu.c \
    _src/pamodbus/src/framer_tcp.c \
    _src/pamodbus/src/crc16.c \
    -lm
```

### Run

```bash
./test_pamodbus
```

### Expected Output

```
pamodbus Unit Tests
==================
=== CRC-16 Tests ===
  PASS: CRC for read holding regs request
  ...
=== PDU Build Tests ===
  ...
=== TCP Framing Tests ===
  ...
=== RTU Master Feed Tests ===
  ...
=== Slave Mode Tests ===
  ...
=== Coil Operation Tests ===
  ...
=== Userdata Isolation Tests ===
  ...
=== Broadcast Address Tests ===
  ...
=== Framer Switch Tests ===
  ...
=== Discovery Address Tests ===
  ...
=== Slave Address Tests ===
  ...
========================
Results: X passed, 0 failed
```

Exit code `0` indicates all tests passed. Exit code `1` indicates one or more failures.

## Key Testing Approach

### Bypassing I/O

Unlike typical MODBUS libraries that require serial port or socket I/O, pamodbus tests construct raw protocol frames in memory and feed them directly to the parser functions:

- **Master responses**: Built as byte arrays (RTU with CRC or TCP with MBAP header), fed via `pa_modbus_master_feed()`
- **Slave requests**: Built as byte arrays, fed via `pa_modbus_slave_feed()`

This allows comprehensive protocol-level testing without any hardware dependencies.

### CRC Verification

The test suite verifies CRC correctness in two ways:
1. **Known CRC values** — Pre-computed CRC values for known MODBUS frames
2. **Runtime CRC calculation** — Frames are built with correct CRC, then verified by the parser

### Slave Mode Testing

Slave mode tests register real callback functions backed by in-memory arrays (`test_holding_regs[100]`, `test_coils[32]`). After feeding a request, the test verifies:
- The request was parsed correctly (function, address, count)
- The response was built correctly (frame structure, data values)
- The backing store was updated correctly (for write operations)

### Multiple Context Isolation

The userdata isolation test creates two independent MODBUS contexts with separate register stores and verifies that operations on one context do not affect the other.