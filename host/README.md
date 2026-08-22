# pamodbus Host-side Discovery Tests (RS485 over real serial)

These tools exercise the pamodbus discovery protocol **over a real serial (or
RS485) line** on the host, using the same `pamodbus` / `pamodbus-disco`
libraries the embedded features use. They pair with:

- the embedded target features in `_feature/pamodbus/master/disco` and
  `_feature/pamodbus/slave/disco`, or
- each other, for host-to-host testing.

## What's here

| File | Purpose |
|------|---------|
| `pa_port.h` (up one dir) | Generic RS485 transport interface (`pa_port_t`) |
| `pa_serial.h` / `pa_serial.c` | POSIX `termios` implementation of `pa_port_t` |
| `master_disco.c` | Discovery **master**: scans the bus, assigns+verifies slave IDs |
| `slave_disco.c` | Discovery **slave**: reports identity, adopts assigned ID |
| `shim/` | Host-build shims so `pamodbus-disco`'s `brisc_*` heap/string deps resolve to libc |
| `Makefile` | Builds both tools against the pamodbus + pamodbus-disco sources |

## Build

```bash
make
```

## Usage

```bash
# Master side
./master_disco /dev/ttyUSB0 115200 [scan_seconds]

# Slave side
./slave_disco  /dev/ttyUSB0 115200 [timeout_s]
```

Each takes a serial device path and baud rate. On a half-duplex RS485 adapter
that needs manual direction control, hook `pa_serial_set_dir` / the RTS line
in `pa_serial.c` (`ser_set_dir`); most USB-RS485 adapters auto-manage
transmit-enable.

### Topologies

| Topology | Run |
|----------|-----|
| host → host | `slave_disco` on one adapter, `master_disco` on another (looped) |
| host → target | host tool on USB-RS485 talking to an embedded `master/disco` or `slave/disco` image |
| target → target | two embedded images on the bus |

## Host-to-host loopback test (no hardware)

`socat` can create a virtual serial pair, e.g.:

```bash
socat pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB &
./slave_disco  /tmp/ttyA 115200
./master_disco /tmp/ttyB 115200
```

The slave must be listening before the master scans. The provided harness does
this for you:

```bash
./loopback_test.sh
```

## Findings & notes from bring-up

1. **`PA_DISCO_HOLDING_NREGS` was too small (32).** A slave register-map window
   must serve both the master's verify read of register `PA_DISCO_VERIFY_REG`
   (0) and the discovery block `PA_DISCO_REG_START..+COUNT` (23..34). `32`
   made that impossible; bumped to **64** in `pamodbus-disco.h`. Set the slave
   holding window with `start=0` (verify reg) and populate the discovery IDs
   inside it.

2. **Host slave must keep serving after assignment.** Unlike an embedded slave
   (which never exits), a host `slave_disco` that exits as soon as it is
   assigned will never answer the master's verify read-back, so the master
   cannot complete discovery — this tool keeps serving ~2 s after assignment.

3. **Master verify pacing.** The disco master's `do_state_verify` consumes one
   **non-blocking** `pa_modbus_recv` per attempt and re-sends a fresh read each
   attempt. Over a real (or pty) serial link the response needs the master loop
   to yield between `service()` calls; the ~20 ms cadence here is the
   host analog of an OS yield. The slave demonstrably answers every verify read
   (the slave side PASSes); on the host the master-side verify confirmation is
   sensitive to this pacing and is the remaining piece to click over reliably
   end-to-end. This is a library-level timing characteristic worth polishing.

4. **Generic transport seam.** `pa_port_t` (see `pa_port.h`) lets the **same**
   feature logic run on a host serial port and on real target UART + RS485
   direction GPIO; signatures match pamodbus callbacks so the port members are
   handed straight to `pa_modbus_set_send_cb`/`recv_cb`/`ticks_cb`.

## Clean build

```bash
make clean && make   # builds both binaries, -Wall -Wextra
```