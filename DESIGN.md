# Design Document — Simplified I2C Subsystem Model in C

This is a simplified implementation of a I2C controller and endpoint device model in C. It provides a transaction-level simulation of I2C communication, focusing on the software-visible behavior of the controller and devices rather than electrical waveform accuracy.

---

## 1. Purpose and Scope

This implementation is a **transaction-level functional model** of a simplified I2C
subsystem written in portable C11.

This design implements a transaction-level I2C subsystem model with:
- An MMIO based I2C controller/master
- An I2C bus abstraction
- One or more I2C endpoint/slave devices
- Endpoint internal registers
- START, STOP, WRITE, and READ commands
- Status reporting through BUSY, DONE, ERROR, and RX_VALID bits
- A simple controller state machine

### What it models

| Component        | Description                                                   |
|------------------|---------------------------------------------------------------|
| Controller       | I2C master exposed through 5 MMIO registers                   |
| Bus              | Routes transactions to endpoints by 7-bit address             |
| Endpoint         | I2C slave with a 256-byte register map                        |
| Command protocol | `START`, `STOP`, `WRITE`, `READ` written to the CMD register  |

### What it does *not* model

| Excluded                 | Reason                               |
|--------------------------|--------------------------------------|
| SDA / SCL waveforms      | Transaction-level only               |
| Bit timing / clock       | No cycle accuracy                    |
| Clock stretching         | Simplification                       |
| Multi-master arbitration | Single master assumed                |
| 10-bit addressing        | 7-bit only per spec                  |
| Interrupts / DMA         | Polled-status model                  |

These exclusions are deliberate and documented. See §10 for extension paths.

---

## 2. Requirements

| Requirement                  | Implementation                                             |
|------------------------------|------------------------------------------------------------|
| MMIO controller              | `I2CController` with CONTROL, STATUS, TXDATA, RXDATA, CMD  |
| START/STOP/WRITE/READ        | Implemented through CMD register                           |
| 7-bit addressing             | Address decoded from `TXDATA[7:1]` during START            |
| Read/write transactions      | Endpoint register-pointer model supports both              |
| Status reporting             | `BUSY`, `DONE`, `ERROR`, `RX_VALID`                        |
| Endpoint address             | `I2CEndpoint.address` (7-bit, normalized at init)          |
| Endpoint register map        | 256-byte register array with Device ID, Status, Data       |
| Bus abstraction              | `I2CBus` routes transactions by endpoint address           |
| Multiple slaves              | Up to 8 endpoints (`I2C_MAX_ENDPOINTS`)                    |
| Transaction-level model      | No SDA/SCL waveform, no clock stretching                   |

---

## 3. High-Level Architecture

```text
   +------------------+   MMIO read/write    +----------------------+
   |  Firmware / CPU  |--------------------->|  I2C Controller      |
   |  (test or demo)  |<---------------------|  CTRL/STAT/TX/RX/CMD |
   +------------------+                      +----------+-----------+
                                                        |
                                                        | transaction call
                                                        v
                                              +----------------------+
                                              |  I2C Bus             |
                                              |  (address router)    |
                                              +----+------------+----+
                                                   |            |
                                                   v            v
                                          +-------------+  +-------------+
                                          | Endpoint    |  | Endpoint    |
                                          | addr 0x50   |  | addr 0x68   |
                                          | regs[0..255]|  | regs[0..255]|
                                          +-------------+  +-------------+
```

There are **three layers**, each implemented as C struct with
associated functions:

| Layer      | Struct          | Responsibility                                   |
|------------|-----------------|--------------------------------------------------|
| Controller | `I2CController` | MMIO interface, command sequencing, FSM, status  |
| Bus        | `I2CBus`        | Owns endpoint pointers; routes by 7-bit address  |
| Endpoint   | `I2CEndpoint`   | 256-byte register file, register-pointer access  |

The dependency arrow goes one way: **Controller → Bus → Endpoint**. This
keeps the model easy to reason about and unit-test in isolation.

---

## 4. File Layout

```text
.
├── Makefile                   # C11, -Wall -Wextra -Werror -pedantic
├── README.md                  # User-facing quick reference
├── DESIGN.md                  # This document
├── include/
│   └── i2c_model.h            # Complete public API
├── src/
│   ├── i2c_model.c            # All implementation (controller + bus + endpoint)
│   └── demo.c                 # Annotated runnable demo
└── tests/
    └── test_i2c_model.c       # Assertion-based test suite (9 tests)
```

A single header (`i2c_model.h`) is the only file consumers include.
All implementation detail is private to `i2c_model.c`.

---

## 5. Controller Programmer's Model

The controller is modelled as a memory-mapped peripheral with five 32-bit
registers.

### 4.1 Register Map

| Offset  | Name      | Access | Description                                   |
|--------:|-----------|--------|-----------------------------------------------|
| `0x00`  | `CONTROL` | R/W    | Enable / configuration flags                  |
| `0x04`  | `STATUS`  | R/W1C  | Busy / done / error / rx_valid status flags   |
| `0x08`  | `TXDATA`  | R/W    | Address byte or data byte to transmit         |
| `0x0C`  | `RXDATA`  | R      | Last byte received from an endpoint           |
| `0x10`  | `CMD`     | W      | Command trigger (write-only; reads return 0)  |

### 4.2 CONTROL bits

| Bit | Name     | Meaning                                  |
|----:|----------|------------------------------------------|
| 0   | `ENABLE` | 1 = controller accepts commands          |

Clearing `ENABLE` mid-transaction releases the selected endpoint and returns the FSM to `IDLE`. 
This prevents a "stuck slave" condition if firmware aborts.

### 4.3 STATUS bits

| Bit | Name       | Type    | Cleared by                                 |
|----:|------------|---------|--------------------------------------------|
| 0   | `BUSY`     | live    | hardware only (not W1C-able)               |
| 1   | `DONE`     | sticky  | next command or W1C write                  |
| 2   | `ERROR`    | sticky  | next *successful* command or W1C write     |
| 3   | `RX_VALID` | sticky  | next command or W1C write                  |

**Write-One-to-Clear (W1C):** 
writing a `1` to a sticky STATUS bit clears it;
writing `0` is a no-op. 
`BUSY` cannot be cleared by software. This matches common real-world peripheral designs (ARM CMSIS, RISC-V PLIC, etc.).

**Transient STATUS clearing:** 
at the start of every `execute_command()`,
`DONE` and `RX_VALID` are cleared so that firmware always reads fresh completion status after each command.

### 4.4 CMD values

| Value | Mnemonic | Behaviour                                                     |
|------:|----------|---------------------------------------------------------------|
| 1     | `START`  | Decode `TXDATA` as I2C address byte; select endpoint          |
| 2     | `STOP`   | End active transaction; release endpoint; go to `IDLE`        |
| 3     | `WRITE`  | Forward `TXDATA` byte to selected endpoint                    |
| 4     | `READ`   | Pull one byte from endpoint into `RXDATA`; set `RX_VALID`     |

### 4.5 Address byte convention (on START)

```text
 TXDATA:  bit 7 ... bit 1    bit 0
          [ 7-bit address ]  [R/W]
          R/W = 0 → write direction
          R/W = 1 → read direction
```

Example: to read from slave `0x50`, set `TXDATA = 0xA1` before issuing START.

---

## 6. Controller State Machine

```text
                    +--------+
              +---->|  IDLE  |
              |     +---+----+
              |         |
              |         | CMD=START, addr matches
              |         v
              |    +-----------+       (transient: address resolution)
              |    | SEND_ADDR |
              |    +-----------+
              |         |  endpoint found
              |         v
              |    +----------+
              |    | TRANSFER |<------+
              |    +----+-----+       |
              |         |             | CMD=WRITE / CMD=READ
              |         +-------------+
              |         |
              |         | CMD=STOP (only valid here)
              +---------+


   Any command that fails → ERROR state
   (no slave, disabled, STOP from idle, wrong direction, bad opcode)
```

**State descriptions:**

| State       | Meaning                                                    |
|-------------|------------------------------------------------------------|
| `IDLE`      | No transaction in progress; controller ready               |
| `START`     | Transient; entered at top of `command_start()`             |
| `SEND_ADDR` | Transient; address is being decoded (see §5.1)             |
| `TRANSFER`  | Transaction active; WRITE/READ commands accepted           |
| `STOP`      | Transient; entered briefly as endpoint is released         |
| `ERROR`     | Sticky error; clears on next successful command or W1C     |

### 5.1 Why `SEND_ADDR` exists

`SEND_ADDR` is a transient state that is assigned and then immediately
overwritten within a single call to `command_start()`. 
It is **never externally observable** through the public API in the current functional
model because address resolution completes atomically.

It is kept in the enum to:
1. Document the natural hardware pipeline step.
2. Make future cycle-accurate or multi-step expansion straightforward
   (just make `command_start()` return early when in `SEND_ADDR`).

### 5.2 Recovery from `ERROR`

`ERROR` is sticky in STATUS. The intended recovery path is:

1. **Software W1C:** write `I2C_STATUS_ERROR | I2C_STATUS_DONE` to STATUS to
   explicitly clear the error flags.
2. **Issue a fresh `START`:** a well-formed `START` with a valid address is the
   first command that can succeed from any state. On success, `set_done()`
   clears the `ERROR` bit and sets `DONE`, recovering the FSM to `TRANSFER`.

`STOP`, `WRITE`, and `READ` all require an active transaction and will
themselves set `ERROR` if issued while in `ERROR` state, so `START` is the
correct entry point after any error.

---

## 7. `STOP` Enforcement

`STOP` is valid **only while a transaction is active** — i.e., the FSM is in
`TRANSFER` state with a selected endpoint. Issuing `STOP` from `IDLE` or
`ERROR` returns `I2C_ERR_INVALID_SEQUENCE` and sets the `ERROR` STATUS bit.

This is consistent with how `READ` and `WRITE` behave outside an active
transaction, and ensures firmware sequencing bugs (e.g. a bare `STOP` with no
preceding `START`) are surfaced immediately rather than silently ignored.

This is validated by `test_stop_without_active_transaction_sets_error`.

---

## 8. Endpoint Programmer's Model

Each endpoint is a generic "register-style" I2C device:

```c
struct I2CEndpoint {
    uint8_t address;                    // normalized 7-bit address
    uint8_t device_id;                  // mirrored into registers[0x00]
    uint8_t registers[256];             // full register file
    uint8_t register_pointer;           // auto-incrementing, wraps at 256
    bool    active;                     // true while inside a transaction
    bool    expect_register_pointer;    // true until first WRITE sets the pointer
    char    name[32];                   // human-readable label
};
```

### 7.1 Required internal registers

| Offset | Name      | Initial value            |
|-------:|-----------|--------------------------|
| `0x00` | Device ID | Set at `i2c_endpoint_init()` |
| `0x01` | Status    | `0x00`                   |
| `0x02` | Data      | `0x00`                   |

Registers `0x03`–`0xFF` are general-purpose and initialized to `0x00`.

> **Design note:** register `0x00` (Device ID) is writable through the
> normal bus interface. This keeps the endpoint model generic. 
> Firmware can write to it if needed, and tests can pre-load it with known values.

### 7.2 Register-pointer access convention

Modeled after typical sensors and EEPROMs (e.g., MPU-6050, AT24CXX):
For a write transaction, the first byte after address phase sets the endpoint's internal register pointer.
Subsequent write bytes write data starting at that pointer.
For a read transaction, the endpoint returns data from the current register pointer and auto-increments the pointer after each byte.

**Write transaction:**
```text
START + addr(W)          → endpoint.active = true, expect_pointer = true
WRITE <reg_offset>       → register_pointer = reg_offset, expect_pointer = false
WRITE <byte0>            → registers[ptr] = byte0; ptr++
WRITE <byte1>            → registers[ptr] = byte1; ptr++
STOP                     → endpoint.active = false
```

**Combined-format read:**
```text
START + addr(W)          → active = true, expect_pointer = true
WRITE <reg_offset>       → register_pointer = reg_offset
START + addr(R)          → repeated START; active = true, expect_pointer = false
READ                     → value = registers[ptr]; ptr++
READ                     → value = registers[ptr]; ptr++
STOP
```

`register_pointer` is a `uint8_t` and wraps naturally at 256.

### 7.3 Partial-write protection via `i2c_endpoint_end()`

If firmware issues `START(W)` then `STOP` without writing a register pointer,
`i2c_endpoint_end()` clears `expect_register_pointer`. This prevents
stale state from leaking into the next transaction.

### 7.4 Testbench backdoor API

| Function                     | Purpose                                       |
|------------------------------|-----------------------------------------------|
| `i2c_endpoint_peek_register` | Read a register without going through the bus |
| `i2c_endpoint_poke_register` | Write a register without going through the bus|

These are not part of the firmware-visible interface. They exist for
verification — e.g., to pre-load test data or to verify a write side-effect.

---

## 9. Bus Model

```c
struct I2CBus {
    I2CEndpoint *endpoints[16]; // pointers, not values
    size_t       endpoint_count;
};
```

The bus stores **pointers** to endpoints. Ownership and lifetime stay with
the caller. This is idiomatic for embedded C where allocators are avoided.

`i2c_bus_attach()` enforces two invariants before accepting an endpoint:
1. Bus is not at capacity → `I2C_ERR_BUS_FULL`
2. No existing endpoint has the same 7-bit address → `I2C_ERR_DUPLICATE_ADDR`

`i2c_bus_find()` does a **normalized linear scan** — it compares the
caller-supplied address directly against the stored (already-normalized)
addresses, without redundant masking. With a 16-endpoint cap this is
always O(16); no index structure needed.

---

## 10. Typical Transaction Walkthrough

Reading Device ID register (`0x00`) from slave `0x50`:

```c
write(CONTROL, ENABLE);

// Phase 1: set register pointer (write direction)
write(TXDATA, 0x50 << 1);         // 0xA0 — write direction
write(CMD,    START);             // → bus finds 0x50, endpoint.begin(WRITE)

write(TXDATA, 0x00);              // register offset
write(CMD,    WRITE);             // → endpoint.register_pointer = 0x00

// Phase 2: read data (repeated START, read direction)
write(TXDATA, (0x50 << 1) | 1);   // 0xA1 — read direction
write(CMD,    START);             // → endpoint.begin(READ); pointer unchanged

write(CMD,    READ);              // → RXDATA = registers[0x00]; ptr++
read (RXDATA);                    // → Device ID value

write(CMD,    STOP);              // → endpoint.end(); state = IDLE
```

**What happens inside `CMD = START` (first one):**

1. `i2c_controller_mmio_write(0x10, 1)` is called.
2. `execute_command()` clears `DONE` and `RX_VALID`; checks `ENABLE`.
3. `command_start()` transitions state to `I2C_STATE_START`, sets `BUSY`.
4. Decodes `TXDATA`: address = `0x50`, direction = `WRITE`.
5. `i2c_bus_find(bus, 0x50)` returns the matching endpoint.
6. If there was a previously selected endpoint with a different address,
   `i2c_endpoint_end()` is called on it (repeated START with address change).
7. `selected_endpoint` is set; state moves to `SEND_ADDR`.
8. `i2c_endpoint_begin(endpoint, WRITE)` arms the endpoint:
   `active = true`, `expect_register_pointer = true`.
9. State moves to `TRANSFER`; `set_done()` clears `ERROR`, sets `DONE`.

---

## 11. Error Handling

Every public function returns `I2C_OK` (0) or a negative `I2CResult` code.
Any failure inside `execute_command()` also sets the `ERROR` STATUS bit and
transitions the FSM to `I2C_STATE_ERROR`.

| Code                        | Cause                                          |
|-----------------------------|------------------------------------------------|
| `I2C_ERR_INVALID_ARG`       | NULL pointer passed to any public function     |
| `I2C_ERR_NO_SLAVE`          | No endpoint matched address on `START`         |
| `I2C_ERR_INVALID_SEQUENCE`  | WRITE/READ/STOP outside a valid transaction    |
| `I2C_ERR_DISABLED`          | Any `CMD` write while `ENABLE = 0`             |
| `I2C_ERR_DUPLICATE_ADDR`    | Two endpoints with same 7-bit address          |
| `I2C_ERR_BUS_FULL`          | More than `I2C_MAX_ENDPOINTS` (8) attached    |
| `I2C_ERR_INVALID_OFFSET`    | MMIO access to an unmapped register address    |
| `I2C_ERR_INVALID_COMMAND`   | Unknown opcode written to CMD                  |

Use `i2c_result_to_string(rc)` for human-readable logging (used by `demo.c`).

> **Known limitation:** 
> missing slave (`I2C_ERR_NO_SLAVE`) and invalid sequence (`I2C_ERR_INVALID_SEQUENCE`)
> both raise the same generic `ERROR` STATUS bit.
> The return code distinguishes them at the call site.
> A future MMIO-visible error-cause register could expose the distinction to
> firmware without needing the C return value.

---

## 12. How to Build, Run, and Test

```bash
make            # builds tests + demo into ./build/
make test       # runs the assertion-based test suite (9 tests)
make demo       # runs the annotated demo program
make clean      # removes build/
```

Build flags: `-std=c11 -Wall -Wextra -Werror -pedantic`. No external
dependencies — standard C library only.

### Test suite coverage

| Test function                                  | What it validates                         |
|------------------------------------------------|-------------------------------------------|
| `test_device_id_read`                          | Combined-format register read             |
| `test_register_write_and_readback`             | Pointer auto-increment and persistence    |
| `test_multiple_endpoints`                      | Bus address routing                       |
| `test_missing_slave_sets_error`                | START to unmapped address fails           |
| `test_invalid_read_sequence_sets_error`        | READ without active read transaction      |
| `test_stop_without_active_transaction_sets_error` | STOP from IDLE is an error *(new)*     |
| `test_controller_disabled_sets_error`          | ENABLE gating                             |
| `test_duplicate_address_rejected`              | Bus uniqueness invariant                  |
| `test_status_write_one_to_clear`               | W1C semantics for sticky STATUS bits      |

Expected output:
```
PASS: device ID read returned 0xa5
PASS: register write/readback returned 0x5a
PASS: multiple endpoints routed correctly
PASS: missing slave address produced ERROR status
PASS: READ without active transaction produced ERROR status
PASS: STOP without active transaction produced ERROR status
PASS: command while disabled produced ERROR status
PASS: duplicate slave address rejected
PASS: STATUS write-one-to-clear behavior works
All I2C C model tests passed.
```


---

## 13. Coding Conventions

- **C11**, no compiler-specific extensions.
- All public types prefixed `I2C` / `i2c_`.
- Every public function takes the owning struct as its first argument.
- NULL pointer arguments are checked; functions degrade gracefully.
- `uint8_t` / `uint32_t` everywhere — no bare `int` for hardware values.
- Bit constants use `1u << N` to avoid signed-shift UB under `-pedantic`.
- No global state. Every instance is explicitly initialized.

---

## 14. Glossary

| Term               | Meaning in this project                                      |
|--------------------|--------------------------------------------------------------|
| Controller         | I2C master                                                   |
| Endpoint           | I2C slave / target                                           |
| Bus                | Container that routes the controller to one endpoint         |
| MMIO               | Memory-mapped I/O — registers accessed by byte offset        |
| W1C                | Write-One-to-Clear: writing 1 clears the bit; 0 is no-op     |
| Sticky bit         | Latches when set; stays until explicitly cleared             |
| Register pointer   | Endpoint-internal index that auto-increments on access       |
| Repeated START     | A second START issued without an intervening STOP            |
| Transaction-level  | Models programmer-visible behavior, not electrical waveforms |
