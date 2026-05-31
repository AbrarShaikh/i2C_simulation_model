# Simplified I2C Controller and Endpoint Device Model in C

---

## Introduction — How I2C Works

**I2C (Inter-Integrated Circuit)** is a two-wire serial communication protocol
widely used to connect microcontrollers to sensors, EEPROMs, RTCs, displays,
and other low-speed peripherals on the same board.

### Physical bus

Only two wires are needed:

| Signal | Purpose                                      |
| ------ | -------------------------------------------- |
| SDA    | Serial Data — carries address and data bits  |
| SCL    | Serial Clock — driven by the master          |

Both lines are **open-drain** and pulled high by resistors. Any device can pull
a line low; none can actively drive it high.

```
VCC
 │
 ├── R ──────────────────────── SDA──┬──────────┬──────────┐
 │                                   │          │          │
 ├── R ──────────────────────── SCL──┤          │          │
 │                                   │          │          │
                                 ┌───┴───┐  ┌───┴───┐  ┌───┴───┐
                                 │Master │  │Slave 1│  │Slave 2│
                                 │(MCU)  │  │(0x50) │  │(0x68) │
                                 └───────┘  └───────┘  └───────┘
```

### Addressing

Every slave has a unique **7-bit address** (configured by hardware pins or
factory-programmed). The master selects which slave to talk to by sending its
address at the start of each transaction. Up to 112 devices can share one bus.

### Transaction structure

A transaction always follows this sequence:

```
        START   Address + R/W   ACK   Data byte(s)   ACK   STOP
SDA:  ___╲____  [A6..A0 | R/W]  _     [D7..D0]       _    ╱‾‾‾
SCL:       ‾‾  ________________ ‾‾   ______________ ‾‾   ‾‾
```

| Step         | Description                                                  |
| ------------ | ------------------------------------------------------------ |
| **START**    | Master pulls SDA low while SCL is high — signals bus claim   |
| **Address**  | 7-bit slave address + 1 R/W bit sent MSB-first               |
| **ACK**      | Addressed slave pulls SDA low to acknowledge                 |
| **Data**     | One or more bytes transferred; each followed by an ACK       |
| **STOP**     | Master releases SDA while SCL is high — signals bus release  |

A **repeated START** (Sr) lets the master change direction or address without
releasing the bus — commonly used to write a register pointer then read back
data in a single atomic transaction.

### Write transaction example — set a register

```
Master → Slave 0x50, write register 0x02 = 0xAB

  START  [0x50 | W]  ACK  [0x02]  ACK  [0xAB]  ACK  STOP
```

### Read transaction example — read a register

```
Master → Slave 0x50, read register 0x02

  START  [0x50 | W]  ACK  [0x02]  ACK  Sr  [0x50 | R]  ACK  [0xAB]  NACK  STOP
         ← set register pointer →          ← switch to read →   ← data ←
```

### How this model fits

This model operates at **transaction level** — it skips electrical waveforms
and clock cycles, and instead exposes the same firmware-visible interface a
real I2C controller provides:

- Firmware writes to **MMIO registers** (CONTROL, TXDATA, CMD)
- The controller **routes** the command to the matching slave by address
- The slave responds; the result appears in **STATUS** and **RXDATA**

The model is faithful to the software protocol (addressing, R/W direction,
register-pointer convention, error reporting) without simulating SDA/SCL signals.

---

This is implementation of a simplified I2C controller and endpoint device model in C. It provides a transaction-level simulation of I2C communication, focusing on the software-visible behavior of the controller and devices rather than electrical waveform accuracy.
The model includes:
- I2C Controller / Master exposed through memory-mapped registers
- One or more I2C Endpoint Devices / Slaves
- Simple I2C bus abstraction that routes transactions by 7-bit address
- Transaction-level model; no SDA/SCL waveform simulation

The implementation focuses on **system modelling**, **peripheral modelling**, and **register interface design**.

---

## 1. Build and run

```bash
make test
make demo
```

Clean build artifacts:

```bash
make clean
```

The project uses only standard C11 and does not require external libraries.

---

## 2. File layout

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

---

## 3. Design assumptions

- Single I2C master
- Multiple slave endpoints supported
- 7-bit I2C addressing only
- Transaction-level functional model
- No SDA/SCL electrical waveform simulation
- No clock stretching
- No arbitration
- No timing/cycle accuracy
- Repeated START supported
- Endpoint register pointer convention is used for simple register reads/writes
- Endpoint register `0x00` / Device ID is writable in this simplified model. This is intentional to keep the endpoint model generic and small. A production-quality model could add per-register access permissions to make Device ID read-only.
- If firmware performs `START(W)` followed by `STOP` without writing a register pointer, the endpoint's pending register-pointer state is cleared by `i2c_endpoint_end()`. This avoids state leakage into the next transaction.
- Missing slave and invalid sequence both report the generic `ERROR` status bit. The function return code distinguishes `I2C_ERR_NO_SLAVE` from `I2C_ERR_INVALID_SEQUENCE`; a future MMIO-visible error-cause register could expose that distinction to firmware.

---

## 5. Architecture

```text
+------------------+       MMIO read/write       +----------------------+
| Firmware / CPU   | --------------------------> | I2C Controller       |
| Bare-metal code  |                             | CONTROL/STATUS/etc.  |
+------------------+ <-------------------------- +----------+-----------+
        ^                                                |
        |                                                | transaction calls
        |                                                v
        |                                      +----------------------+
        |                                      | I2C Bus              |
        |                                      | address routing      |
        |                                      +----+------------+----+
        |                                           |            |
        |                                           v            v
        |                                  +-------------+  +-------------+
        |                                  | Endpoint    |  | Endpoint    |
        |                                  | addr 0x50   |  | addr 0x68   |
        |                                  | regs 0..255 |  | regs 0..255 |
        |                                  +-------------+  +-------------+
```

---

## 6. Controller MMIO register interface

All registers are 32-bit MMIO words. Only the low 8 bits of `TXDATA` and `RXDATA` are used.

| Offset | Register  | Access | Description               |
|-------:|-----------|--------|---------------------------|
| `0x00` | `CONTROL` | R/W    | Enable/configuration      |
| `0x04` | `STATUS`  | R/W1C  | Busy/done/error reporting |
| `0x08` | `TXDATA`  | R/W    | Address/data to transmit  |
| `0x0C` | `RXDATA`  | R      | Received data             |
| `0x10` | `CMD`     | W      | Command register          |

### CONTROL bits

| Bit | Name     | Meaning                    |
|----:|----------|----------------------------|
|   0 | `ENABLE` | Enables the I2C controller |

### STATUS bits

| Bit | Name       | Meaning                           |
|----:|------------|-----------------------------------|
|   0 | `BUSY`     | Transaction is active             |
|   1 | `DONE`     | Last command completed            |
|   2 | `ERROR`    | Error occurred                    |
|   3 | `RX_VALID` | `RXDATA` contains valid read data |

`DONE`, `ERROR`, and `RX_VALID` can be cleared by writing `1` to the corresponding bit in `STATUS`.

### CMD values

| Value | Command | Description                        |
|------:|---------|-------------------------------------|
|     1 | `START` | Begin transaction / repeated start |
|     2 | `STOP`  | End transaction                    |
|     3 | `WRITE` | Send one byte from `TXDATA`        |
|     4 | `READ`  | Receive one byte into `RXDATA`     |

---

## 7. Addressing convention

During `START`, the controller interprets `TXDATA` as the I2C address byte:

```text
TXDATA[7:1] = 7-bit slave address
TXDATA[0]   = direction bit
              0 = write
              1 = read
```

Examples:

```text
Slave address 0x50, write phase: TXDATA = 0xA0
Slave address 0x50, read phase:  TXDATA = 0xA1
```

---

## 8. Endpoint device model

Each endpoint has:

- A fixed 7-bit address
- A 256-byte internal register map
- A simple register-pointer access convention

Required internal registers:

| Register offset | Description |
|----------------:|-------------|
|          `0x00` | Device ID   |
|          `0x01` | Status      |
|          `0x02` | Data        |

### Endpoint register access convention

For a write transaction, the first byte after address phase sets the endpoint's internal register pointer.
Subsequent write bytes write data starting at that pointer.

For a read transaction, the endpoint returns data from the current register pointer and auto-increments the pointer after each byte.

This convention supports the common I2C register-read sequence:

```text
START + addr(W)
WRITE register_offset
Repeated START + addr(R)
READ data
STOP
```

---

## 9. State machine

```text
+------+    START     +-----------+   address match   +----------+
| IDLE | -----------> | START     | ----------------> | TRANSFER |
+------+              +-----------+                   +-----+----+
   ^                                                       |
   |                                                       | READ/WRITE
   |                                                       v
   |                                                  +----------+
   |                         STOP                     | TRANSFER |
   +--------------------------------------------------+----------+


[Invalid command / missing slave / disabled controller]
        |
        v
+-------------+
| ERROR state |
+-------------+
```

The state values are:

```text
IDLE
START
SEND_ADDR
TRANSFER
STOP
ERROR
```

`SEND_ADDR` is a transient state in this transaction-level model. It is assigned inside the START command while resolving the address, then the model immediately transitions to `TRANSFER` if a matching endpoint is found. It is kept in the enum to document the natural hardware state-machine step and to make future cycle-accurate expansion easier.

`STOP` is valid only while a transaction is active. A STOP from `IDLE` or `ERROR` is treated as an invalid sequence and sets `ERROR`; this avoids hiding firmware sequencing bugs.

---

## 10. Example transaction: read Device ID from slave 0x50

```text
1. TXDATA = (0x50 << 1) | 0
2. CMD    = START

3. TXDATA = 0x00              ; Device ID register offset
4. CMD    = WRITE

5. TXDATA = (0x50 << 1) | 1   ; repeated START, read direction
6. CMD    = START

7. CMD    = READ
8. RXDATA now contains Device ID

9. CMD    = STOP
```

Equivalent C helper:

```c
static uint8_t i2c_read_register(I2CController *controller,
                                 uint8_t slave_addr,
                                 uint8_t reg)
{
    uint32_t value = 0;

    i2c_controller_mmio_write(controller, I2C_REG_TXDATA, slave_addr << 1);
    i2c_controller_mmio_write(controller, I2C_REG_CMD, I2C_CMD_START);

    i2c_controller_mmio_write(controller, I2C_REG_TXDATA, reg);
    i2c_controller_mmio_write(controller, I2C_REG_CMD, I2C_CMD_WRITE);

    i2c_controller_mmio_write(controller, I2C_REG_TXDATA, (slave_addr << 1) | 1);
    i2c_controller_mmio_write(controller, I2C_REG_CMD, I2C_CMD_START);

    i2c_controller_mmio_write(controller, I2C_REG_CMD, I2C_CMD_READ);
    i2c_controller_mmio_read(controller, I2C_REG_RXDATA, &value);

    i2c_controller_mmio_write(controller, I2C_REG_CMD, I2C_CMD_STOP);
    return (uint8_t)value;
}
```

---

## 11. Validation covered by tests

`tests/test_i2c_model.c` validates:

1. Reading Device ID register from slave `0x50`
2. Writing endpoint register `0x02` and reading it back
3. Routing to multiple endpoints
4. Missing slave address sets `ERROR`
5. `READ` without active read transaction sets `ERROR`
6. `STOP` without an active transaction sets `ERROR`
7. Command while controller is disabled sets `ERROR`
8. Duplicate endpoint address is rejected
9. `STATUS` write-one-to-clear behavior

Expected output:

```text
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

## 12. What to explain in a review

A concise design explanation:

> I implemented the controller as a memory-mapped peripheral with CONTROL, STATUS, TXDATA, RXDATA, and CMD registers. Software starts transactions by writing an address byte into TXDATA and writing START to CMD. The bus abstraction decodes the 7-bit address and routes the transaction to the matching endpoint. The endpoint contains a 256-byte register map with Device ID, Status, and Data registers. The model is transaction-level rather than electrical-level, so it focuses on programmer-visible behavior: command sequencing, address matching, state transitions, status bits, error reporting, endpoint register access, and bus routing.

Possible improvements if more time were available:

- Add interrupt register and IRQ output
- Add FIFO support
- Add 10-bit addressing
- Add read-only/write-only access permissions for endpoint registers
- Add NACK cause reporting or an MMIO-visible error-cause register
- Add clock stretching support
- Add timing model for command latency
- Add trace logging hooks for debugging
