#ifndef I2C_MODEL_H
#define I2C_MODEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Simplified transaction-level I2C subsystem model.
 *
 * The model is intentionally functional, not electrical/cycle accurate.
 * It models the programmer-visible behavior expected by firmware:
 * MMIO registers, commands, status bits, 7-bit addressing, simple endpoint
 * register access, and bus routing to matching slave devices.
 */

/** Maximum number of registers in a single endpoint's register file. */
#define I2C_ENDPOINT_REGISTER_COUNT 256u
/** Maximum number of endpoints (slaves) that can be attached to one bus. */
#define I2C_MAX_ENDPOINTS          8u
/** Maximum length of an endpoint name string, including the null terminator. */
#define I2C_ENDPOINT_NAME_LEN      32u

/**
 * MMIO register offsets for the I2C controller.
 * All registers are 32-bit words. Pass these to i2c_controller_mmio_read/write().
 */
typedef enum I2CRegisterOffset {
    I2C_REG_CONTROL = 0x00u, // R/W   — Enable/configuration register.
    I2C_REG_STATUS  = 0x04u, // R/W1C — Status flags (BUSY, DONE, ERROR, RX_VALID).
    I2C_REG_TXDATA  = 0x08u, // R/W   — Byte to transmit (address or data, low 8 bits).
    I2C_REG_RXDATA  = 0x0Cu, // R     — Last byte received from an endpoint.
    I2C_REG_CMD     = 0x10u  // W     — Command trigger; reads return 0.
} I2CRegisterOffset;

/**
 * Bit masks for the CONTROL register.
 */
typedef enum I2CControlBits {
    I2C_CTRL_ENABLE = 1u << 0  // Set to enable the controller; clear to disable.
} I2CControlBits;

/**
 * Bit masks for the STATUS register.
 *
 * BUSY is a live (non-sticky) bit driven by hardware.
 * DONE, ERROR, and RX_VALID are sticky: they latch when set and remain until
 * the next command clears them or software writes 1 to the bit (W1C).
 */
typedef enum I2CStatusBits {
    I2C_STATUS_BUSY     = 1u << 0, // Transaction is in progress (live, not W1C-able).
    I2C_STATUS_DONE     = 1u << 1, // Last command completed successfully (sticky).
    I2C_STATUS_ERROR    = 1u << 2, // An error occurred on the last command (sticky).
    I2C_STATUS_RX_VALID = 1u << 3  // RXDATA contains a valid received byte (sticky).
} I2CStatusBits;

/**
 * Commands written to the CMD register to drive the I2C transaction.
 */
typedef enum I2CCommand {
    I2C_CMD_START = 1u, // Decode TXDATA as address byte and select matching endpoint.
    I2C_CMD_STOP  = 2u, // End transaction; release endpoint; return FSM to IDLE.
    I2C_CMD_WRITE = 3u, // Forward TXDATA byte to the selected endpoint.
    I2C_CMD_READ  = 4u  // Read one byte from endpoint into RXDATA; set RX_VALID.
} I2CCommand;

/**
 * Transaction direction, decoded from TXDATA[0] on START.
 */
typedef enum I2CDirection {
    I2C_DIR_WRITE = 0, // Write transaction: firmware sends data to endpoint.
    I2C_DIR_READ  = 1  // Read transaction: firmware receives data from endpoint.
} I2CDirection;

/**
 * Internal FSM states of the I2C controller.
 * The externally observable stable states are IDLE, TRANSFER, and ERROR.
 * START, SEND_ADDR, and STOP are transient and pass through within a single
 * MMIO write in the current transaction-level model.
 */
typedef enum I2CControllerState {
    I2C_STATE_IDLE      = 0, // No transaction active; ready for a START command.
    I2C_STATE_START,         // Transient: entered at the top of command_start().
            // Transient: address decoding in progress. Not externally observable in the
            // current functional model. Kept to document the hardware pipeline step and
            // ease future cycle-accurate expansion.
    I2C_STATE_SEND_ADDR,
    I2C_STATE_TRANSFER,      // Transaction active; WRITE and READ accepted.
    I2C_STATE_STOP,          // Transient: endpoint being released after STOP.
    I2C_STATE_ERROR          // Sticky error; recover via W1C STATUS then a new START.
} I2CControllerState;

/**
 * Return codes used by all i2c_* functions.
 * Zero (I2C_OK) means success; all error codes are negative.
 */
typedef enum I2CResult {
    I2C_OK                   =  0, // Operation completed successfully.
    I2C_ERR_INVALID_ARG      = -1, // A required pointer argument is NULL.
    I2C_ERR_NO_SLAVE         = -2, // No endpoint matched the address given on START.
    I2C_ERR_INVALID_SEQUENCE = -3, // Command issued out of order (e.g. WRITE without START).
    I2C_ERR_DISABLED         = -4, // CMD written while CONTROL.ENABLE = 0.
    I2C_ERR_DUPLICATE_ADDR   = -5, // Endpoint address already present on the bus.
    I2C_ERR_BUS_FULL         = -6, // I2C_MAX_ENDPOINTS endpoints already attached.
    I2C_ERR_INVALID_OFFSET   = -7, // MMIO offset does not map to any register.
    I2C_ERR_INVALID_COMMAND  = -8  // Unknown value written to the CMD register.
} I2CResult;

/**
 * Model of a single I2C slave (endpoint) device.
 *
 * Each endpoint has a fixed 7-bit address, a 256-byte internal register file,
 * and a register pointer that auto-increments on every read or data write.
 * The first WRITE byte after START(W) sets the register pointer rather than
 * writing data — the standard register-addressed I2C device convention.
 */
typedef struct I2CEndpoint {
    uint8_t address;                             // Normalized 7-bit slave address (bits [6:0]).
    uint8_t device_id;                           // Mirrored into registers[0x00] at init/reset.
    char    name[I2C_ENDPOINT_NAME_LEN];         // Human-readable label for logging/debug.

    uint8_t registers[I2C_ENDPOINT_REGISTER_COUNT]; // Full 256-byte internal register file.
    uint8_t register_pointer;                    // Current read/write index; wraps at 256.
    bool    active;                              // True while inside a transaction (begin→end).
    bool    expect_register_pointer;             // True until the first WRITE sets the pointer.
} I2CEndpoint;

/**
 * I2C bus abstraction connecting the controller to one or more endpoints.
 *
 * Stores pointers to endpoints (not copies). The caller owns each endpoint's
 * lifetime. Up to I2C_MAX_ENDPOINTS may be attached; duplicate addresses are
 * rejected at attach time.
 */
typedef struct I2CBus {
    I2CEndpoint *endpoints[I2C_MAX_ENDPOINTS]; // Pointers to attached endpoints.
    size_t       endpoint_count;               // Number of currently attached endpoints.
} I2CBus;

/**
 * I2C controller (master) peripheral model.
 *
 * Exposed to software through five 32-bit MMIO registers (CONTROL, STATUS,
 * TXDATA, RXDATA, CMD). Internally maintains a transaction FSM and a pointer
 * to the currently selected endpoint.
 */
typedef struct I2CController {
    I2CBus *bus;                    // Bus the controller drives; set at init, not owned.

    uint32_t control;               // Shadow of the CONTROL MMIO register.
    uint32_t status;                // Shadow of the STATUS MMIO register (BUSY/DONE/ERROR/RX).
    uint8_t  txdata;                // Shadow of TXDATA (low 8 bits only).
    uint8_t  rxdata;                // Shadow of RXDATA; meaningful only when RX_VALID is set.

    I2CControllerState state;       // Current FSM state.
    I2CDirection current_direction; // Direction (R/W) of the active transaction.
    I2CEndpoint *selected_endpoint; // Endpoint chosen by the last START; NULL when idle.
} I2CController;

/***********************/
/* Endpoint/slave API. */
/***********************/

/**
 * Initialise an endpoint to a known state.
 *
 * @param endpoint           Pointer to the endpoint to initialise.
 * @param seven_bit_address  7-bit I2C slave address (bits [6:0] used; bit 7 ignored).
 * @param device_id          Value placed in internal register 0x00 (Device ID).
 * @param name               Human-readable label (NULL → auto-generated from address).
 */
void i2c_endpoint_init(I2CEndpoint *endpoint,
                       uint8_t seven_bit_address,
                       uint8_t device_id,
                       const char *name);

/**
 * Reset an endpoint's register map and state without changing its address or device ID.
 * Clears all registers to 0x00, restores device_id into register 0x00, and
 * resets the register pointer and active/expect_register_pointer flags.
 *
 * @param endpoint  Pointer to the endpoint to reset.
 */
void i2c_endpoint_reset(I2CEndpoint *endpoint);

/**
 * Begin a transaction on the endpoint (called by the controller on START).
 * Sets active = true. In write mode, arms expect_register_pointer so the
 * first WRITE byte is treated as a register offset rather than data.
 *
 * @param endpoint   Pointer to the endpoint.
 * @param direction  I2C_DIR_WRITE or I2C_DIR_READ.
 */
void i2c_endpoint_begin(I2CEndpoint *endpoint, I2CDirection direction);

/**
 * End a transaction on the endpoint (called by the controller on STOP or error).
 * Clears active and expect_register_pointer to prevent state leaking into
 * the next transaction.
 *
 * @param endpoint  Pointer to the endpoint.
 */
void i2c_endpoint_end(I2CEndpoint *endpoint);

/**
 * Write one byte into the endpoint during an active write transaction.
 * If expect_register_pointer is set, the byte sets the register pointer and
 * does not write data. Subsequent bytes write to registers[register_pointer++].
 *
 * @param endpoint  Pointer to the endpoint.
 * @param value     Byte to write.
 * @return          I2C_OK on success.
 *                  I2C_ERR_INVALID_ARG if endpoint is NULL.
 *                  I2C_ERR_INVALID_SEQUENCE if the endpoint is not active.
 */
int i2c_endpoint_write_byte(I2CEndpoint *endpoint, uint8_t value);

/**
 * Read one byte from the endpoint during an active read transaction.
 * Returns registers[register_pointer] and increments register_pointer.
 *
 * @param endpoint  Pointer to the endpoint.
 * @param value     Output: byte read from the current register pointer.
 * @return          I2C_OK on success.
 *                  I2C_ERR_INVALID_ARG if endpoint or value is NULL.
 *                  I2C_ERR_INVALID_SEQUENCE if the endpoint is not active.
 */
int i2c_endpoint_read_byte(I2CEndpoint *endpoint, uint8_t *value);

/**
 * Testbench backdoor: read a register directly without going through the bus.
 * Does not affect register_pointer or active state.
 *
 * @param endpoint  Pointer to the endpoint (const; no side-effects).
 * @param offset    Register offset (0x00–0xFF).
 * @return          Register value, or 0 if endpoint is NULL.
 */
uint8_t i2c_endpoint_peek_register(const I2CEndpoint *endpoint, uint8_t offset);

/**
 * Testbench backdoor: write a register directly without going through the bus.
 * Useful for pre-loading test data or injecting fault conditions.
 *
 * @param endpoint  Pointer to the endpoint.
 * @param offset    Register offset (0x00–0xFF).
 * @param value     Value to write.
 */
void i2c_endpoint_poke_register(I2CEndpoint *endpoint, uint8_t offset, uint8_t value);

/************/
/* Bus API. */
/************/

/**
 * Initialise a bus to an empty state (zero endpoints).
 *
 * @param bus  Pointer to the bus to initialise.
 */
void i2c_bus_init(I2CBus *bus);

/**
 * Attach an endpoint to the bus.
 * The bus stores a pointer to the endpoint; the caller owns the endpoint's lifetime.
 *
 * @param bus       Pointer to the bus.
 * @param endpoint  Pointer to the endpoint to attach.
 * @return          I2C_OK on success.
 *                  I2C_ERR_INVALID_ARG if bus or endpoint is NULL.
 *                  I2C_ERR_BUS_FULL if I2C_MAX_ENDPOINTS are already attached.
 *                  I2C_ERR_DUPLICATE_ADDR if the address is already on the bus.
 */
int i2c_bus_attach(I2CBus *bus, I2CEndpoint *endpoint);

/**
 * Find an endpoint by its 7-bit address.
 *
 * @param bus               Pointer to the bus.
 * @param seven_bit_address 7-bit address to look up.
 * @return                  Pointer to the matching endpoint, or NULL if not found.
 */
I2CEndpoint *i2c_bus_find(I2CBus *bus, uint8_t seven_bit_address);

/**
 * Return the number of endpoints currently attached to the bus.
 *
 * @param bus  Pointer to the bus (const; no side-effects).
 * @return     Number of attached endpoints, or 0 if bus is NULL.
 */
size_t i2c_bus_endpoint_count(const I2CBus *bus);

/**************************/
/* Controller/master API. */
/**************************/

/**
 * Initialise a controller and associate it with a bus.
 * Calls i2c_controller_reset() internally.
 *
 * @param controller  Pointer to the controller to initialise.
 * @param bus         Pointer to the bus the controller will drive.
 */
void i2c_controller_init(I2CController *controller, I2CBus *bus);

/**
 * Reset a controller to its power-on state.
 * Clears all registers, returns FSM to IDLE, and ends any active endpoint
 * transaction. Does not detach endpoints from the bus.
 *
 * @param controller  Pointer to the controller to reset.
 */
void i2c_controller_reset(I2CController *controller);

/**
 * Perform an MMIO read from a controller register.
 *
 * @param controller  Pointer to the controller.
 * @param offset      Register offset (use I2CRegisterOffset values).
 * @param value       Output: register value on success; 0 on error.
 * @return            I2C_OK on success.
 *                    I2C_ERR_INVALID_ARG if controller or value is NULL.
 *                    I2C_ERR_INVALID_OFFSET if offset does not map to a register.
 */
int i2c_controller_mmio_read(I2CController *controller, uint32_t offset, uint32_t *value);

/**
 * Perform an MMIO write to a controller register.
 * Writing to CMD triggers immediate command execution.
 * Writing to STATUS clears sticky bits using write-one-to-clear semantics.
 *
 * @param controller  Pointer to the controller.
 * @param offset      Register offset (use I2CRegisterOffset values).
 * @param value       Value to write.
 * @return            I2C_OK on success.
 *                    I2C_ERR_INVALID_ARG if controller is NULL.
 *                    I2C_ERR_INVALID_OFFSET if offset does not map to a register.
 *                    I2C_ERR_DISABLED if CMD is written while CONTROL.ENABLE = 0.
 *                    I2C_ERR_NO_SLAVE if START address has no matching endpoint.
 *                    I2C_ERR_INVALID_SEQUENCE if the command is out of order.
 *                    I2C_ERR_INVALID_COMMAND if an unknown CMD value is written.
 */
int i2c_controller_mmio_write(I2CController *controller, uint32_t offset, uint32_t value);

/**
 * Return the current FSM state of the controller.
 *
 * @param controller  Pointer to the controller (const; no side-effects).
 * @return            Current I2CControllerState, or I2C_STATE_ERROR if controller is NULL.
 */
I2CControllerState i2c_controller_state(const I2CController *controller);

/* Utility helpers for tests/demo/logging. */

/**
 * Convert an I2CControllerState to a human-readable string.
 *
 * @param state  FSM state value.
 * @return       Null-terminated string (e.g. "IDLE", "TRANSFER", "ERROR").
 */
const char *i2c_state_to_string(I2CControllerState state);

/**
 * Convert a CMD register value to a human-readable string.
 *
 * @param command  Command value (use I2CCommand enum values).
 * @return         Null-terminated string (e.g. "START", "WRITE"), or "UNKNOWN".
 */
const char *i2c_command_to_string(uint32_t command);

/**
 * Convert an I2CResult return code to a human-readable string.
 *
 * @param result  Return code from any i2c_* function.
 * @return        Null-terminated string (e.g. "I2C_OK", "I2C_ERR_NO_SLAVE").
 */
const char *i2c_result_to_string(int result);

#ifdef __cplusplus
}
#endif

#endif /* I2C_MODEL_H */
