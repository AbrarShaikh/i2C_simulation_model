#include "i2c_model.h"

#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 * Internal helpers
 *---------------------------------------------------------------------------*/

static bool controller_enabled(const I2CController *controller)
{
    return (controller->control & I2C_CTRL_ENABLE) != 0u;
}

static bool transaction_active(const I2CController *controller)
{
    return controller->selected_endpoint != NULL &&
           controller->state != I2C_STATE_IDLE &&
           controller->state != I2C_STATE_ERROR;
}

static void set_busy(I2CController *controller, bool value)
{
    if (value) {
        controller->status |= I2C_STATUS_BUSY;
    } else {
        controller->status &= ~I2C_STATUS_BUSY;
    }
}

// Clear DONE and RX_VALID before each command; ERROR persists until W1C or set_done().
static void clear_transient_status(I2CController *controller)
{
    controller->status &= ~(I2C_STATUS_DONE | I2C_STATUS_RX_VALID);
}

// Marks a command as successful. Clears ERROR so a fresh START after W1C
// recovery lands in a clean state. DONE is set so firmware can poll completion.
static void set_done(I2CController *controller)
{
    controller->status &= ~I2C_STATUS_ERROR;
    controller->status |= I2C_STATUS_DONE;
}

// Aborts any active transaction and enters ERROR state.
// DONE is set alongside ERROR so firmware polling for DONE also sees the error.
static void set_error(I2CController *controller)
{
    if (controller->selected_endpoint != NULL) {
        i2c_endpoint_end(controller->selected_endpoint);
        controller->selected_endpoint = NULL;
    }

    set_busy(controller, false);
    controller->state = I2C_STATE_ERROR;
    controller->status |= I2C_STATUS_ERROR | I2C_STATUS_DONE;
}

/*----------------------------------------------------------------------------
 * Command handlers — one per CMD register value
 *---------------------------------------------------------------------------*/

static int command_start(I2CController *controller)
{
    uint8_t address;
    I2CDirection direction;
    I2CEndpoint *endpoint;

    controller->state = I2C_STATE_START;
    set_busy(controller, true);

    // TXDATA[7:1] = 7-bit address, TXDATA[0] = direction (0=W, 1=R)
    address   = (uint8_t)((controller->txdata >> 1u) & 0x7Fu);
    direction = (controller->txdata & 0x1u) ? I2C_DIR_READ : I2C_DIR_WRITE;

    endpoint = i2c_bus_find(controller->bus, address);
    if (endpoint == NULL) {
        set_error(controller);
        return I2C_ERR_NO_SLAVE;
    }

    // Repeated START: end previous endpoint if address changed.
    if (controller->selected_endpoint != NULL &&
        controller->selected_endpoint != endpoint) {
        i2c_endpoint_end(controller->selected_endpoint);
    }

    controller->selected_endpoint = endpoint;
    controller->current_direction = direction;

    controller->state = I2C_STATE_SEND_ADDR; // transient; resolves atomically
    i2c_endpoint_begin(endpoint, direction);

    controller->state = I2C_STATE_TRANSFER;
    set_done(controller);
    return I2C_OK;
}

static int command_stop(I2CController *controller)
{
    // STOP without an active transaction is a firmware sequencing error.
    if (!transaction_active(controller)) {
        set_error(controller);
        return I2C_ERR_INVALID_SEQUENCE;
    }

    controller->state = I2C_STATE_STOP;

    if (controller->selected_endpoint != NULL) {
        i2c_endpoint_end(controller->selected_endpoint); // clears expect_register_pointer
        controller->selected_endpoint = NULL;
    }

    set_busy(controller, false);
    controller->state = I2C_STATE_IDLE;
    set_done(controller);
    return I2C_OK;
}

static int command_write(I2CController *controller)
{
    int rc;

    // Reject WRITE if no active transaction or if direction is READ.
    // Mixing directions without a repeated START is a firmware bug.
    if (!transaction_active(controller) ||
        controller->current_direction != I2C_DIR_WRITE) {
        set_error(controller);
        return I2C_ERR_INVALID_SEQUENCE;
    }

    controller->state = I2C_STATE_TRANSFER;
    rc = i2c_endpoint_write_byte(controller->selected_endpoint, controller->txdata);
    if (rc != I2C_OK) {
        set_error(controller);
        return rc;
    }

    set_done(controller);
    return I2C_OK;
}

static int command_read(I2CController *controller)
{
    int rc;
    uint8_t value = 0u;

    // Reject READ if no active transaction or if direction is WRITE.
    // Firmware must issue START(R) before issuing READ commands.
    if (!transaction_active(controller) ||
        controller->current_direction != I2C_DIR_READ) {
        set_error(controller);
        return I2C_ERR_INVALID_SEQUENCE;
    }

    controller->state = I2C_STATE_TRANSFER;
    rc = i2c_endpoint_read_byte(controller->selected_endpoint, &value);
    if (rc != I2C_OK) {
        set_error(controller);
        return rc;
    }

    controller->rxdata = value;
    controller->status |= I2C_STATUS_RX_VALID; // signals firmware that RXDATA holds fresh data
    set_done(controller);
    return I2C_OK;
}

// Dispatch a CMD register write. Clears transient status first, then checks ENABLE.
static int execute_command(I2CController *controller, uint32_t command)
{
    clear_transient_status(controller);

    if (!controller_enabled(controller)) {
        set_error(controller);
        return I2C_ERR_DISABLED;
    }

    switch (command) {
    case I2C_CMD_START:
        return command_start(controller);
    case I2C_CMD_STOP:
        return command_stop(controller);
    case I2C_CMD_WRITE:
        return command_write(controller);
    case I2C_CMD_READ:
        return command_read(controller);
    default:
        set_error(controller);
        return I2C_ERR_INVALID_COMMAND;
    }
}

/*----------------------------------------------------------------------------
 * Endpoint (slave) implementation
 *---------------------------------------------------------------------------*/
 
 void i2c_endpoint_init(I2CEndpoint *endpoint, uint8_t seven_bit_address,
                       uint8_t device_id, const char *name)
{
    size_t len;

    if (endpoint == NULL) {
        return;
    }

    memset(endpoint, 0, sizeof(*endpoint));
    endpoint->address   = (uint8_t)(seven_bit_address & 0x7Fu);
    endpoint->device_id = device_id;

    if (name != NULL) {
        len = strlen(name);
        if (len >= I2C_ENDPOINT_NAME_LEN) {
            len = I2C_ENDPOINT_NAME_LEN - 1u;
        }
        memcpy(endpoint->name, name, len);
        endpoint->name[len] = '\0';
    } else {
        (void)snprintf(endpoint->name, sizeof(endpoint->name),
                       "i2c-endpoint-0x%02x", endpoint->address);
    }

    i2c_endpoint_reset(endpoint);
}

void i2c_endpoint_reset(I2CEndpoint *endpoint)
{
    if (endpoint == NULL) {
        return;
    }

    memset(endpoint->registers, 0, sizeof(endpoint->registers));

    // Required register map (assignment spec).
    endpoint->registers[0x00u] = endpoint->device_id; // Device ID
    endpoint->registers[0x01u] = 0x00u;               // Status
    endpoint->registers[0x02u] = 0x00u;               // Data

    endpoint->register_pointer        = 0u;
    endpoint->active                  = false;
    endpoint->expect_register_pointer = false;
}

void i2c_endpoint_begin(I2CEndpoint *endpoint, I2CDirection direction)
{
    if (endpoint == NULL) {
        return;
    }

    endpoint->active = true;

    // First byte of a write transaction sets the register pointer, not data.
    endpoint->expect_register_pointer = (direction == I2C_DIR_WRITE);
}

void i2c_endpoint_end(I2CEndpoint *endpoint)
{
    if (endpoint == NULL) {
        return;
    }

    endpoint->active                  = false;
    endpoint->expect_register_pointer = false;
}

int i2c_endpoint_write_byte(I2CEndpoint *endpoint, uint8_t value)
{
    if (endpoint == NULL) {
        return I2C_ERR_INVALID_ARG;
    }

    if (!endpoint->active) {
        return I2C_ERR_INVALID_SEQUENCE;
    }

    if (endpoint->expect_register_pointer) {
        endpoint->register_pointer        = value;
        endpoint->expect_register_pointer = false;
        return I2C_OK;
    }

    // Subsequent bytes write data; pointer auto-increments (wraps at 256).
    endpoint->registers[endpoint->register_pointer] = value;
    endpoint->register_pointer = (uint8_t)(endpoint->register_pointer + 1u);
    return I2C_OK;
}

int i2c_endpoint_read_byte(I2CEndpoint *endpoint, uint8_t *value)
{
    if (endpoint == NULL || value == NULL) {
        return I2C_ERR_INVALID_ARG;
    }

    if (!endpoint->active) {
        return I2C_ERR_INVALID_SEQUENCE;
    }

    // Pointer auto-increments after each read (wraps at 256).
    *value = endpoint->registers[endpoint->register_pointer];
    endpoint->register_pointer = (uint8_t)(endpoint->register_pointer + 1u);
    return I2C_OK;
}

// peek/poke are testbench backdoors for direct register inspection without
// going through the I2C transaction protocol.
uint8_t i2c_endpoint_peek_register(const I2CEndpoint *endpoint, uint8_t offset)
{
    if (endpoint == NULL) {
        return 0u;
    }

    return endpoint->registers[offset];
}

void i2c_endpoint_poke_register(I2CEndpoint *endpoint, uint8_t offset, uint8_t value)
{
    if (endpoint == NULL) {
        return;
    }

    endpoint->registers[offset] = value;
}

/*----------------------------------------------------------------------------
 * Bus implementation
 *---------------------------------------------------------------------------*/

void i2c_bus_init(I2CBus *bus)
{
    if (bus == NULL) {
        return;
    }

    memset(bus, 0, sizeof(*bus));
}

int i2c_bus_attach(I2CBus *bus, I2CEndpoint *endpoint)
{
    if (bus == NULL || endpoint == NULL) {
        return I2C_ERR_INVALID_ARG;
    }

    if (bus->endpoint_count >= I2C_MAX_ENDPOINTS) {
        return I2C_ERR_BUS_FULL;
    }

    // Reject duplicate addresses; two slaves at the same address makes routing ambiguous.
    if (i2c_bus_find(bus, endpoint->address) != NULL) {
        return I2C_ERR_DUPLICATE_ADDR;
    }

    bus->endpoints[bus->endpoint_count] = endpoint;
    bus->endpoint_count++;
    return I2C_OK;
}

I2CEndpoint *i2c_bus_find(I2CBus *bus, uint8_t seven_bit_address)
{
    size_t i;

    if (bus == NULL) {
        return NULL;
    }

    // Linear scan is acceptable; I2C_MAX_ENDPOINTS is small (8).
    for (i = 0u; i < bus->endpoint_count; ++i) {
        if (bus->endpoints[i] != NULL &&
            bus->endpoints[i]->address == seven_bit_address) {
            return bus->endpoints[i];
        }
    }

    return NULL;
}

size_t i2c_bus_endpoint_count(const I2CBus *bus)
{
    if (bus == NULL) {
        return 0u;
    }

    return bus->endpoint_count;
}

/*----------------------------------------------------------------------------
 * Controller (master) implementation
 *---------------------------------------------------------------------------*/

void i2c_controller_init(I2CController *controller, I2CBus *bus)
{
    if (controller == NULL) {
        return;
    }

    memset(controller, 0, sizeof(*controller));
    controller->bus = bus;
    i2c_controller_reset(controller);
}

void i2c_controller_reset(I2CController *controller)
{
    if (controller == NULL) {
        return;
    }

    // End any active transaction before wiping state.
    if (controller->selected_endpoint != NULL) {
        i2c_endpoint_end(controller->selected_endpoint);
    }

    controller->control           = 0u;
    controller->status            = 0u;
    controller->txdata            = 0u;
    controller->rxdata            = 0u;
    controller->state             = I2C_STATE_IDLE;
    controller->current_direction = I2C_DIR_WRITE;
    controller->selected_endpoint = NULL;
}

int i2c_controller_mmio_read(I2CController *controller, uint32_t offset, uint32_t *value)
{
    if (controller == NULL || value == NULL) {
        return I2C_ERR_INVALID_ARG;
    }

    switch (offset) {
    case I2C_REG_CONTROL:
        *value = controller->control;
        return I2C_OK;
    case I2C_REG_STATUS:
        *value = controller->status;
        return I2C_OK;
    case I2C_REG_TXDATA:
        *value = controller->txdata;
        return I2C_OK;
    case I2C_REG_RXDATA:
        *value = controller->rxdata;
        return I2C_OK;
    case I2C_REG_CMD:
        *value = 0u; // write-only; reads return 0
        return I2C_OK;
    default:
        *value = 0u;
        return I2C_ERR_INVALID_OFFSET;
    }
}

int i2c_controller_mmio_write(I2CController *controller, uint32_t offset, uint32_t value)
{
    if (controller == NULL) {
        return I2C_ERR_INVALID_ARG;
    }

    switch (offset) {
    case I2C_REG_CONTROL:
        controller->control = value;
        // Disabling mid-transaction aborts it cleanly.
        if (!controller_enabled(controller) && controller->selected_endpoint != NULL) {
            i2c_endpoint_end(controller->selected_endpoint);
            controller->selected_endpoint = NULL;
            set_busy(controller, false);
            controller->state = I2C_STATE_IDLE;
        }
        return I2C_OK;

    case I2C_REG_STATUS:
        // Write-one-to-clear (W1C): a 1 bit in the written value clears the
        // corresponding sticky bit. BUSY is excluded — hardware controls it.
        controller->status &= ~(value &
            (I2C_STATUS_DONE | I2C_STATUS_ERROR | I2C_STATUS_RX_VALID));
        return I2C_OK;

    case I2C_REG_TXDATA:
        controller->txdata = (uint8_t)(value & 0xFFu); // only low 8 bits used
        return I2C_OK;

    case I2C_REG_RXDATA:
        // RXDATA is read-only; writes are silently ignored.
        return I2C_OK;

    case I2C_REG_CMD:
        return execute_command(controller, value);

    default:
        set_error(controller);
        return I2C_ERR_INVALID_OFFSET;
    }
}

I2CControllerState i2c_controller_state(const I2CController *controller)
{
    if (controller == NULL) {
        return I2C_STATE_ERROR;
    }

    return controller->state;
}

/*----------------------------------------------------------------------------
 * Utility / logging helpers
 *---------------------------------------------------------------------------*/

const char *i2c_state_to_string(I2CControllerState state)
{
    switch (state) {
    case I2C_STATE_IDLE:      return "IDLE";
    case I2C_STATE_START:     return "START";
    case I2C_STATE_SEND_ADDR: return "SEND_ADDR";
    case I2C_STATE_TRANSFER:  return "TRANSFER";
    case I2C_STATE_STOP:      return "STOP";
    case I2C_STATE_ERROR:     return "ERROR";
    default:                  return "UNKNOWN";
    }
}

const char *i2c_command_to_string(uint32_t command)
{
    switch (command) {
    case I2C_CMD_START: return "START";
    case I2C_CMD_STOP:  return "STOP";
    case I2C_CMD_WRITE: return "WRITE";
    case I2C_CMD_READ:  return "READ";
    default:            return "UNKNOWN";
    }
}

const char *i2c_result_to_string(int result)
{
    switch (result) {
    case I2C_OK:                   return "I2C_OK";
    case I2C_ERR_INVALID_ARG:      return "I2C_ERR_INVALID_ARG";
    case I2C_ERR_NO_SLAVE:         return "I2C_ERR_NO_SLAVE";
    case I2C_ERR_INVALID_SEQUENCE: return "I2C_ERR_INVALID_SEQUENCE";
    case I2C_ERR_DISABLED:         return "I2C_ERR_DISABLED";
    case I2C_ERR_DUPLICATE_ADDR:   return "I2C_ERR_DUPLICATE_ADDR";
    case I2C_ERR_BUS_FULL:         return "I2C_ERR_BUS_FULL";
    case I2C_ERR_INVALID_OFFSET:   return "I2C_ERR_INVALID_OFFSET";
    case I2C_ERR_INVALID_COMMAND:  return "I2C_ERR_INVALID_COMMAND";
    default:                       return "I2C_ERR_UNKNOWN";
    }
}
