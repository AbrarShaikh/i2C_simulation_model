#include "i2c_model.h"

#include <stdint.h>
#include <stdio.h>

// Wrapper: write an MMIO register and log any error.
static int mmio_write_or_log(I2CController *controller, uint32_t offset, uint32_t value)
{
    int rc = i2c_controller_mmio_write(controller, offset, value);
    if (rc != I2C_OK) {
        printf("MMIO write offset 0x%02x value 0x%02x failed: %s\n",
               offset, value, i2c_result_to_string(rc));
    }
    return rc;
}

// Wrapper: read an MMIO register, return 0 and log on error.
static uint32_t mmio_read_or_zero(I2CController *controller, uint32_t offset)
{
    uint32_t value = 0u;
    int rc = i2c_controller_mmio_read(controller, offset, &value);
    if (rc != I2C_OK) {
        printf("MMIO read offset 0x%02x failed: %s\n",
               offset, i2c_result_to_string(rc));
    }
    return value;
}

static void print_status(I2CController *controller, const char *prefix)
{
    uint32_t status = mmio_read_or_zero(controller, I2C_REG_STATUS);

    printf("%s STATUS=0x%08x [busy=%u done=%u error=%u rx_valid=%u] state=%s\n",
           prefix,
           status,
           (status & I2C_STATUS_BUSY) ? 1u : 0u,
           (status & I2C_STATUS_DONE) ? 1u : 0u,
           (status & I2C_STATUS_ERROR) ? 1u : 0u,
           (status & I2C_STATUS_RX_VALID) ? 1u : 0u,
           i2c_state_to_string(i2c_controller_state(controller)));
}

// Write a byte to a slave register: START(W) -> reg pointer -> data -> STOP.
static void i2c_write_register(I2CController *controller,
                               uint8_t slave_addr,
                               uint8_t reg,
                               uint8_t value)
{
    // Phase 1: address byte — slave_addr shifted left, bit 0 = 0 (write).
    (void)mmio_write_or_log(controller, I2C_REG_TXDATA, (uint8_t)(slave_addr << 1u));
    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_START);

    // Phase 2: first WRITE byte sets the internal register pointer.
    (void)mmio_write_or_log(controller, I2C_REG_TXDATA, reg);
    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_WRITE);

    // Phase 3: second WRITE byte stores the data value.
    (void)mmio_write_or_log(controller, I2C_REG_TXDATA, value);
    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_WRITE);

    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_STOP);
}

// Read a byte from a slave register:
// START(W) -> reg pointer -> repeated START(R) -> READ -> STOP.
static uint8_t i2c_read_register(I2CController *controller,
                                 uint8_t slave_addr,
                                 uint8_t reg)
{
    uint32_t value;

    // Phase 1: write transaction to set the register pointer.
    (void)mmio_write_or_log(controller, I2C_REG_TXDATA, (uint8_t)(slave_addr << 1u));
    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_START);

    (void)mmio_write_or_log(controller, I2C_REG_TXDATA, reg);
    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_WRITE);

    // Phase 2: repeated START with R/W=1 switches to read direction.
    (void)mmio_write_or_log(controller, I2C_REG_TXDATA,
                            (uint8_t)((slave_addr << 1u) | 1u));
    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_START);

    // Phase 3: READ clocks in the byte; result is in RXDATA.
    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_READ);
    value = mmio_read_or_zero(controller, I2C_REG_RXDATA);

    (void)mmio_write_or_log(controller, I2C_REG_CMD, I2C_CMD_STOP);
    return (uint8_t)(value & 0xFFu);
}

int main(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CEndpoint rtc;
    I2CController controller;
    uint8_t id;
    uint8_t data;
    int rc;

    printf("Simplified C I2C subsystem model demo\n");
    printf("-------------------------------------\n");

    // Set up bus with two endpoints: sensor at 0x50, RTC at 0x68.
    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "sensor");
    i2c_endpoint_init(&rtc, 0x68u, 0x68u, "rtc");

    (void)i2c_bus_attach(&bus, &sensor);
    (void)i2c_bus_attach(&bus, &rtc);

    i2c_controller_init(&controller, &bus);
    (void)mmio_write_or_log(&controller, I2C_REG_CONTROL, I2C_CTRL_ENABLE);

    printf("Attached endpoints: %u\n", (unsigned)i2c_bus_endpoint_count(&bus));

    // --- Demo 1: read Device ID from sensor (address 0x50, register 0x00) ---
    id = i2c_read_register(&controller, 0x50u, 0x00u);
    printf("Read sensor Device ID register 0x00: 0x%02x\n", id);
    print_status(&controller, "After ID read:");

    // --- Demo 2: write then read back sensor Data register (0x02) ---
    i2c_write_register(&controller, 0x50u, 0x02u, 0x5Au);
    data = i2c_read_register(&controller, 0x50u, 0x02u);
    printf("Wrote sensor Data register 0x02 = 0x5a, read back: 0x%02x\n", data);

    // --- Demo 3: read Device ID from RTC (address 0x68, register 0x00) ---
    id = i2c_read_register(&controller, 0x68u, 0x00u);
    printf("Read RTC Device ID register 0x00: 0x%02x\n", id);

    // --- Demo 4: error path — START to an address with no attached endpoint ---
    printf("Trying to access missing slave address 0x33...\n");
    (void)mmio_write_or_log(&controller, I2C_REG_TXDATA, (uint8_t)(0x33u << 1u));
    rc = i2c_controller_mmio_write(&controller, I2C_REG_CMD, I2C_CMD_START);
    printf("START returned: %s\n", i2c_result_to_string(rc));
    print_status(&controller, "After missing slave:");

    return 0;
}

