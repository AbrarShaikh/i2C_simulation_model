#include "i2c_model.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static void write_mmio(I2CController *controller, uint32_t offset, uint32_t value)
{
    int rc = i2c_controller_mmio_write(controller, offset, value);
    assert(rc == I2C_OK);
}

static uint32_t read_mmio(I2CController *controller, uint32_t offset)
{
    uint32_t value = 0u;
    int rc = i2c_controller_mmio_read(controller, offset, &value);
    assert(rc == I2C_OK);
    return value;
}

static void i2c_write_register(I2CController *controller,
                               uint8_t slave_addr,
                               uint8_t reg,
                               uint8_t value)
{
    write_mmio(controller, I2C_REG_TXDATA, (uint8_t)(slave_addr << 1u));
    write_mmio(controller, I2C_REG_CMD, I2C_CMD_START);

    write_mmio(controller, I2C_REG_TXDATA, reg);
    write_mmio(controller, I2C_REG_CMD, I2C_CMD_WRITE);

    write_mmio(controller, I2C_REG_TXDATA, value);
    write_mmio(controller, I2C_REG_CMD, I2C_CMD_WRITE);

    write_mmio(controller, I2C_REG_CMD, I2C_CMD_STOP);
}

static uint8_t i2c_read_register(I2CController *controller,
                                 uint8_t slave_addr,
                                 uint8_t reg)
{
    uint32_t value;

    /* Write phase: set endpoint internal register pointer. */
    write_mmio(controller, I2C_REG_TXDATA, (uint8_t)(slave_addr << 1u));
    write_mmio(controller, I2C_REG_CMD, I2C_CMD_START);

    write_mmio(controller, I2C_REG_TXDATA, reg);
    write_mmio(controller, I2C_REG_CMD, I2C_CMD_WRITE);

    /* Repeated START with read direction. */
    write_mmio(controller, I2C_REG_TXDATA, (uint8_t)((slave_addr << 1u) | 1u));
    write_mmio(controller, I2C_REG_CMD, I2C_CMD_START);

    write_mmio(controller, I2C_REG_CMD, I2C_CMD_READ);
    value = read_mmio(controller, I2C_REG_RXDATA);

    write_mmio(controller, I2C_REG_CMD, I2C_CMD_STOP);
    return (uint8_t)(value & 0xFFu);
}

static void test_device_id_read(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CController controller;
    uint8_t id;
    uint32_t status;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "example-sensor");
    assert(i2c_bus_attach(&bus, &sensor) == I2C_OK);

    i2c_controller_init(&controller, &bus);
    write_mmio(&controller, I2C_REG_CONTROL, I2C_CTRL_ENABLE);

    id = i2c_read_register(&controller, 0x50u, 0x00u);
    status = read_mmio(&controller, I2C_REG_STATUS);

    assert(id == 0xA5u);
    assert((status & I2C_STATUS_ERROR) == 0u);

    printf("PASS: device ID read returned 0x%02x\n", id);
}

static void test_register_write_and_readback(void)
{
    I2CBus bus;
    I2CEndpoint eeprom;
    I2CController controller;
    uint8_t value;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&eeprom, 0x51u, 0xE1u, "example-eeprom");
    assert(i2c_bus_attach(&bus, &eeprom) == I2C_OK);

    i2c_controller_init(&controller, &bus);
    write_mmio(&controller, I2C_REG_CONTROL, I2C_CTRL_ENABLE);

    i2c_write_register(&controller, 0x51u, 0x02u, 0x5Au);
    value = i2c_read_register(&controller, 0x51u, 0x02u);

    assert(value == 0x5Au);
    assert(i2c_endpoint_peek_register(&eeprom, 0x02u) == 0x5Au);

    printf("PASS: register write/readback returned 0x%02x\n", value);
}

static void test_multiple_endpoints(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CEndpoint rtc;
    I2CController controller;
    uint8_t sensor_id;
    uint8_t rtc_id;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "sensor");
    i2c_endpoint_init(&rtc, 0x68u, 0x68u, "rtc");
    assert(i2c_bus_attach(&bus, &sensor) == I2C_OK);
    assert(i2c_bus_attach(&bus, &rtc) == I2C_OK);
    assert(i2c_bus_endpoint_count(&bus) == 2u);

    i2c_controller_init(&controller, &bus);
    write_mmio(&controller, I2C_REG_CONTROL, I2C_CTRL_ENABLE);

    sensor_id = i2c_read_register(&controller, 0x50u, 0x00u);
    rtc_id = i2c_read_register(&controller, 0x68u, 0x00u);

    assert(sensor_id == 0xA5u);
    assert(rtc_id == 0x68u);

    printf("PASS: multiple endpoints routed correctly\n");
}

static void test_missing_slave_sets_error(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CController controller;
    uint32_t status;
    int rc;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "sensor");
    assert(i2c_bus_attach(&bus, &sensor) == I2C_OK);

    i2c_controller_init(&controller, &bus);
    write_mmio(&controller, I2C_REG_CONTROL, I2C_CTRL_ENABLE);

    write_mmio(&controller, I2C_REG_TXDATA, (uint8_t)(0x33u << 1u));
    rc = i2c_controller_mmio_write(&controller, I2C_REG_CMD, I2C_CMD_START);
    status = read_mmio(&controller, I2C_REG_STATUS);

    assert(rc == I2C_ERR_NO_SLAVE);
    assert((status & I2C_STATUS_ERROR) != 0u);
    assert(i2c_controller_state(&controller) == I2C_STATE_ERROR);

    printf("PASS: missing slave address produced ERROR status\n");
}

static void test_invalid_read_sequence_sets_error(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CController controller;
    uint32_t status;
    int rc;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "sensor");
    assert(i2c_bus_attach(&bus, &sensor) == I2C_OK);

    i2c_controller_init(&controller, &bus);
    write_mmio(&controller, I2C_REG_CONTROL, I2C_CTRL_ENABLE);

    rc = i2c_controller_mmio_write(&controller, I2C_REG_CMD, I2C_CMD_READ);
    status = read_mmio(&controller, I2C_REG_STATUS);

    assert(rc == I2C_ERR_INVALID_SEQUENCE);
    assert((status & I2C_STATUS_ERROR) != 0u);
    assert(i2c_controller_state(&controller) == I2C_STATE_ERROR);

    printf("PASS: READ without active transaction produced ERROR status\n");
}

static void test_stop_without_active_transaction_sets_error(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CController controller;
    uint32_t status;
    int rc;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "sensor");
    assert(i2c_bus_attach(&bus, &sensor) == I2C_OK);

    i2c_controller_init(&controller, &bus);
    write_mmio(&controller, I2C_REG_CONTROL, I2C_CTRL_ENABLE);

    rc = i2c_controller_mmio_write(&controller, I2C_REG_CMD, I2C_CMD_STOP);
    status = read_mmio(&controller, I2C_REG_STATUS);

    assert(rc == I2C_ERR_INVALID_SEQUENCE);
    assert((status & I2C_STATUS_ERROR) != 0u);
    assert(i2c_controller_state(&controller) == I2C_STATE_ERROR);

    printf("PASS: STOP without active transaction produced ERROR status\n");
}

static void test_controller_disabled_sets_error(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CController controller;
    uint32_t status;
    int rc;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "sensor");
    assert(i2c_bus_attach(&bus, &sensor) == I2C_OK);

    i2c_controller_init(&controller, &bus);

    write_mmio(&controller, I2C_REG_TXDATA, (uint8_t)(0x50u << 1u));
    rc = i2c_controller_mmio_write(&controller, I2C_REG_CMD, I2C_CMD_START);
    status = read_mmio(&controller, I2C_REG_STATUS);

    assert(rc == I2C_ERR_DISABLED);
    assert((status & I2C_STATUS_ERROR) != 0u);

    printf("PASS: command while disabled produced ERROR status\n");
}

static void test_duplicate_address_rejected(void)
{
    I2CBus bus;
    I2CEndpoint a;
    I2CEndpoint b;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&a, 0x50u, 0xA5u, "a");
    i2c_endpoint_init(&b, 0x50u, 0xB5u, "b");

    assert(i2c_bus_attach(&bus, &a) == I2C_OK);
    assert(i2c_bus_attach(&bus, &b) == I2C_ERR_DUPLICATE_ADDR);

    printf("PASS: duplicate slave address rejected\n");
}

static void test_status_write_one_to_clear(void)
{
    I2CBus bus;
    I2CEndpoint sensor;
    I2CController controller;
    uint32_t status;
    int rc;

    i2c_bus_init(&bus);
    i2c_endpoint_init(&sensor, 0x50u, 0xA5u, "sensor");
    assert(i2c_bus_attach(&bus, &sensor) == I2C_OK);

    i2c_controller_init(&controller, &bus);

    rc = i2c_controller_mmio_write(&controller, I2C_REG_CMD, I2C_CMD_START);
    assert(rc == I2C_ERR_DISABLED);

    status = read_mmio(&controller, I2C_REG_STATUS);
    assert((status & I2C_STATUS_ERROR) != 0u);

    write_mmio(&controller, I2C_REG_STATUS, I2C_STATUS_ERROR | I2C_STATUS_DONE);
    status = read_mmio(&controller, I2C_REG_STATUS);

    assert((status & I2C_STATUS_ERROR) == 0u);
    assert((status & I2C_STATUS_DONE) == 0u);

    printf("PASS: STATUS write-one-to-clear behavior works\n");
}

int main(void)
{
    test_device_id_read();
    test_register_write_and_readback();
    test_multiple_endpoints();
    test_missing_slave_sets_error();
    test_invalid_read_sequence_sets_error();
    test_stop_without_active_transaction_sets_error();
    test_controller_disabled_sets_error();
    test_duplicate_address_rejected();
    test_status_write_one_to_clear();

    printf("All I2C C model tests passed.\n");
    return 0;
}
