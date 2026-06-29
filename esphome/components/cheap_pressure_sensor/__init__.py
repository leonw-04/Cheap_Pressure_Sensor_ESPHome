import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ['uart']
AUTO_LOAD = ['sensor']

cheap_pressure_sensor_ns = cg.esphome_ns.namespace('cheap_pressure_sensor')
CheapPressureSensor = cheap_pressure_sensor_ns.class_('CheapPressureSensor', cg.PollingComponent, uart.UARTDevice)

CONF_SLAVE_ID = 'slave_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CheapPressureSensor),
    cv.Optional(CONF_SLAVE_ID, default=1): cv.int_range(min=1, max=255),
}).extend(cv.polling_component_schema('60s')).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
