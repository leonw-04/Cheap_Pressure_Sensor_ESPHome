import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, STATE_CLASS_MEASUREMENT, DEVICE_CLASS_PRESSURE

from . import CheapPressureSensor, cheap_pressure_sensor_ns

CONF_CHEAP_PRESSURE_SENSOR_ID = 'cheap_pressure_sensor_id'
CONF_PRESSURE = 'pressure'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_CHEAP_PRESSURE_SENSOR_ID): cv.use_id(CheapPressureSensor),
    cv.Required(CONF_PRESSURE): sensor.sensor_schema(
        unit_of_measurement='bar',
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_PRESSURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_CHEAP_PRESSURE_SENSOR_ID])
    sens = await sensor.new_sensor(config[CONF_PRESSURE])
    cg.add(hub.set_pressure_sensor(sens))
