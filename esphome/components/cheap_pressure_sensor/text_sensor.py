import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CheapPressureSensor, cheap_pressure_sensor_ns

CONF_CHEAP_PRESSURE_SENSOR_ID = 'cheap_pressure_sensor_id'
CONF_UNIT_TEXT = 'unit_text'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_CHEAP_PRESSURE_SENSOR_ID): cv.use_id(CheapPressureSensor),
    cv.Required(CONF_UNIT_TEXT): text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_CHEAP_PRESSURE_SENSOR_ID])
    
    if CONF_UNIT_TEXT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_UNIT_TEXT])
        cg.add(hub.set_unit_text_sensor(sens))
