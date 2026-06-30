import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID, 
    STATE_CLASS_MEASUREMENT, 
    DEVICE_CLASS_PRESSURE,
    ENTITY_CATEGORY_DIAGNOSTIC
)

from . import CheapPressureSensor, cheap_pressure_sensor_ns

CONF_CHEAP_PRESSURE_SENSOR_ID = 'cheap_pressure_sensor_id'
CONF_PRESSURE = 'pressure'
CONF_UNIT = 'unit'
CONF_DECIMAL_PLACES = 'decimal_places'
CONF_UNIT_TEXT = 'unit_text'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_CHEAP_PRESSURE_SENSOR_ID): cv.use_id(CheapPressureSensor),
    cv.Required(CONF_PRESSURE): sensor.sensor_schema(
        unit_of_measurement='bar',
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_PRESSURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_UNIT): sensor.sensor_schema(
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    cv.Optional(CONF_DECIMAL_PLACES): sensor.sensor_schema(
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    cv.Optional(CONF_UNIT_TEXT): text_sensor.text_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_CHEAP_PRESSURE_SENSOR_ID])
    
    sens = await sensor.new_sensor(config[CONF_PRESSURE])
    cg.add(hub.set_pressure_sensor(sens))
    
    if CONF_UNIT in config:
        sens = await sensor.new_sensor(config[CONF_UNIT])
        cg.add(hub.set_unit_sensor(sens))
        
    if CONF_DECIMAL_PLACES in config:
        sens = await sensor.new_sensor(config[CONF_DECIMAL_PLACES])
        cg.add(hub.set_decimal_places_sensor(sens))
        
    if CONF_UNIT_TEXT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_UNIT_TEXT])
        cg.add(hub.set_unit_text_sensor(sens))
