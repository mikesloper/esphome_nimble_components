import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CURRENT,
    CONF_POWER,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)

from . import CONF_NIMBLE_JBD_BMS_ID, NIMBLE_JBD_BMS_COMPONENT_SCHEMA

DEPENDENCIES = ["nimble_jbd_bms"]

CONF_STATE_OF_CHARGE = "state_of_charge"
CONF_NOMINAL_CAPACITY = "nominal_capacity"
CONF_CHARGING_CYCLES = "charging_cycles"
CONF_CAPACITY_REMAINING = "capacity_remaining"
CONF_TOTAL_VOLTAGE = "total_voltage"
CONF_CHARGING_POWER = "charging_power"
CONF_DISCHARGING_POWER = "discharging_power"
CONF_MIN_CELL_VOLTAGE = "min_cell_voltage"
CONF_MAX_CELL_VOLTAGE = "max_cell_voltage"
CONF_MIN_VOLTAGE_CELL = "min_voltage_cell"
CONF_MAX_VOLTAGE_CELL = "max_voltage_cell"
CONF_DELTA_CELL_VOLTAGE = "delta_cell_voltage"
CONF_AVERAGE_CELL_VOLTAGE = "average_cell_voltage"
CONF_CELL_COUNT = "cell_count"

CELLS = [f"cell_voltage_{i}" for i in range(1, 33)]
TEMPERATURES = [f"temperature_{i}" for i in range(1, 7)]

SENSOR_DEFS = {
    CONF_STATE_OF_CHARGE: {
        "unit_of_measurement": UNIT_PERCENT,
        "accuracy_decimals": 0,
        "device_class": DEVICE_CLASS_BATTERY,
    },
    CONF_TOTAL_VOLTAGE: {
        "unit_of_measurement": UNIT_VOLT,
        "accuracy_decimals": 2,
        "device_class": DEVICE_CLASS_VOLTAGE,
    },
    CONF_CURRENT: {
        "unit_of_measurement": UNIT_AMPERE,
        "accuracy_decimals": 1,
        "device_class": DEVICE_CLASS_CURRENT,
    },
    CONF_POWER: {
        "unit_of_measurement": UNIT_WATT,
        "accuracy_decimals": 1,
        "device_class": DEVICE_CLASS_POWER,
    },
    CONF_CHARGING_POWER: {
        "unit_of_measurement": UNIT_WATT,
        "accuracy_decimals": 2,
        "device_class": DEVICE_CLASS_POWER,
    },
    CONF_DISCHARGING_POWER: {
        "unit_of_measurement": UNIT_WATT,
        "accuracy_decimals": 2,
        "device_class": DEVICE_CLASS_POWER,
    },
    CONF_NOMINAL_CAPACITY: {
        "unit_of_measurement": "Ah",
        "accuracy_decimals": 2,
        "device_class": DEVICE_CLASS_EMPTY,
    },
    CONF_CHARGING_CYCLES: {
        "accuracy_decimals": 0,
        "device_class": DEVICE_CLASS_EMPTY,
    },
    CONF_CAPACITY_REMAINING: {
        "unit_of_measurement": "Ah",
        "accuracy_decimals": 2,
        "device_class": DEVICE_CLASS_EMPTY,
    },
    CONF_MIN_CELL_VOLTAGE: {
        "unit_of_measurement": UNIT_VOLT,
        "accuracy_decimals": 3,
        "device_class": DEVICE_CLASS_VOLTAGE,
    },
    CONF_MAX_CELL_VOLTAGE: {
        "unit_of_measurement": UNIT_VOLT,
        "accuracy_decimals": 3,
        "device_class": DEVICE_CLASS_VOLTAGE,
    },
    CONF_MIN_VOLTAGE_CELL: {
        "accuracy_decimals": 0,
        "device_class": DEVICE_CLASS_EMPTY,
    },
    CONF_MAX_VOLTAGE_CELL: {
        "accuracy_decimals": 0,
        "device_class": DEVICE_CLASS_EMPTY,
    },
    CONF_DELTA_CELL_VOLTAGE: {
        "unit_of_measurement": UNIT_VOLT,
        "accuracy_decimals": 4,
        "device_class": DEVICE_CLASS_VOLTAGE,
    },
    CONF_AVERAGE_CELL_VOLTAGE: {
        "unit_of_measurement": UNIT_VOLT,
        "accuracy_decimals": 4,
        "device_class": DEVICE_CLASS_VOLTAGE,
    },
    CONF_CELL_COUNT: {
        "accuracy_decimals": 0,
        "device_class": DEVICE_CLASS_EMPTY,
    },
}

_CELL_VOLTAGE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=3,
    device_class=DEVICE_CLASS_VOLTAGE,
)

_TEMPERATURE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
)

CONFIG_SCHEMA = NIMBLE_JBD_BMS_COMPONENT_SCHEMA.extend(
    {cv.Optional(key): sensor.sensor_schema(**kwargs) for key, kwargs in SENSOR_DEFS.items()}
).extend({cv.Optional(key): _CELL_VOLTAGE_SCHEMA for key in CELLS}).extend(
    {cv.Optional(key): _TEMPERATURE_SCHEMA for key in TEMPERATURES}
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_NIMBLE_JBD_BMS_ID])
    for i, key in enumerate(TEMPERATURES):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(hub.set_temperature_sensor(i, sens))
    for i, key in enumerate(CELLS):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(hub.set_cell_voltage_sensor(i, sens))
    for key in SENSOR_DEFS:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
