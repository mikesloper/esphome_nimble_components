import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv

from . import CONF_NIMBLE_JBD_BMS_ID, NIMBLE_JBD_BMS_COMPONENT_SCHEMA

DEPENDENCIES = ["nimble_jbd_bms"]

CONF_ERRORS = "errors"
CONF_OPERATION_STATUS = "operation_status"

CONFIG_SCHEMA = NIMBLE_JBD_BMS_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_ERRORS): text_sensor.text_sensor_schema(
            icon="mdi:alert-circle-outline",
        ),
        cv.Optional(CONF_OPERATION_STATUS): text_sensor.text_sensor_schema(
            icon="mdi:heart-pulse",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_NIMBLE_JBD_BMS_ID])
    for key in (CONF_ERRORS, CONF_OPERATION_STATUS):
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
