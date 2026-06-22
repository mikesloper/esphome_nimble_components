import pathlib

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import CONF_ID, CONF_MAC_ADDRESS
from esphome.core import CORE

CONF_NIMBLE_HOST_ID = "nimble_host_id"
CONF_NOTIFY_SERVICE_UUID = "notify_service_uuid"
CONF_NOTIFY_CHARACTERISTIC_UUID = "notify_characteristic_uuid"
CONF_WRITE_SERVICE_UUID = "write_service_uuid"
CONF_WRITE_CHARACTERISTIC_UUID = "write_characteristic_uuid"
CONF_PASSKEY = "passkey"
CONF_AUTO_CONNECT = "auto_connect"
CONF_DEVICE_MODEL = "device_model"
CONF_CHARGING_STATUS_TEXT_SENSOR_ID = "charging_status_text_sensor"
CONF_BATTERY_VOLTAGE_SENSOR_ID = "battery_voltage_sensor"
CONF_BATTERY_CURRENT_SENSOR_ID = "battery_current_sensor"
CONF_BATTERY_TEMPERATURE_SENSOR_ID = "battery_temperature_sensor"
CONF_PV_VOLTAGE_SENSOR_ID = "pv_voltage_sensor"
CONF_PV_CURRENT_SENSOR_ID = "pv_current_sensor"
CONF_PV_POWER_SENSOR_ID = "pv_power_sensor"
CONF_LOAD_VOLTAGE_SENSOR_ID = "load_voltage_sensor"
CONF_LOAD_CURRENT_SENSOR_ID = "load_current_sensor"
CONF_TOTAL_CURRENT_SENSOR_ID = "total_current_sensor"

DEPENDENCIES = ["nimble_host", "nimble_gap", "sensor", "text_sensor"]

lib_path = str(pathlib.Path(CORE.config_dir) / "lib")
cg.add_build_flag(f'-I"{lib_path}"')

nimble_host_ns = cg.esphome_ns.namespace("nimble_host")
NimbleHost = nimble_host_ns.class_("NimbleHost", cg.Component)

nimble_renogy_ns = cg.esphome_ns.namespace("nimble_renogy")
NimbleRenogy = nimble_renogy_ns.class_("NimbleRenogy", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NimbleRenogy),
        cv.GenerateID(CONF_NIMBLE_HOST_ID): cv.use_id(NimbleHost),
        cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        cv.Optional(CONF_NOTIFY_SERVICE_UUID, default="fff0"): cv.string,
        cv.Optional(CONF_NOTIFY_CHARACTERISTIC_UUID, default="fff1"): cv.string,
        cv.Optional(CONF_WRITE_SERVICE_UUID, default="ffd0"): cv.string,
        cv.Optional(CONF_WRITE_CHARACTERISTIC_UUID, default="ffd1"): cv.string,
        cv.Optional(CONF_DEVICE_MODEL, default=""): cv.string,
        cv.Optional(CONF_PASSKEY, default=0): cv.int_,
        cv.Optional(CONF_AUTO_CONNECT, default=True): cv.boolean,
        cv.Optional(CONF_CHARGING_STATUS_TEXT_SENSOR_ID): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_BATTERY_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_BATTERY_CURRENT_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_BATTERY_TEMPERATURE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_PV_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_PV_CURRENT_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_PV_POWER_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_LOAD_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_LOAD_CURRENT_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_TOTAL_CURRENT_SENSOR_ID): cv.use_id(sensor.Sensor),
    }
).extend(cv.polling_component_schema("10s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    host = await cg.get_variable(config[CONF_NIMBLE_HOST_ID])
    cg.add(var.set_nimble_host(host))
    cg.add(var.set_address(config[CONF_MAC_ADDRESS].as_hex))
    cg.add(var.set_notify_service_uuid(config[CONF_NOTIFY_SERVICE_UUID]))
    cg.add(var.set_notify_uuid(config[CONF_NOTIFY_CHARACTERISTIC_UUID]))
    cg.add(var.set_write_service_uuid(config[CONF_WRITE_SERVICE_UUID]))
    cg.add(var.set_write_uuid(config[CONF_WRITE_CHARACTERISTIC_UUID]))
    cg.add(var.set_device_model(config[CONF_DEVICE_MODEL]))
    cg.add(var.set_passkey(config[CONF_PASSKEY]))
    cg.add(var.set_auto_connect(config[CONF_AUTO_CONNECT]))

    for conf_key, setter in (
        (CONF_CHARGING_STATUS_TEXT_SENSOR_ID, "set_charging_status_text_sensor"),
        (CONF_BATTERY_VOLTAGE_SENSOR_ID, "set_battery_voltage_sensor"),
        (CONF_BATTERY_CURRENT_SENSOR_ID, "set_battery_current_sensor"),
        (CONF_BATTERY_TEMPERATURE_SENSOR_ID, "set_battery_temperature_sensor"),
        (CONF_PV_VOLTAGE_SENSOR_ID, "set_pv_voltage_sensor"),
        (CONF_PV_CURRENT_SENSOR_ID, "set_pv_current_sensor"),
        (CONF_PV_POWER_SENSOR_ID, "set_pv_power_sensor"),
        (CONF_LOAD_VOLTAGE_SENSOR_ID, "set_load_voltage_sensor"),
        (CONF_LOAD_CURRENT_SENSOR_ID, "set_load_current_sensor"),
        (CONF_TOTAL_CURRENT_SENSOR_ID, "set_total_current_sensor"),
    ):
        if conf_key in config:
            ent = await cg.get_variable(config[conf_key])
            cg.add(getattr(var, setter)(ent))

    await cg.register_component(var, config)
