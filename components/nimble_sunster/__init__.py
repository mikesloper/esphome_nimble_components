import pathlib

import esphome.codegen as cg
from esphome import automation
from esphome.components import binary_sensor, number, sensor, text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MAC_ADDRESS
from esphome.core import CORE, ID

CONF_NIMBLE_HOST_ID = "nimble_host_id"
CONF_SERVICE_UUID = "service_uuid"
CONF_CHARACTERISTIC_UUID = "characteristic_uuid"
CONF_PASSKEY = "passkey"
CONF_AUTO_CONNECT = "auto_connect"
CONF_DISABLER_TAG = "disabler_tag"
CONF_RUNNING_BINARY_SENSOR_ID = "running_binary_sensor"
CONF_GLOW_PLUG_TEXT_SENSOR_ID = "glow_plug_text_sensor"
CONF_MODE_TEXT_SENSOR_ID = "mode_text_sensor"
CONF_ERROR_CODE_SENSOR_ID = "error_code_sensor"
CONF_BATTERY_VOLTAGE_SENSOR_ID = "battery_voltage_sensor"
CONF_ALTITUDE_SENSOR_ID = "altitude_sensor"
CONF_RUNNING_STATUS_CODE_SENSOR_ID = "running_status_code_sensor"
CONF_ROOM_TEMPERATURE_SENSOR_ID = "room_temperature_sensor"
CONF_HEATING_TEMPERATURE_SENSOR_ID = "heating_temperature_sensor"
CONF_HEATER_LEVEL_NUMBER_ID = "heater_level_number"
CONF_HEATER_TEMPERATURE_NUMBER_ID = "heater_temperature_number"

DEPENDENCIES = ["nimble_host", "nimble_gap", "binary_sensor", "sensor", "text_sensor", "number"]

lib_path = str(pathlib.Path(CORE.config_dir) / "lib")
cg.add_build_flag(f'-I"{lib_path}"')

nimble_host_ns = cg.esphome_ns.namespace("nimble_host")
NimbleHost = nimble_host_ns.class_("NimbleHost", cg.Component)

nimble_sunster_ns = cg.esphome_ns.namespace("nimble_sunster")
NimbleSunster = nimble_sunster_ns.class_("NimbleSunster", cg.Component)
NimbleSunsterWriteAction = nimble_sunster_ns.class_("NimbleSunsterWriteAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NimbleSunster),
        cv.GenerateID(CONF_NIMBLE_HOST_ID): cv.use_id(NimbleHost),
        cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        cv.Required(CONF_SERVICE_UUID): cv.string,
        cv.Required(CONF_CHARACTERISTIC_UUID): cv.string,
        cv.Optional(CONF_PASSKEY, default=0): cv.int_,
        cv.Optional(CONF_AUTO_CONNECT, default=True): cv.boolean,
        cv.Optional(CONF_DISABLER_TAG): cv.string,
        cv.Optional(CONF_RUNNING_BINARY_SENSOR_ID): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_GLOW_PLUG_TEXT_SENSOR_ID): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_MODE_TEXT_SENSOR_ID): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_ERROR_CODE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_BATTERY_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_ALTITUDE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_RUNNING_STATUS_CODE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_ROOM_TEMPERATURE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_HEATING_TEMPERATURE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_HEATER_LEVEL_NUMBER_ID): cv.use_id(number.Number),
        cv.Optional(CONF_HEATER_TEMPERATURE_NUMBER_ID): cv.use_id(number.Number),
    }
).extend(cv.COMPONENT_SCHEMA)

WRITE_SCHEMA = cv.maybe_simple_value(
    {
        cv.GenerateID(): cv.use_id(NimbleSunster),
        cv.Required("value"): [cv.hex_uint8_t],
    },
    key="value",
)


@automation.register_action(
    "nimble_sunster.write", NimbleSunsterWriteAction, WRITE_SCHEMA, synchronous=True
)
async def write_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    data = config["value"]
    arr_id = ID(f"{action_id}_data", is_declaration=True, type=cg.uint8)
    arr = cg.static_const_array(arr_id, cg.ArrayInitializer(*data))
    cg.add(var.set_data(arr, len(data)))
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    disabler_wrap = CONF_DISABLER_TAG in config
    reg_comp_config = config
    if disabler_wrap:
        tag = config[CONF_DISABLER_TAG]
        cg.add(cg.RawStatement(f'if(disabler_disabler_id->exists("{tag}")) {{'))
        reg_comp_config = {k: v for k, v in config.items() if k != CONF_DISABLER_TAG}

    host = await cg.get_variable(config[CONF_NIMBLE_HOST_ID])
    cg.add(var.set_nimble_host(host))
    cg.add(var.set_address(config[CONF_MAC_ADDRESS].as_hex))
    cg.add(var.set_service_uuid(config[CONF_SERVICE_UUID]))
    cg.add(var.set_characteristic_uuid(config[CONF_CHARACTERISTIC_UUID]))
    cg.add(var.set_passkey(config[CONF_PASSKEY]))
    cg.add(var.set_auto_connect(config[CONF_AUTO_CONNECT]))

    for conf_key, setter in (
        (CONF_RUNNING_BINARY_SENSOR_ID, "set_running_binary_sensor"),
        (CONF_GLOW_PLUG_TEXT_SENSOR_ID, "set_glow_plug_text_sensor"),
        (CONF_MODE_TEXT_SENSOR_ID, "set_mode_text_sensor"),
        (CONF_ERROR_CODE_SENSOR_ID, "set_error_code_sensor"),
        (CONF_BATTERY_VOLTAGE_SENSOR_ID, "set_battery_voltage_sensor"),
        (CONF_ALTITUDE_SENSOR_ID, "set_altitude_sensor"),
        (CONF_RUNNING_STATUS_CODE_SENSOR_ID, "set_running_status_code_sensor"),
        (CONF_ROOM_TEMPERATURE_SENSOR_ID, "set_room_temperature_sensor"),
        (CONF_HEATING_TEMPERATURE_SENSOR_ID, "set_heating_temperature_sensor"),
        (CONF_HEATER_LEVEL_NUMBER_ID, "set_heater_level_number"),
        (CONF_HEATER_TEMPERATURE_NUMBER_ID, "set_heater_temperature_number"),
    ):
        if conf_key in config:
            ent = await cg.get_variable(config[conf_key])
            cg.add(getattr(var, setter)(ent))

    await cg.register_component(var, reg_comp_config)

    if disabler_wrap:
        cg.add(cg.RawStatement("}"))
