import esphome.codegen as cg
from esphome import automation
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MAC_ADDRESS
from esphome.core import ID

CONF_NIMBLE_HOST_ID = "nimble_host_id"
CONF_SERVICE_UUID = "service_uuid"
CONF_NOTIFY_UUID = "notify_uuid"
CONF_WRITE_UUID = "write_uuid"
CONF_PASSKEY = "passkey"
CONF_AUTO_CONNECT = "auto_connect"
CONF_DISABLER_TAG = "disabler_tag"
CONF_RPM_SENSOR_ID = "rpm_sensor"
CONF_KPH_SENSOR_ID = "kph_sensor"
CONF_COOLANT_SENSOR_ID = "coolant_temp_sensor"
CONF_VOLTAGE_SENSOR_ID = "device_voltage_sensor"
CONF_UPTIME_SENSOR_ID = "uptime_sensor"

DEPENDENCIES = ["nimble_host", "sensor"]

nimble_host_ns = cg.esphome_ns.namespace("nimble_host")
NimbleHost = nimble_host_ns.class_("NimbleHost", cg.Component)

nimble_elm327_ns = cg.esphome_ns.namespace("nimble_elm327")
NimbleElm327 = nimble_elm327_ns.class_("NimbleElm327", cg.Component)
NimbleElm327WriteAction = nimble_elm327_ns.class_("NimbleElm327WriteAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NimbleElm327),
        cv.GenerateID(CONF_NIMBLE_HOST_ID): cv.use_id(NimbleHost),
        cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        cv.Required(CONF_SERVICE_UUID): cv.string,
        cv.Required(CONF_NOTIFY_UUID): cv.string,
        cv.Required(CONF_WRITE_UUID): cv.string,
        cv.Optional(CONF_PASSKEY, default=0): cv.int_,
        cv.Optional(CONF_AUTO_CONNECT, default=True): cv.boolean,
        cv.Optional(CONF_DISABLER_TAG): cv.string,
        cv.Optional(CONF_RPM_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_KPH_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_COOLANT_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_UPTIME_SENSOR_ID): cv.use_id(sensor.Sensor),
    }
).extend(cv.COMPONENT_SCHEMA)

WRITE_SCHEMA = cv.maybe_simple_value(
    {
        cv.GenerateID(): cv.use_id(NimbleElm327),
        cv.Required("value"): [cv.hex_uint8_t],
    },
    key="value",
)


@automation.register_action(
    "nimble_elm327.write", NimbleElm327WriteAction, WRITE_SCHEMA, synchronous=True
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
    cg.add(var.set_notify_uuid(config[CONF_NOTIFY_UUID]))
    cg.add(var.set_write_uuid(config[CONF_WRITE_UUID]))
    cg.add(var.set_passkey(config[CONF_PASSKEY]))
    cg.add(var.set_auto_connect(config[CONF_AUTO_CONNECT]))

    for conf_key, setter in (
        (CONF_RPM_SENSOR_ID, "set_rpm_sensor"),
        (CONF_KPH_SENSOR_ID, "set_kph_sensor"),
        (CONF_COOLANT_SENSOR_ID, "set_coolant_sensor"),
        (CONF_VOLTAGE_SENSOR_ID, "set_voltage_sensor"),
        (CONF_UPTIME_SENSOR_ID, "set_uptime_sensor"),
    ):
        if conf_key in config:
            sens = await cg.get_variable(config[conf_key])
            cg.add(getattr(var, setter)(sens))

    await cg.register_component(var, reg_comp_config)

    if disabler_wrap:
        cg.add(cg.RawStatement("}"))
