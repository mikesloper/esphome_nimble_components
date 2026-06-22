import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MAC_ADDRESS, CONF_PASSWORD

CONF_NIMBLE_HOST_ID = "nimble_host_id"
CONF_NIMBLE_JBD_BMS_ID = "nimble_jbd_bms_id"
CONF_SERVICE_UUID = "service_uuid"
CONF_NOTIFY_CHARACTERISTIC_UUID = "notify_characteristic_uuid"
CONF_CONTROL_CHARACTERISTIC_UUID = "control_characteristic_uuid"
CONF_AUTH_TIMEOUT = "auth_timeout"
CONF_AUTO_CONNECT = "auto_connect"

DEPENDENCIES = ["nimble_host", "nimble_gap"]
AUTO_LOAD = ["sensor", "text_sensor", "switch"]
MULTI_CONF = True

nimble_host_ns = cg.esphome_ns.namespace("nimble_host")
NimbleHost = nimble_host_ns.class_("NimbleHost", cg.Component)

nimble_jbd_bms_ns = cg.esphome_ns.namespace("nimble_jbd_bms")
NimbleJbdBms = nimble_jbd_bms_ns.class_("NimbleJbdBms", cg.PollingComponent)

NIMBLE_JBD_BMS_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_NIMBLE_JBD_BMS_ID): cv.use_id(NimbleJbdBms),
    }
)


def validate_password(value):
    if not value:
        return value
    if len(value) != 6:
        raise cv.Invalid("Password must be exactly 6 characters long")
    allowed = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
    for char in value:
        if char not in allowed:
            raise cv.Invalid(f"Password contains invalid character '{char}'")
    return value


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NimbleJbdBms),
        cv.GenerateID(CONF_NIMBLE_HOST_ID): cv.use_id(NimbleHost),
        cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        cv.Optional(CONF_SERVICE_UUID, default="ff00"): cv.string,
        cv.Optional(CONF_NOTIFY_CHARACTERISTIC_UUID, default="ff01"): cv.string,
        cv.Optional(CONF_CONTROL_CHARACTERISTIC_UUID, default="ff02"): cv.string,
        cv.Optional(CONF_PASSWORD, default=""): cv.All(cv.string_strict, validate_password),
        cv.Optional(CONF_AUTH_TIMEOUT, default="10s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_AUTO_CONNECT, default=True): cv.boolean,
    }
).extend(cv.polling_component_schema("8s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    host = await cg.get_variable(config[CONF_NIMBLE_HOST_ID])
    cg.add(var.set_nimble_host(host))
    cg.add(var.set_address(config[CONF_MAC_ADDRESS].as_hex))
    cg.add(var.set_service_uuid(config[CONF_SERVICE_UUID]))
    cg.add(var.set_notify_uuid(config[CONF_NOTIFY_CHARACTERISTIC_UUID]))
    cg.add(var.set_control_uuid(config[CONF_CONTROL_CHARACTERISTIC_UUID]))
    cg.add(var.set_auto_connect(config[CONF_AUTO_CONNECT]))
    if config[CONF_PASSWORD]:
        cg.add(var.set_password(config[CONF_PASSWORD]))
    cg.add(var.set_authentication_timeout(config[CONF_AUTH_TIMEOUT]))
    await cg.register_component(var, config)
