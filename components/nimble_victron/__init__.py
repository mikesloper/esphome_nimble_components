import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_BINDKEY, CONF_ID, CONF_MAC_ADDRESS
from esphome.yaml_util import ESPHomeDumper

CONF_NIMBLE_HOST_ID = "nimble_host_id"
CONF_BATTERY_VOLTAGE_SENSOR_ID = "battery_voltage_sensor"
CONF_BATTERY_CURRENT_SENSOR_ID = "battery_current_sensor"
CONF_STATE_OF_CHARGE_SENSOR_ID = "state_of_charge_sensor"
CONF_CONSUMED_AH_SENSOR_ID = "consumed_ah_sensor"
CONF_TIME_TO_GO_SENSOR_ID = "time_to_go_sensor"
CONF_AUX_VOLTAGE_SENSOR_ID = "aux_voltage_sensor"
CONF_PV_POWER_SENSOR_ID = "pv_power_sensor"
CONF_SOLAR_BATTERY_VOLTAGE_SENSOR_ID = "solar_battery_voltage_sensor"
CONF_SOLAR_BATTERY_CURRENT_SENSOR_ID = "solar_battery_current_sensor"

DEPENDENCIES = ["nimble_host", "sensor"]
MULTI_CONF = True

nimble_host_ns = cg.esphome_ns.namespace("nimble_host")
NimbleHost = nimble_host_ns.class_("NimbleHost", cg.Component)

nimble_victron_ns = cg.esphome_ns.namespace("nimble_victron")
NimbleVictron = nimble_victron_ns.class_("NimbleVictron", cg.Component)


class Array:
    def __init__(self, *parts):
        self.parts = parts

    def __str__(self):
        return "".join(f"{part:02X}" for part in self.parts)

    @property
    def as_array(self):
        from esphome.cpp_generator import RawExpression

        num = ", 0x".join(f"{part:02X}" for part in self.parts)
        return RawExpression(f"{{0x{num}}}")


ESPHomeDumper.add_multi_representer(Array, ESPHomeDumper.represent_stringify)


def bind_key_array(value):
    value = cv.bind_key(value)
    parts = [value[i : i + 2] for i in range(0, len(value), 2)]
    return Array(*(int(part, 16) for part in parts))


CONFIG_SCHEMA = cv.All(
    cv.only_on_esp32,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NimbleVictron),
            cv.GenerateID(CONF_NIMBLE_HOST_ID): cv.use_id(NimbleHost),
            cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
            cv.Required(CONF_BINDKEY): bind_key_array,
            cv.Optional(CONF_BATTERY_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_BATTERY_CURRENT_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_STATE_OF_CHARGE_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_CONSUMED_AH_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_TIME_TO_GO_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_AUX_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_PV_POWER_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_SOLAR_BATTERY_VOLTAGE_SENSOR_ID): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_SOLAR_BATTERY_CURRENT_SENSOR_ID): cv.use_id(sensor.Sensor),
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    host = await cg.get_variable(config[CONF_NIMBLE_HOST_ID])
    cg.add(var.set_nimble_host(host))
    cg.add(var.set_address(config[CONF_MAC_ADDRESS].as_hex))
    cg.add(var.set_bindkey(config[CONF_BINDKEY].as_array))

    for conf_key, setter in (
        (CONF_BATTERY_VOLTAGE_SENSOR_ID, "set_battery_voltage_sensor"),
        (CONF_BATTERY_CURRENT_SENSOR_ID, "set_battery_current_sensor"),
        (CONF_STATE_OF_CHARGE_SENSOR_ID, "set_state_of_charge_sensor"),
        (CONF_CONSUMED_AH_SENSOR_ID, "set_consumed_ah_sensor"),
        (CONF_TIME_TO_GO_SENSOR_ID, "set_time_to_go_sensor"),
        (CONF_AUX_VOLTAGE_SENSOR_ID, "set_aux_voltage_sensor"),
        (CONF_PV_POWER_SENSOR_ID, "set_pv_power_sensor"),
        (CONF_SOLAR_BATTERY_VOLTAGE_SENSOR_ID, "set_solar_battery_voltage_sensor"),
        (CONF_SOLAR_BATTERY_CURRENT_SENSOR_ID, "set_solar_battery_current_sensor"),
    ):
        if conf_key in config:
            sens = await cg.get_variable(config[conf_key])
            cg.add(getattr(var, setter)(sens))

    await cg.register_component(var, config)
