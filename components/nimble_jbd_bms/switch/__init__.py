import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv

from .. import CONF_NIMBLE_JBD_BMS_ID, NIMBLE_JBD_BMS_COMPONENT_SCHEMA, nimble_jbd_bms_ns
from ..const import CONF_CHARGING, CONF_DISCHARGING

DEPENDENCIES = ["nimble_jbd_bms"]

SWITCHES = {
    CONF_DISCHARGING: [0xE1, 1],
    CONF_CHARGING: [0xE1, 0],
}

NimbleJbdSwitch = nimble_jbd_bms_ns.class_("NimbleJbdSwitch", switch.Switch, cg.Component)

CONFIG_SCHEMA = NIMBLE_JBD_BMS_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_DISCHARGING): switch.switch_schema(
            NimbleJbdSwitch,
            icon="mdi:battery-charging-50",
        ),
        cv.Optional(CONF_CHARGING): switch.switch_schema(
            NimbleJbdSwitch,
            icon="mdi:battery-charging-50",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_NIMBLE_JBD_BMS_ID])
    for key, address in SWITCHES.items():
        if key in config:
            var = await switch.new_switch(config[key])
            await cg.register_component(var, config[key])
            cg.add(getattr(hub, f"set_{key}_switch")(var))
            cg.add(var.set_parent(hub))
            cg.add(var.set_address(address[0]))
            cg.add(var.set_bitmask(address[1]))
