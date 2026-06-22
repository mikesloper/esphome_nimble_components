import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_NIMBLE_HOST_ID = "nimble_host_id"

DEPENDENCIES = ["nimble_host"]

nimble_host_ns = cg.esphome_ns.namespace("nimble_host")
NimbleHost = nimble_host_ns.class_("NimbleHost", cg.Component)

nimble_gap_ns = cg.esphome_ns.namespace("nimble_gap")
NimbleGapCoordinator = nimble_gap_ns.class_("NimbleGapCoordinator", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NimbleGapCoordinator),
        cv.Required(CONF_NIMBLE_HOST_ID): cv.use_id(NimbleHost),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    cg.add_define("USE_NIMBLE_GAP")
    var = cg.new_Pvariable(config[CONF_ID])
    host = await cg.get_variable(config[CONF_NIMBLE_HOST_ID])
    cg.add(var.set_nimble_host(host))
    await cg.register_component(var, config)
