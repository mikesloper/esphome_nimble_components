import esphome.codegen as cg
from esphome import automation
from esphome.components.esp32 import add_idf_sdkconfig_option
import esphome.config_validation as cv
from esphome.const import CONF_ENABLE_ON_BOOT, CONF_ID

nimble_host_ns = cg.esphome_ns.namespace("nimble_host")
NimbleHost = nimble_host_ns.class_("NimbleHost", cg.Component)
NimbleHostEnableAction = nimble_host_ns.class_("NimbleHostEnableAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NimbleHost),
        cv.Optional(CONF_ENABLE_ON_BOOT, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


def _configure_nimble_sdkconfig() -> None:
    add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_CONTROLLER_DISABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", False)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_TRANSPORT_UART", False)
    add_idf_sdkconfig_option("CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE", True)
    add_idf_sdkconfig_option("CONFIG_ESP_HOSTED_NIMBLE_HCI_VHCI", True)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_CENTRAL", True)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_PERIPHERAL", False)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_BROADCASTER", False)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_OBSERVER", True)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_GATT_CLIENT", True)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_MAX_CONNECTIONS", 4)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_MAX_BONDS", 3)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_LEGACY", True)
    add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_BONDING", True)


async def to_code(config):
    cg.add_define("USE_NIMBLE_HOST")
    _configure_nimble_sdkconfig()

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_enable_on_boot(config[CONF_ENABLE_ON_BOOT]))


@automation.register_action(
    "nimble_host.enable",
    NimbleHostEnableAction,
    cv.Schema({cv.GenerateID(): cv.use_id(NimbleHost)}),
    synchronous=True,
)
async def nimble_host_enable_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
