from esphome.components import select
import esphome.config_validation as cv
import esphome.codegen as cg
from . import inchanet_wallbox_ns, CONF_INCHANET_WALLBOX_ID, InchanetWallboxComponent

DEPENDENCIES = ["inchanet_wallbox"]

InchanetWallboxSelect = inchanet_wallbox_ns.class_("InchanetWallboxSelect", select.Select, cg.PollingComponent)

CONF_SELECT_TYPE = "type"

SelectTyps = inchanet_wallbox_ns.enum("SelectTyps")
SELECT_TYPE = {
    "max_charge_current": SelectTyps.MAX_CHARGE_CURRENT,
    "default_charge_current": SelectTyps.DEFAUT_CHARGE_CURRENT,
}

ChargingCurrentOption = inchanet_wallbox_ns.enum("ChargingCurrentOptios")
CHARGING_CURRENT_OPTION = {
    "OFF": ChargingCurrentOption.CHARGING_DISABLED,
    "6A":  ChargingCurrentOption.CHARGING_CURRENT_6A,
    "7A":  ChargingCurrentOption.CHARGING_CURRENT_7A,
    "8A":  ChargingCurrentOption.CHARGING_CURRENT_8A,
    "9A":  ChargingCurrentOption.CHARGING_CURRENT_9A,
    "10A": ChargingCurrentOption.CHARGING_CURRENT_10A,
    "11A": ChargingCurrentOption.CHARGING_CURRENT_11A,
    "12A": ChargingCurrentOption.CHARGING_CURRENT_12A,
    "13A": ChargingCurrentOption.CHARGING_CURRENT_13A,
    "14A": ChargingCurrentOption.CHARGING_CURRENT_14A,
    "15A": ChargingCurrentOption.CHARGING_CURRENT_15A,
    "16A": ChargingCurrentOption.CHARGING_CURRENT_16A,
}

CONFIG_SCHEMA = (
    select.select_schema(InchanetWallboxSelect).extend(
        {
            cv.GenerateID(CONF_INCHANET_WALLBOX_ID): cv.use_id(InchanetWallboxComponent),
            cv.Required(CONF_SELECT_TYPE): cv.enum(SELECT_TYPE),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    options_map = CHARGING_CURRENT_OPTION;
    select_type = SELECT_TYPE[config[CONF_SELECT_TYPE]]

    var = await select.new_select(config, options=list(options_map.keys()))
    await cg.register_component(var, config)
    paren = await cg.get_variable(config[CONF_INCHANET_WALLBOX_ID])

    cg.add(var.set_inchanet_wallbox_parent(paren))
    cg.add(var.set_select_type(select_type))
    cg.add(var.set_select_mappings(list(options_map.values())))