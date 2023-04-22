from esphome.components import switch
import esphome.config_validation as cv
import esphome.codegen as cg
from . import inchanet_wallbox_ns, CONF_INCHANET_WALLBOX_ID, InchanetWallboxComponent

DEPENDENCIES = ["inchanet_wallbox"]

InchanetWallboxSwitch = inchanet_wallbox_ns.class_("InchanetWallboxSwitch", switch.Switch, cg.Component)

CONFIG_SCHEMA = (
    switch.switch_schema(InchanetWallboxSwitch).extend(
        {
            cv.GenerateID(CONF_INCHANET_WALLBOX_ID): cv.use_id(InchanetWallboxComponent),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_INCHANET_WALLBOX_ID])
    cg.add(var.set_inchanet_wallbox_parent(paren))