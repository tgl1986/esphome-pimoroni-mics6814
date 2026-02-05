import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from .sensor import pim_ns, PimoroniMics6814

PimoroniHeaterSwitch = pim_ns.class_("PimoroniHeaterSwitch", switch.Switch)

CONF_MICS_ID = "mics_id"

CONFIG_SCHEMA = switch.switch_schema(PimoroniHeaterSwitch).extend(
    {
        cv.GenerateID(): cv.declare_id(PimoroniHeaterSwitch),
        cv.Required(CONF_MICS_ID): cv.use_id(PimoroniMics6814),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_MICS_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await switch.register_switch(var, config)
