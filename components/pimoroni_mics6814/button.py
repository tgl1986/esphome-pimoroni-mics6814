import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

from .sensor import pim_ns, PimoroniMics6814

PimoroniCalibrateButton = pim_ns.class_("PimoroniCalibrateButton", button.Button)

CONF_MICS_ID = "mics_id"

CONFIG_SCHEMA = button.button_schema(PimoroniCalibrateButton).extend(
    {
        cv.GenerateID(): cv.declare_id(PimoroniCalibrateButton),
        cv.Required(CONF_MICS_ID): cv.use_id(PimoroniMics6814),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_MICS_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await button.register_button(var, config)
