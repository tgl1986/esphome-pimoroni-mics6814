import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from .sensor import pim_ns, PimoroniMics6814

PimoroniBaselineNumber = pim_ns.class_("PimoroniBaselineNumber", number.Number)

CONF_MICS_ID = "mics_id"
CONF_CHANNEL = "channel"

CHANNELS = {
    "oxidising": 0,
    "reducing": 1,
    "nh3": 2,
}


CONFIG_SCHEMA = number.number_schema(PimoroniBaselineNumber).extend(
    {
        cv.GenerateID(): cv.declare_id(PimoroniBaselineNumber),
        cv.Required(CONF_MICS_ID): cv.use_id(PimoroniMics6814),
        cv.Required(CONF_CHANNEL): cv.enum(CHANNELS, lower=True),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MICS_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await number.register_number(var, config, min_value=0.0, max_value=5e6, step=1.0)
    cg.add(var.set_channel(config[CONF_CHANNEL]))