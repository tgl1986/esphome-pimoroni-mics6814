import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_NONE,
    STATE_CLASS_MEASUREMENT,
)

pim_ns = cg.esphome_ns.namespace("pimoroni_mics6814")
PimoroniMics6814 = pim_ns.class_("PimoroniMics6814", cg.PollingComponent, i2c.I2CDevice)

CONF_OXIDISING = "oxidising"
CONF_REDUCING = "reducing"
CONF_NH3 = "nh3"
CONF_REF = "ref_voltage"

CONF_R0_OXIDISING = "r0_oxidising"
CONF_R0_REDUCING = "r0_reducing"
CONF_R0_NH3 = "r0_nh3"

CONF_RATIO_OXIDISING = "ratio_oxidising"
CONF_RATIO_REDUCING = "ratio_reducing"
CONF_RATIO_NH3 = "ratio_nh3"

SENSOR_SCHEMA_OHM = sensor.sensor_schema(
    unit_of_measurement="Ω",
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_NONE,
    state_class=STATE_CLASS_MEASUREMENT,
)

SENSOR_SCHEMA_RATIO = sensor.sensor_schema(
    unit_of_measurement="",
    accuracy_decimals=4,
    device_class=DEVICE_CLASS_NONE,
    state_class=STATE_CLASS_MEASUREMENT,
)

SENSOR_SCHEMA_V = sensor.sensor_schema(
    unit_of_measurement="V",
    accuracy_decimals=3,
    device_class=DEVICE_CLASS_NONE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PimoroniMics6814),
            cv.Optional(CONF_ADDRESS, default=0x19): cv.i2c_address,
            cv.Optional(CONF_UPDATE_INTERVAL, default="10s"): cv.update_interval,

            cv.Required(CONF_OXIDISING): SENSOR_SCHEMA_OHM,
            cv.Required(CONF_REDUCING): SENSOR_SCHEMA_OHM,
            cv.Required(CONF_NH3): SENSOR_SCHEMA_OHM,

            cv.Optional(CONF_REF): SENSOR_SCHEMA_V,

            cv.Optional(CONF_R0_OXIDISING): SENSOR_SCHEMA_OHM,
            cv.Optional(CONF_R0_REDUCING): SENSOR_SCHEMA_OHM,
            cv.Optional(CONF_R0_NH3): SENSOR_SCHEMA_OHM,

            cv.Optional(CONF_RATIO_OXIDISING): SENSOR_SCHEMA_RATIO,
            cv.Optional(CONF_RATIO_REDUCING): SENSOR_SCHEMA_RATIO,
            cv.Optional(CONF_RATIO_NH3): SENSOR_SCHEMA_RATIO,
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(i2c.i2c_device_schema(0x19))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    s_ox = await sensor.new_sensor(config[CONF_OXIDISING])
    s_red = await sensor.new_sensor(config[CONF_REDUCING])
    s_nh3 = await sensor.new_sensor(config[CONF_NH3])

    cg.add(var.set_oxidising_sensor(s_ox))
    cg.add(var.set_reducing_sensor(s_red))
    cg.add(var.set_nh3_sensor(s_nh3))

    if CONF_REF in config:
        s_ref = await sensor.new_sensor(config[CONF_REF])
        cg.add(var.set_ref_sensor(s_ref))

    if CONF_R0_OXIDISING in config:
        s = await sensor.new_sensor(config[CONF_R0_OXIDISING])
        cg.add(var.set_r0_oxidising_sensor(s))
    if CONF_R0_REDUCING in config:
        s = await sensor.new_sensor(config[CONF_R0_REDUCING])
        cg.add(var.set_r0_reducing_sensor(s))
    if CONF_R0_NH3 in config:
        s = await sensor.new_sensor(config[CONF_R0_NH3])
        cg.add(var.set_r0_nh3_sensor(s))

    if CONF_RATIO_OXIDISING in config:
        s = await sensor.new_sensor(config[CONF_RATIO_OXIDISING])
        cg.add(var.set_ratio_oxidising_sensor(s))
    if CONF_RATIO_REDUCING in config:
        s = await sensor.new_sensor(config[CONF_RATIO_REDUCING])
        cg.add(var.set_ratio_reducing_sensor(s))
    if CONF_RATIO_NH3 in config:
        s = await sensor.new_sensor(config[CONF_RATIO_NH3])
        cg.add(var.set_ratio_nh3_sensor(s))
