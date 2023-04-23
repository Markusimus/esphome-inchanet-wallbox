import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, uart
from esphome.const import *
inchanet_wallbox_ns = cg.esphome_ns.namespace('inchanet_wallbox')
InchanetWallboxComponent = inchanet_wallbox_ns.class_('InchanetWallboxComponent', cg.PollingComponent)

DEPENDENCIES = ['uart']
AUTO_LOAD = ['uart', 'sensor', 'text_sensor']

ChargingCurrentOptions = inchanet_wallbox_ns.enum("ChargingCurrentOptions")
CHARGING_CURRENT_OPTIONS = {
    "DISABLED": ChargingCurrentOptions.CHARGING_DISABLED,
    "6A": ChargingCurrentOptions.CHARGING_CURRENT_6A,
    "7A": ChargingCurrentOptions.CHARGING_CURRENT_7A,
    "8A": ChargingCurrentOptions.CHARGING_CURRENT_8A,
    "9A": ChargingCurrentOptions.CHARGING_CURRENT_9A,
    "10A": ChargingCurrentOptions.CHARGING_CURRENT_10A,
    "11A": ChargingCurrentOptions.CHARGING_CURRENT_11A,
    "12A": ChargingCurrentOptions.CHARGING_CURRENT_12A,
    "13A": ChargingCurrentOptions.CHARGING_CURRENT_13A,
    "14A": ChargingCurrentOptions.CHARGING_CURRENT_14A,
    "15A": ChargingCurrentOptions.CHARGING_CURRENT_15A,
    "16A": ChargingCurrentOptions.CHARGING_CURRENT_16A,
}

CONF_INCHANET_WALLBOX_ID = "inchanet_wallbox_id"
CONF_SERIAL_NUMBER = "serial_number"
CONF_VOLTAGE_L1 = "voltage_l1"
CONF_VOLTAGE_L2 = "voltage_l2"
CONF_VOLTAGE_L3 = "voltage_l3"
CONF_CURRENT_L1 = "current_l1"
CONF_CURRENT_L2 = "current_l2"
CONF_CURRENT_L3 = "current_l3"
CONF_POWER_FACTOR_L1 = "power_factor_l1"
CONF_POWER_FACTOR_L2 = "power_factor_l2"
CONF_POWER_FACTOR_L3 = "power_factor_l3"
CONF_FREQUENCY_L1 = "frequency_l1"
CONF_FREQUENCY_L2 = "frequency_l2"
CONF_FREQUENCY_L3 = "frequency_l3"
CONF_EXTERNAL_CURRENT_L1 = "external_current_l1"
CONF_EXTERNAL_CURRENT_L2 = "external_current_l2"
CONF_EXTERNAL_CURRENT_L3 = "external_current_l3"
CONF_WH_IN_THIS_SESSION = "wh_in_this_session"
CONF_WH_ALL_TIME = "wh_all_time"
CONF_EVSE_TEMPERATURE = "evse_temperature"
CONF_STATE = "state"
CONF_STATE_OF_CHARGING = "state_of_charging"
CONF_STATE_OF_ELECTRIC_VEHICLE = "state_of_electric_vehicle"
CONF_STATE_OF_SOCKETS_LOCK = "state_of_sockets_lock"
CONF_WARNINGS = "warnings"
CONF_SERIOUS_ERRORS = "serious_errors"
CONF_MEASURED_PP_RESISTANCE = "measured_pp_resistance"

CONF_ENABLE_3_PHASE = "enable_3_phase"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(InchanetWallboxComponent),

    cv.Required(CONF_SERIAL_NUMBER): cv.positive_int,

    cv.Optional(CONF_VOLTAGE_L1):
      sensor.sensor_schema(device_class=DEVICE_CLASS_VOLTAGE,unit_of_measurement=UNIT_VOLT,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_VOLTAGE_L2):
      sensor.sensor_schema(device_class=DEVICE_CLASS_VOLTAGE,unit_of_measurement=UNIT_VOLT,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_VOLTAGE_L3):
      sensor.sensor_schema(device_class=DEVICE_CLASS_VOLTAGE,unit_of_measurement=UNIT_VOLT,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),

    cv.Optional(CONF_CURRENT_L1):
      sensor.sensor_schema(device_class=DEVICE_CLASS_CURRENT,unit_of_measurement=UNIT_AMPERE,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_CURRENT_L2):
      sensor.sensor_schema(device_class=DEVICE_CLASS_CURRENT,unit_of_measurement=UNIT_AMPERE,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_CURRENT_L3):
      sensor.sensor_schema(device_class=DEVICE_CLASS_CURRENT,unit_of_measurement=UNIT_AMPERE,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),

    cv.Optional(CONF_POWER_FACTOR_L1):
      sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR,unit_of_measurement=UNIT_EMPTY,accuracy_decimals=2,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_POWER_FACTOR_L2):
      sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR,unit_of_measurement=UNIT_EMPTY,accuracy_decimals=2,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_POWER_FACTOR_L3):
      sensor.sensor_schema(device_class=DEVICE_CLASS_POWER_FACTOR,unit_of_measurement=UNIT_EMPTY,accuracy_decimals=2,state_class=STATE_CLASS_MEASUREMENT).extend(),

    cv.Optional(CONF_FREQUENCY_L1):
      sensor.sensor_schema(device_class=DEVICE_CLASS_FREQUENCY,unit_of_measurement=UNIT_HERTZ,accuracy_decimals=1,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_FREQUENCY_L2):
      sensor.sensor_schema(device_class=DEVICE_CLASS_FREQUENCY,unit_of_measurement=UNIT_HERTZ,accuracy_decimals=1,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_FREQUENCY_L3):
      sensor.sensor_schema(device_class=DEVICE_CLASS_FREQUENCY,unit_of_measurement=UNIT_HERTZ,accuracy_decimals=1,state_class=STATE_CLASS_MEASUREMENT).extend(),

    cv.Optional(CONF_EXTERNAL_CURRENT_L1):
      sensor.sensor_schema(device_class=DEVICE_CLASS_CURRENT,unit_of_measurement=UNIT_AMPERE,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_EXTERNAL_CURRENT_L2):
      sensor.sensor_schema(device_class=DEVICE_CLASS_CURRENT,unit_of_measurement=UNIT_AMPERE,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),
    cv.Optional(CONF_EXTERNAL_CURRENT_L3):
      sensor.sensor_schema(device_class=DEVICE_CLASS_CURRENT,unit_of_measurement=UNIT_AMPERE,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),

    cv.Optional(CONF_WH_IN_THIS_SESSION):
      sensor.sensor_schema(device_class=DEVICE_CLASS_ENERGY,unit_of_measurement=UNIT_WATT_HOURS,accuracy_decimals=0,state_class=STATE_CLASS_MEASUREMENT).extend(),

    cv.Optional(CONF_WH_ALL_TIME):
      sensor.sensor_schema(device_class=DEVICE_CLASS_ENERGY,unit_of_measurement=UNIT_WATT_HOURS,accuracy_decimals=0,state_class=STATE_CLASS_TOTAL_INCREASING).extend(),

    cv.Optional(CONF_EVSE_TEMPERATURE):
      sensor.sensor_schema(device_class=DEVICE_CLASS_TEMPERATURE,unit_of_measurement=UNIT_CELSIUS,accuracy_decimals=1,state_class=STATE_CLASS_MEASUREMENT).extend(),

    cv.Optional(CONF_STATE):
      text_sensor.text_sensor_schema().extend(),

    cv.Optional(CONF_STATE_OF_CHARGING):
      text_sensor.text_sensor_schema().extend(),

    cv.Optional(CONF_STATE_OF_ELECTRIC_VEHICLE):
      text_sensor.text_sensor_schema().extend(),

    cv.Optional(CONF_STATE_OF_SOCKETS_LOCK):
      text_sensor.text_sensor_schema().extend(),

    cv.Optional(CONF_WARNINGS):
      text_sensor.text_sensor_schema().extend(),

    cv.Optional(CONF_SERIOUS_ERRORS):
      text_sensor.text_sensor_schema().extend(),

    cv.Optional(CONF_MEASURED_PP_RESISTANCE):
      text_sensor.text_sensor_schema().extend(),
}).extend(cv.polling_component_schema('10s')).extend(uart.UART_DEVICE_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)

    cg.add(var.set_evse_id(config[CONF_SERIAL_NUMBER]))

    if CONF_VOLTAGE_L1 in config:
      conf = config[CONF_VOLTAGE_L1]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_voltage_l1_sensor(sens))
    if CONF_VOLTAGE_L2 in config:
      conf = config[CONF_VOLTAGE_L2]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_voltage_l2_sensor(sens))
    if CONF_VOLTAGE_L3 in config:
      conf = config[CONF_VOLTAGE_L3]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_voltage_l3_sensor(sens))
    if CONF_CURRENT_L1 in config:
      conf = config[CONF_CURRENT_L1]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_current_l1_sensor(sens))
    if CONF_CURRENT_L2 in config:
      conf = config[CONF_CURRENT_L2]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_current_l2_sensor(sens))
    if CONF_CURRENT_L3 in config:
      conf = config[CONF_CURRENT_L3]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_current_l3_sensor(sens))
    if CONF_POWER_FACTOR_L1 in config:
      conf = config[CONF_POWER_FACTOR_L1]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_power_factor_l1_sensor(sens))
    if CONF_POWER_FACTOR_L2 in config:
      conf = config[CONF_POWER_FACTOR_L2]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_power_factor_l2_sensor(sens))
    if CONF_POWER_FACTOR_L3 in config:
      conf = config[CONF_POWER_FACTOR_L3]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_power_factor_l3_sensor(sens))
    if CONF_FREQUENCY_L1 in config:
      conf = config[CONF_FREQUENCY_L1]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_frequency_l1_sensor(sens))
    if CONF_FREQUENCY_L2 in config:
      conf = config[CONF_FREQUENCY_L2]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_frequency_l2_sensor(sens))
    if CONF_FREQUENCY_L3 in config:
      conf = config[CONF_FREQUENCY_L3]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_frequency_l3_sensor(sens))
    if CONF_EXTERNAL_CURRENT_L1 in config:
      conf = config[CONF_EXTERNAL_CURRENT_L1]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_external_current_l1_sensor(sens))
    if CONF_EXTERNAL_CURRENT_L2 in config:
      conf = config[CONF_EXTERNAL_CURRENT_L2]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_external_current_l2_sensor(sens))
    if CONF_EXTERNAL_CURRENT_L3 in config:
      conf = config[CONF_EXTERNAL_CURRENT_L3]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_external_current_l3_sensor(sens))
    if CONF_WH_IN_THIS_SESSION in config:
      conf = config[CONF_WH_IN_THIS_SESSION]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_wh_in_this_session_sensor(sens))
    if CONF_WH_ALL_TIME in config:
      conf = config[CONF_WH_ALL_TIME]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_wh_all_time_sensor(sens))
    if CONF_EVSE_TEMPERATURE in config:
      conf = config[CONF_EVSE_TEMPERATURE]
      sens = yield sensor.new_sensor(conf)
      cg.add(var.set_evse_temperature_sensor(sens))
    if CONF_STATE in config:
      conf = config[CONF_STATE]
      sens = yield text_sensor.new_text_sensor(conf)
      cg.add(var.set_state_sensor(sens))
    if CONF_STATE_OF_CHARGING in config:
      conf = config[CONF_STATE_OF_CHARGING]
      sens = yield text_sensor.new_text_sensor(conf)
      cg.add(var.set_state_of_charging_sensor(sens))
    if CONF_STATE_OF_ELECTRIC_VEHICLE in config:
      conf = config[CONF_STATE_OF_ELECTRIC_VEHICLE]
      sens = yield text_sensor.new_text_sensor(conf)
      cg.add(var.set_state_of_electric_vehicle_sensor(sens))
    if CONF_STATE_OF_SOCKETS_LOCK in config:
      conf = config[CONF_STATE_OF_SOCKETS_LOCK]
      sens = yield text_sensor.new_text_sensor(conf)
      cg.add(var.set_state_of_sockets_lock_sensor(sens))
    if CONF_WARNINGS in config:
      conf = config[CONF_WARNINGS]
      sens = yield text_sensor.new_text_sensor(conf)
      cg.add(var.set_warnings_sensor(sens))
    if CONF_SERIOUS_ERRORS in config:
      conf = config[CONF_SERIOUS_ERRORS]
      sens = yield text_sensor.new_text_sensor(conf)
      cg.add(var.set_serious_errors_sensor(sens))
    if CONF_MEASURED_PP_RESISTANCE in config:
      conf = config[CONF_MEASURED_PP_RESISTANCE]
      sens = yield text_sensor.new_text_sensor(conf)
      cg.add(var.set_measured_pp_resistance_sensor(sens))
