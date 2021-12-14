# Generic pwm controller that follows adc sensor measurement
#
# Copyright (C) 2016-2018  Aivars Vaivods <aivars@vaivods.lv>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import adc_temperature
from . import fan


SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 0.500
RANGE_CHECK_COUNT = 4
MIN_POSSIBLE_VAL = -9999999.9

# Controller has similar logic to temperature fan controller
#  in sense, that it tries to keep sensor value below target, by turning on PWM output
#from .import pysnooper
#@pysnooper.snoop(logging.info, watch=('self.target_value', 'self.output_active'))
class AdcPwmController:
    def __init__(self, config):
        # sensor specific init
        self.name = " ".join(config.get_name().split()[1:])
        self.precision = config.getint('sensor_measurement_precision', 2)
        self.units = config.get('sensor_units', '')
        params = []
        for i in range(1, 1000):
            t = config.getfloat('value%d' % (i,), None)
            if t is None:
                break
            v = config.getfloat('voltage%d' % (i,))
            params.append((t, v))
        self.adc_convert = adc_temperature.LinearVoltage(config, params)
        self.sensor_last_value = 0
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        mcu_adc = ppins.setup_pin('adc', config.get('sensor_pin'))
        mcu_adc.setup_adc_callback(REPORT_TIME, self.adc_callback)
        query_adc = self.printer.load_object(config, 'query_adc')
        query_adc.register_adc(config.get_name(), mcu_adc)
        self.sensor_min_value = config.getfloat('sensor_min_value', MIN_POSSIBLE_VAL,
                                         minval=MIN_POSSIBLE_VAL)
        self.sensor_max_value = config.getfloat('sensor_max_value', -1 * MIN_POSSIBLE_VAL,
                                         above=self.sensor_min_value)
        # Reusing interpolation code from temperature sensor
        adc_range = [self.adc_convert.calc_adc(v) for v in [self.sensor_min_value, self.sensor_max_value]]
        mcu_adc.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT,
                              minval=min(adc_range), maxval=max(adc_range),
                              range_check_count=RANGE_CHECK_COUNT)
        self.output_active = False
        # pwm specific init
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1,
                                               minval=0.)
        self.off_below = config.getfloat('off_below', default=0.,
                                         minval=0., maxval=1.)
        shutdown_value = config.getfloat(
            'shutdown_value', 0., minval=0., maxval=1.)
        self.pwm_last_value = 0.
        self.pwm_last_time = 0.
        self.target_value_conf = config.getfloat('target_value', 0.,
                minval=self.sensor_min_value, maxval=self.sensor_max_value)
        self.target_value = self.target_value_conf
        self.value_max_delta = config.getfloat('value_max_delta', 2.0, above=0.)

        gcode = self.printer.lookup_object('gcode')
        cycle_time = config.getfloat('cycle_time', 0.010, above=0.)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        self.mcu_pwm_pin = ppins.setup_pin('pwm', config.get('pwm_pin'))
        self.mcu_pwm_pin.setup_max_duration(0.)
        self.mcu_pwm_pin.setup_cycle_time(cycle_time, hardware_pwm)
        shutdown_power = max(0., min(self.max_power, shutdown_value))
        self.mcu_pwm_pin.setup_start_value(0., shutdown_power)

        self.printer.register_event_handler('gcode:request_restart',
                self._handle_request_restart)
        gcode.register_mux_command(
                'SET_ADC_PWM_CONTROLLER_TARGET', 'CONTROLLER', self.name,
                self.cmd_SET_ADC_PWM_CONTROLLER_TARGET,
                desc=self.desc_SET_ADC_PWM_CONTROLLER_TARGET)

    desc_SET_ADC_PWM_CONTROLLER_TARGET = 'Sets adc sensor target value'
    def cmd_SET_ADC_PWM_CONTROLLER_TARGET(self, gcmd):
        target = gcmd.get_float('TARGET', self.target_value_conf)
        self.set_target(target)
        gcmd.respond_info('%s target set to %f' %(self.name, target))

    def _handle_request_restart(self, print_time):
        self.set_pwm_value(print_time, 0.)

    def set_target(self, target):
        if target < self.sensor_min_value or target > self.sensor_max_value:
            raise self.printer.command_error(
                    'Requested adc target (%.2f) out of range (%.2f : %.2f)'
                    %(target, self.sensor_min_value, self.sensor_max_value))
        self.target_value = target

    def set_pwm_value(self, read_time, val):
        if val < self.off_below:
            val = 0.
        value = max(0., min(self.max_power, val * self.max_power))
        if abs(value - self.pwm_last_value) < 0.03:
            # small change - we can skip update
            return
        read_time = max(self.pwm_last_time + fan.FAN_MIN_TIME, read_time)
        read_time += adc_temperature.REPORT_TIME
        # do kick-starting
        if self.pwm_last_value == 0. and self.kick_start_time > 0. and value < self.max_power:
            self.mcu_pwm_pin.set_pwm(read_time, self.max_power)
            read_time += self.kick_start_time

        self.mcu_pwm_pin.set_pwm(read_time, value)
        self.pwm_last_time = read_time
        self.pwm_last_value = value

    def adc_callback(self, read_time, read_value):
        # Reusing interpolation code from temperature sensor
        value = self.adc_convert.calc_temp(read_value)
        self.sensor_last_value = value
        read_time += adc_temperature.SAMPLE_TIME * adc_temperature.SAMPLE_COUNT
        # Pwm logic similar to Bang-bang control algorithm from temperature fan
        # Keeps sensor value below target by activating pwm
        if value < self.target_value - self.value_max_delta and self.output_active:
            self.set_pwm_value(read_time, 0.)
            self.output_active = False
        elif value > self.target_value + self.value_max_delta and not self.output_active:
            self.set_pwm_value(read_time, 1.)
            self.output_active = True

    def get_status(self, eventtime):
        return {'value': round(self.sensor_last_value, self.precision),
                'target': round(self.target_value, self.precision),
                'units': self.units}

def load_config_prefix(config):
    return AdcPwmController(config)
