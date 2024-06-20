#!/bin/python3

import argparse
import signal
import sys

import pytofcrust
from scipy.stats._mstats_basic import mode

def signal_handler(sig, frame):
    print('stop requested')
    signal_handler.stop = True

signal_handler.stop = False
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser(epilog="EXAMPLE: tof-limits.py -p /dev/ttyACM0 -m 30 -M 3600000")
parser.add_argument('-p','--port_name', default=pytofcrust.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name for connecting to sensor.")
parser.add_argument('-a','--min_amplitude_min', type=int, default=None, help="Minimum minimum amplitude setting.")
parser.add_argument('-A','--min_amplitude_max', type=int, default=None, help="Maximum minimum amplitude setting..")
parser.add_argument('-f','--mod_freq_khz_min', type=int, default=None, help="Minimum modulation frequency in kHz.")
parser.add_argument('-F','--mod_freq_khz_max', type=int, default=None, help="Maximum modulation frequency in kHz.")
parser.add_argument('-S','--mod_freq_khz_step', type=int, default=None, help="Modulation frequency step size in kHz.")
parser.add_argument('-i','--integ_time_us_min', type=int, default=None, help="Minimum integration time in uS.")
parser.add_argument('-I','--integ_time_us_max', type=int, default=None, help="Maximum integration time in uS.")
parser.add_argument('-m','--min_frame_period_ms', type=int, default=None, help="Minimum frame period (mS).")
parser.add_argument('-M','--max_frame_period_ms', type=int, default=None,  help="Maximum frame period (mS).")
parser.add_argument('-s','--skip_factory_mode', type=bool, default=None,  help="Skip factory mode request.")
parser.add_argument('-w','--write_values', type=bool, default=None,  help="Write values to persistent storage.")
args = parser.parse_args()

sensor = pytofcrust.Sensor(port_name=args.port_name)

def print_frame_period_and_limits():
    frame_period_data = sensor.get_frame_period_and_limits()
    frame_period, minimum_frame_period, maximum_frame_period = frame_period_data
    print(
        f"Frame Period\n"
        f"\tcurrent:\t{frame_period} mS\n"
        f"\tminimum:\t{minimum_frame_period} mS\n"
        f"\tmaximum:\t{maximum_frame_period} mS")

def print_integration_time_and_limits():
    integration_time_data = sensor.get_integration_time_and_limits()
    integration_time, minimum_integration_time, maximum_integration_time = integration_time_data
    print(
        f"Integration Time\n"
        f"\tcurrent:\t{integration_time} uS\n"
        f"\tminimum:\t{minimum_integration_time} uS\n"
        f"\tmaximum:\t{maximum_integration_time} uS")

def print_modulation_frequency_and_limits():
    modulation_freq_data = sensor.get_modfreq_and_limits_and_step()
    modulation_freq, minimum_modulation_freq, maximum_modulation_freq, modulation_freq_step = modulation_freq_data
    print(
        f"Modulation Frequency\n"
        f"\tcurrent:\t{modulation_freq} kHz\n"
        f"\tminimum:\t{minimum_modulation_freq} kHz\n"
        f"\tmaximum:\t{maximum_modulation_freq} kHz\n"
        f"\tstep size:\t{modulation_freq_step} kHz")

def print_max_vsm_elements():
    max_vsm_elements = sensor.get_vsm_max_number_of_elements()
    print(
        f"VSM\n"
        f"\tMaximum # of elements:\t{max_vsm_elements}")

def print_min_amplitude_and_limits():
    min_amplitude_data = sensor.get_min_amplitude_and_limits()
    min_amplitude, minimum_min_amplitude, maximum_min_amplitude = min_amplitude_data
    print(
        f"Minimum Amplitude\n"
        f"\tcurrent:\t{min_amplitude}\n"
        f"\tminimum:\t{minimum_min_amplitude}\n"
        f"\tmaximum:\t{maximum_min_amplitude}")

def cleanup_and_exit(s:pytofcrust.Sensor):
    s = None
    sys.exit(0)

# Setup arg parser and run the tool

#if args.global_enables or args.category_levels:
#    if args.global_enables: global_enables = args.global_enables
#    else: global_enables = "" 
#    if args.category_levels: category_levels = args.category_levels
#    else: category_levels = "" 
#    sensor.set_log_settings(global_enables, args.category_levels)
skipFactoryMode = False
setFramePeriodLimits = False
setIntegTimeLimits = False
setModFreqLimits = False
setMinAmplitudeLimits = False
writeValues = False

if args.skip_factory_mode:
    skipFactoryMode = True
    
if args.write_values:
    writeValues = True
    
# Check frame period arguments
if args.min_frame_period_ms: 
    minFramePeriodMs = args.min_frame_period_ms;
    if (args.max_frame_period_ms):
        maxFramePeriodMs = args.max_frame_period_ms
    else:
        print("\nERROR: You must specify BOTH the min. and max. frame period in order to change either of them.\n")
        cleanup_and_exit()
    setFramePeriodLimits = True
elif args.max_frame_period_ms:
        print("\nERROR: You must specify BOTH the min. and max. frame period in order to change either of them.\n")
        cleanup_and_exit()
    
# Check integration time arguments
if args.integ_time_us_min: 
    minIntegTimeUs = args.integ_time_us_min;
    if (args.integ_time_us_max):
        maxIntegTimeUs = args.integ_time_us_max
    else:
        print("\nERROR: You must specify BOTH the min. and max. integration time in order to change either of them.\n")
        cleanup_and_exit()
    setIntegTimeLimits = True
elif args.integ_time_us_max:
     print("\nERROR: You must specify BOTH the min. and max. integration time in order to change either of them.\n")
     cleanup_and_exit()
    
# Check modulation frequency arguments
if args.mod_freq_khz_min: 
    minModFreqKhz = args.mod_freq_khz_min;
    if (args.mod_freq_khz_max):
        maxModFreqKhz = args.mod_freq_khz_max
    else:
        print("\nERROR: You must specify BOTH the min. and max. modulation frequency in order to change either of them.\n")
        cleanup_and_exit()
    setModFreqLimits = True
elif args.mod_freq_khz_max:
        print("\nERROR: You must specify BOTH the min. and max. modulation frequency in order to change either of them.\n")
        cleanup_and_exit()
    
if setModFreqLimits:
    if args.mod_freq_khz_step:
        modFreqKhzStep = args.mod_freq_khz_step
    else:
        modFreqKhzStep = 1
        
# Check minimum amplitude time arguments
if args.min_amplitude_min: 
    minMinAmplitude = args.min_amplitude_min;
    if (args.min_amplitude_max):
        maxMinAmplitude = args.min_amplitude_max
    else:
        print("\nERROR: You must specify BOTH the min. and max. minimum amplitude in order to change either of them.\n")
        cleanup_and_exit()
    setMinAmplitudeLimits = True
elif args.min_amplitude_max:
     print("\nERROR: You must specify BOTH the min. and max. minimum amplitude in order to change either of them.\n")
     cleanup_and_exit()
        
    
if setFramePeriodLimits:
    if not skipFactoryMode:
        sensor.set_factory_mode(True)
    print("Setting frame period min/max to (", minFramePeriodMs, ",", maxFramePeriodMs,")")
    sensor.set_frame_period_limits(minFramePeriodMs, maxFramePeriodMs)
    sensor.set_factory_mode(False)
      
if setIntegTimeLimits:
    if not skipFactoryMode:
        sensor.set_factory_mode(True)
    print("Setting integration time min/max to (", minIntegTimeUs, ",", maxIntegTimeUs,")")
    sensor.set_integration_time_limits(minIntegTimeUs, maxIntegTimeUs)
    sensor.set_factory_mode(False)
    
if setModFreqLimits:
    if not skipFactoryMode:
        sensor.set_factory_mode(True)
    print("Setting modulation frequency min/max/step to (", minModFreqKhz, ",", maxModFreqKhz, ",", modFreqKhzStep, ")")
    sensor.set_modulation_frequency_limits(minModFreqKhz, maxModFreqKhz, modFreqKhzStep)
    sensor.set_factory_mode(False)
      
if setMinAmplitudeLimits:
    if not skipFactoryMode:
        sensor.set_factory_mode(True)
    print("Setting minimum amplitude min/max to (", minMinAmplitude, ",", maxMinAmplitude,")")
    sensor.set_min_amplitude_limits(minMinAmplitude, maxMinAmplitude)
    sensor.set_factory_mode(False)
    
# Write to persistent storage   
if writeValues:
    sensor.storeSettings()
   
# Print current values
print_frame_period_and_limits()
print_integration_time_and_limits()
print_modulation_frequency_and_limits()
print_max_vsm_elements()
print_min_amplitude_and_limits()

cleanup_and_exit(sensor)
