import argparse
import signal
import sys

import pytofcrust

def signal_handler(sig, frame):
    print('stop requested')
    signal_handler.stop = True

signal_handler.stop = False
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser(epilog="EXAMPLE: tof-therm.py  -e 85.0 -x 84.0")
parser.add_argument('--port-name', default=pytofcrust.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('-e','--entry_degC', type=float, default=None, help="Set degC value for entry to thermal protection mode.")
parser.add_argument('-x','--exit_degC', type=float, default=None,  help="Set degC value for exit from thermal protection mode.")
args = parser.parse_args()

sensor = pytofcrust.Sensor(port_name=args.port_name)


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
setThresholds = False
if args.entry_degC: 
    entryDegC = args.entry_degC;
    if (args.exit_degC):
        exitDegC = args.exit_degC
    else:
        exitDegC = entryDegC - 0.5
    setThresholds = True
elif args.exit_degC:
    exitDegC = args.exit_degC
    entryDegC = exitDegC + 0.5
    setThresholds = True
    
if setThresholds:
    print("Setting thresholds to (", entryDegC, ",", exitDegC,")")
    sensor.set_factory_mode(True)
    sensor.set_thermal_limits(entryDegC, exitDegC)
    
limits = sensor.get_thermal_limits()

print("Thresholds read:", limits)

#print_log_settings()

cleanup_and_exit(sensor)
