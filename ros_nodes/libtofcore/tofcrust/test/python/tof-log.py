import argparse
import signal
import sys

import pytofcrust

def signal_handler(sig, frame):
    print('stop requested')
    signal_handler.stop = True

signal_handler.stop = False
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser(epilog="EXAMPLE: tof-log.py  -g \"[\\\"INFO\\\",\\\"DBG\\\"]\" -c \"{\\\"DBG\\\":[\\\"COMMAND\\\"]}\"")
parser.add_argument('--port-name', default=pytofcrust.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('-c', '--category_levels', default=None, type=str, help="Set category levels (JSON).")
parser.add_argument('-g', '--global_enables', default=None, type=str,  help="Set global enable (JSON).")
args = parser.parse_args()

sensor = pytofcrust.Sensor(port_name=args.port_name)


def print_log_settings():
    log_settings = sensor.get_log_settings()
    print(log_settings)
    

def cleanup_and_exit(s:pytofcrust.Sensor):
    s = None
    sys.exit(0)

# Setup arg parser and run the tool

if args.global_enables or args.category_levels:
    if args.global_enables: global_enables = args.global_enables
    else: global_enables = "" 
    if args.category_levels: category_levels = args.category_levels
    else: category_levels = "" 
    sensor.set_log_settings(global_enables, args.category_levels)

print_log_settings()

cleanup_and_exit(sensor)
