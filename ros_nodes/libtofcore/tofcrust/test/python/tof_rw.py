'''
tof_rw.py

Read and write data from the sensor. This test can be extended as needed to test various functionalities of the python bindings.
'''

#For this import the pytofcore.*.so module file must be someplace in the PYTHONPATH
# this can be done by adding the build directory to PYTHONPATH (e.g. `export PTYHONPATH=$PYTHONPATH:<path-to-build-dir`)
# or by installing the library to your python site-packages directory. 

import argparse
import signal
import pytofcrust
import time
import timeit

def signal_handler(sig, frame):
    print('stop requested')
    signal_handler.stop = True

signal_handler.stop = False
signal.signal(signal.SIGINT, signal_handler)

parser = argparse.ArgumentParser()
parser.add_argument('--port-name', default=pytofcrust.Sensor.DEFAULT_PORT_NAME, type=str, help="Port name to connect to sensor on.")
parser.add_argument('-d', '--get_test_station_data', default=True, action='store_false', help="Get and print the test station data string stored on the sensor.")
parser.add_argument('-D', '--test_station_data_string', default=None, type=str, help="Write the given test station data string to the sensor.")
parser.add_argument('--set_ib_serial', default=None, type=str, help='Set the illuminator board serial number. Usage: [--set_ib_serial <serial_string>]')
args = parser.parse_args()

s = pytofcrust.Sensor(port_name=args.port_name)


if args.test_station_data_string is not None:
    s.set_factory_mode(True)
    ok = s.set_test_station_data(args.test_station_data_string)
    if not ok:
        print("Error trying to set the test station data!")

if args.set_ib_serial:
    if s.set_illuminator_serial(args.set_ib_serial):
        print(f"Setting Illuminator Board serial number: {args.set_ib_serial}")
    else:
        print("ERROR setting Illuminator serial!")

print("\n")

if args.get_test_station_data:
    test_station_data = s.get_test_station_data()
    print(f"Test Station Data String: {test_station_data}")

print("\n")

sensor_info = s.get_sensor_info()
print("SENSOR INFO")
for name in sensor_info._fields:
    print(f"{name}: {getattr(sensor_info, name)}")
