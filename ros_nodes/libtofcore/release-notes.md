# Release 1.1

- Added support for continuous modulation frequency (MOS-442)
- Added internet protocol support (MOS-749)
    - Implemented URI scheme for specifying how to connect to a sensor
        - Examples for serial (USB) connections:
            - "tofserial:/dev/ttyACM0?baudrate=115200;protocol_version=0"
            - "/dev/ttyACM0"
            - "COM0?baudrate=9600"
        - Examples for ethernet connections:
            - "tofnet://10.10.31.180:50660"
            - "tofnet://10.10.0.180"
            - "10.10.31.180"
            - "tofnet://dotted.host.name?protocol_version=1"
- Added python bindings to jump-to-bootloader (MOS-740)
- Updated Lens info API (MOS-468)
- Remove accelerometer, ROI, grayscrale streaming from tofcore API (MOS-595)
- Adds sensor location and name settings (getters and setters) (MOS-743)

# Release 1.0
Initial release of libtofcore and libtofcrust