## Release 1.2 (Jan-2024)
- Implement Sequence Table for Integration Time and Modulation Frequency (MOS-1000)
- Command, Capture, and Calculation Threads (MOS-975)
- Add a message to CMake build files that compiler-rt is required for -fprofile-arcs (MOS-1078)
- Package cpp library in a deb package for Linux and an installer for Windows (MOS-920)
- Creates distributable python wheel files (MOS-910)
- Bug fixes:
    - Fix build Errors when including libtofcore using find_package (MOS-1024)
    - Don't install Google Test along with ToFCore (MOS-1056)
    - Change TofCore to be a shared library (MOS-1055)
    - Fix missing 3.11 pytofcrust wheel file (MOS-1003)

## Release 1.1
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

## Release 1.0
Initial release of libtofcore and libtofcrust