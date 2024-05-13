/**
 * @file tof-imu.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcore to get various sensor values.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace TofComm;
using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool continuous {false};
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };

namespace po = boost::program_options;

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Tof IMU data");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("cont,c", po::bool_switch(&continuous)->default_value(false), "Continuous output of values, cntl-c to stop");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }
}

static void signalHandler(int signum)
{
    (void)signum;
    exitRequested = true;
}

static int imuDataDisplay()
{
    tofcore::Sensor sensor { devicePort, baudRate };
    /*
     * Imu information
     */
    auto data {sensor.getImuInfo()};

    if (data)
    {
        ImuScaledData_T &imuData = *data; 
       
        std::cout << "Imu Information" << std::endl
            << " accelerometer (xyz): (" << imuData.accelerometer_millig[0] << " milli-g, " << imuData.accelerometer_millig[1] << " milli-g, " << imuData.accelerometer_millig[2] << " milli-g)" << std::endl; 

        std::cout << " gyro (xyz): (" << imuData.gyro_milliDegreesPerSecond[0]  << " milli-deg/sec, " << imuData.gyro_milliDegreesPerSecond[1] << " milli-deg/sec, " << imuData.gyro_milliDegreesPerSecond[2] << " milli-deg/sec)" << std::endl; 

        std::cout << " temperature : " << imuData.temperature_milliDegreesC << " milli-DegC" << std::endl; 

        std::cout << " IMU timestamp : " << imuData.timestamp << " ms, " << std::endl; 
    }
    else
    {
        std::cerr << "Failed to retrieve IMU data" << std::endl;
        return -1;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    try
    {
        parseArgs(argc, argv);
    }
    catch (po::error &x)
    {
        std::cerr << x.what() << std::endl;
        return 1;
    }

    /*
     * Change default action of ^C, ^\ from abnormal termination in order to
     * perform a controlled shutdown.
     */
    signal(SIGINT, signalHandler);
    #if defined(SIGQUIT)
    signal(SIGQUIT, signalHandler);
    #endif
    do
    {
        imuDataDisplay();
        if (exitRequested)
            continuous = false;
    } while (continuous);

    return 0;
}
