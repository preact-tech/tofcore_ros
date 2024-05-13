/**
 * @file get-unit-vector.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that libtofcore to read Lens unit vector.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace tofcore;

inline constexpr unsigned NUM_ROWS { 240 };
inline constexpr unsigned NUM_COLS { 320 };

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Get Lens Information Test");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ;

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

static void printRay(const std::vector<double>& ray, const char* rayName)
{
    std::cout << "double " << rayName << "[" << NUM_ROWS << "][" << NUM_COLS << "] =\n{\n";
    for (unsigned row = 0; row < NUM_ROWS; ++row)
    {
        std::cout << "  /* ROW: " << std::setw(3) << std::setfill('0') << row << " */ {";
        for (unsigned col = 0; col < NUM_COLS; ++col)
        {
            std::cout << ray[row];
            if (col != (NUM_COLS - 1))
            {
                std::cout << ", ";
            }
        }
        std::cout << "}";
        if (row != (NUM_ROWS - 1))
        {
            std::cout << ",";
        }
        std::cout << "\n";
    }
    std::cout << "};\n";

}

int main(int argc, char *argv[])
{
    parseArgs(argc, argv);
    /*
     * Change default action of ^C, ^\ from abnormal termination in order to
     * perform a controlled shutdown.
     */
    signal(SIGINT, signalHandler);
#if defined(SIGQUIT)
    signal(SIGQUIT, signalHandler);
#endif
    {
        tofcore::Sensor sensor { devicePort, baudRate };

        std::vector<double> rays_x;
        std::vector<double> rays_y;
        std::vector<double> rays_z;
        if (sensor.getLensInfo(rays_x, rays_y, rays_z))
        {
            printRay(rays_x, "rays_x");
            printRay(rays_y, "rays_y");
            printRay(rays_z, "rays_z");
        }
        else
        {
            std::cerr << "Unable to read unit vector data" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
