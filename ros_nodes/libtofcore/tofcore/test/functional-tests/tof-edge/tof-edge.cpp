/**
 * @file tof-vsm.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that sets/gets IPv4 settings
 */
#include "tofcore/tof_sensor.hpp"
#include <array>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <boost/program_options.hpp>

using namespace tofcore;
using namespace TofComm;
using namespace std::chrono_literals;
using namespace std::chrono;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };

static bool doEnableDisable { false };
static bool enableEdge { false };

static bool doAutoStart { false };
static bool autoStart { false };

static bool doGetVolume { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc(
                "Edge example\n\n"
                "  Usage: [options] []\n\n"
                "  EXAMPLES:\n"
                );
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("enable-edge,e", po::value<bool>(&enableEdge))
        ("auto-start,a", po::value<bool>(&autoStart))
        ("get-volume,g", po::value<bool>(&doGetVolume))
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    if (vm.count("enable-edge"))
    {
        doEnableDisable = true;
    }

    if (vm.count("auto-start"))
    {
        doAutoStart = true;
    }
}

static void signalHandler(int signum)
{
    (void)signum;
    exitRequested = true;
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

        if (doAutoStart)
        {
            constexpr uint16_t DISABLE_AUTO_START { 0x1002 };
            constexpr uint16_t ENABLE_AUTO_START  { 0x1003 };
            uint16_t command = (autoStart ? ENABLE_AUTO_START : DISABLE_AUTO_START);
            if (!sensor.send_receive(command, Sensor::send_receive_payload_t { }))
            {
                std::cerr << "Failed to enable/disable auto-start of edge application\n";
            }
            else
            {
                std::cout << "Edge Application AUTO-START " << (autoStart ? "ENABLED\n" : "DISABLED\n");
            }
        }

        if (doEnableDisable)
        {
            constexpr uint16_t EDGE_APP_DISABLE { 0x1000 };
            constexpr uint16_t EDGE_APP_ENABLE  { 0x1001 };
           uint16_t command = (enableEdge ? EDGE_APP_ENABLE : EDGE_APP_DISABLE);
            if (!sensor.send_receive(command, Sensor::send_receive_payload_t { }))
            {
                std::cerr << "Failed to enable/disable edge application\n";
            }
            else
            {
                std::cout << "Edge application " << (enableEdge ? "ENABLED\n" : "DISABLED\n");
            }
        }

        if (doGetVolume)
        {
            constexpr uint16_t GET_VOLUME_INFO { 0x1004 };
            auto volumeData = sensor.send_receive(GET_VOLUME_INFO, Sensor::send_receive_payload_t { });

             if (volumeData)
             {
                 std::cout << "Received " << volumeData->size() <<" bytes of volume data from sensor" << std::endl;
                 auto data = *volumeData;
                 uint32_t scaledResult = ((uint32_t)data[1] << 24) +
                                         ((uint32_t)data[2] << 16) +
                                         ((uint32_t)data[3] << 8)  + (uint32_t)data[4];
                 std::cout << "Scaled result: " << scaledResult << std::endl;
             }
             else
             {
                 std::cerr << "ERROR: No valid Volume data received" << std::endl;
             }
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
