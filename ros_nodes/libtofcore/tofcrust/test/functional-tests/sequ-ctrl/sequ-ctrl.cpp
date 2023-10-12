/**
 * @file tof-stat.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to get various sensor values.
 */
#include "tofcrust/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <boost/program_options.hpp>


using namespace tofcrust;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static int checkVersion { -1 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool getVersion { false };
static uint16_t protocolVersion { 1 };
static int setVersion { -1 };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("illuminator board test");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("check-supported,c", po::value<int>(&checkVersion)->default_value(-1), "Check that the provided sequencer version is supported")
        ("get-version,g",  po::bool_switch(&getVersion), "Read the configured sequencer version")
        ("set-version,s", po::value<int>(&setVersion)->default_value(-1), "Set the sequencer version")
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
        tofcrust::Sensor sensor { protocolVersion, devicePort, baudRate };
        /*
         * Check sequencer version for support
         */
        if (checkVersion >= 0)
        {
            const auto result { sensor.sequencerIsVersionSupported(checkVersion) };
            if (result.has_value())
            {
                const bool isSupported { *result };
                std::cout << "Sequencer version " << checkVersion << (isSupported ? " is" : " is not") << " supported" << std::endl;
            }
            else
            {
                std::cerr << "Failed to check sequencer version" << std::endl;
            }
        }
        /*
         * Set sequencer version
         */
        if (setVersion >= 0)
        {
            const bool result { sensor.sequencerSetVersion(setVersion) };
            if (result)
            {
                std::cout << "Sequencer version set to " << setVersion << std::endl;
            }
            else
            {
                std::cerr << "Failed to set sequencer version to " << setVersion << std::endl;
            }
        }
       /*
         * Get sequencer version
         */
        if (getVersion)
        {
            const auto result { sensor.sequencerGetVersion() };
            if (result)
            {
                const auto version { *result };
                std::cout << "Sequencer version is " << version << std::endl;
            }
            else
            {
                std::cerr << "Failed to get sequencer version" << std::endl;
            }
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
