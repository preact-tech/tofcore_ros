/**
 * @file tof-sdram-test.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Initiate SDRAM test on TOF sensor.
 */
#include "tofcrust/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace tofcore;
namespace po = boost::program_options;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool deferReset { false };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool skipFactoryMode { false };

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Request SDRAM test upon next sensor power-up");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("defer-reset,d", po::bool_switch(&deferReset), "Defer reset command until later")
        ("skip-factory-mode,s", po::bool_switch(&skipFactoryMode), "Skip factory mode to verify request will be rejected")
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
    (void) signum;
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
        tofcrust::Sensor sensor { devicePort, baudRate };
        if (!skipFactoryMode)
        {
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "setFactoryMode() FAILED" << std::endl;
                exit(-1);
            }
        }
        if (sensor.sdramTestRequest())
        {
            std::cout << "SDRAM test will occur on next sensor power-up" << std::endl;
            if (!deferReset)
            {
                constexpr uint16_t FULL_RESET_TOKEN { 0x0155 };
                std::cout << "Restarting sensor to run SDRAM test" << std::endl;
                sensor.jumpToBootloader(FULL_RESET_TOKEN);
            }
        }
        else
        {
            std::cerr << "SDRAM test request rejected" << std::endl;
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
