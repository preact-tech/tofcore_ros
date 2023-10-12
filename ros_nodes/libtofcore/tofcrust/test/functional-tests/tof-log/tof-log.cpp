/**
 * @file tof-rw.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program to read Tof persistent storage
 */
#include "crc32.h"
#include "tofcrust/tof_sensor.hpp"
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace std::chrono;
using namespace tofcrust;
using namespace TofComm;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool categoryReset { false };
static std::string categoryStr { };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool globalReset { false };
static std::string globalStr { };
static uint16_t protocolVersion { 1 }; //Note we really need to use protocolVersion 1 for these large messages
static bool storeSettings { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Read/Write sensor logging settings");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("category,c",  po::value<std::string>(&categoryStr), "Set category levels for those described in JSON string")
        ("global,g", po::value<std::string>(&globalStr), "Reset global enables to those described in JSON string")
        ("store-settings,s", po::bool_switch(&storeSettings), "Have sensor store current sensor settings in persistent memory")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n"
            << "NOTES:" << std::endl
            << "  1) Category levels remain unchanged until explicitly set to another value (or sensor reset)" << std::endl
            << "  2) Settings do not persist unless the persistent settings are EXPLICITLY updated (see -s)" << std::endl
            << "  3) FAULT and WARN do not have to be explicitly enabled via -g since they cannot be disabled" << std::endl << std::endl
            << "EXAMPLE - Enable the DBG logging for COMMAND (plus INFO for SYSTEM, STREAM, NONVOL):" << std::endl << std::endl
            << "  tof-log -g \"[\\\"INFO\\\",\\\"DBG\\\"]\" -c \"{\\\"DBG\\\":[\\\"COMMAND\\\"],\\\"INFO\\\":[\\\"NONVOL\\\",\\\"STREAM\\\",\\\"SYSTEM\\\"]}\"" << std::endl << std::endl
            << "EXAMPLE - Enable DBG for STREAM, INFO only for SYSTEM and save as persistent start-up levels:" << std::endl << std::endl
            << "  tof-log -s -g \"[\\\"INFO\\\",\\\"DBG\\\"]\" -c \"{\\\"FAULT\\\":[\\\"COMMAND\\\",\\\"NONVOL\\\"],\\\"INFO\\\":[\\\"SYSTEM\\\"],\\\"DBG\\\":[\\\"STREAM\\\"]}\""
            << std::endl << std::endl;

        exit(0);
    }

    categoryReset = (vm.count("category") != 0);
    globalReset = (vm.count("global") != 0);
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

        std::optional<const char*> categoryLevels = std::nullopt;
        if (categoryReset)
        {
            categoryLevels = categoryStr.c_str();
            std::cout << "Category setting to send: '" << *categoryLevels << "'" << std::endl;
        }

        std::optional<const char*> globalEnables = std::nullopt;
        if (globalReset)
        {
            globalEnables = globalStr.c_str();
            std::cout << "Global setting to send: '" << *globalEnables << "'" << std::endl;
        }

        if (categoryLevels || globalEnables)
        {
            if (sensor.setLogSettings(globalEnables, categoryLevels))
            {
                std::cout << "setLogSettings() succeeded" << std::endl;
            }
            else
            {
                std::cerr << "setLogSettings() FAILED" << std::endl;
            }
        }
        /*
         * Store settings (optional)
         */
        if (storeSettings)
        {
            if (sensor.storeSettings())
            {
                std::cout << "Sensor settings stored in persistent memory." << std::endl;
            }
            else
            {
                std::cerr << "Failed to store sensor settings in persistent memory" << std::endl;
            }
        }

        const auto logSettings = sensor.getLogSettings();

        if (logSettings)
        {
            std::cout << "Log settings reported: '" << logSettings->c_str() << "'" << std::endl;
        }
        else
        {
            std::cerr << "No log settings available" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
