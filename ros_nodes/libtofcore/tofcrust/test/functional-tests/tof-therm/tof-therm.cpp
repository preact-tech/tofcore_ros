/**
 * @file tof-therm.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcrust to get/set sensor thermal limits
 */
#include "tofcrust/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using namespace tofcrust;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static double entryDegC { 0.0 };
static bool entryThresholdSet { false };
static double exitDegC { 0.0 };
static bool exitThresholdSet { false };
static volatile bool exitRequested { false };

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Set/Get horizontal/vertical therm state.");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("entry-degc,e", po::value<double>(&entryDegC), "DegC to enter thermal protection mode.")
        ("exit-degc,x", po::value<double>(&exitDegC), "DegC to exit thermal protection mode.")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    if (vm.count("entry-degc"))
    {
        entryThresholdSet = true;
    }
    if (vm.count("exit-degc"))
    {
        exitThresholdSet = true;
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
        tofcrust::Sensor sensor { devicePort, baudRate };
        /*
         * Logic to set thresholds
         */
        if (entryThresholdSet)
        {
            if (!exitThresholdSet)
            {
                exitDegC = entryDegC - 0.5;
            }
        }
        else if (exitThresholdSet)
        {
            entryDegC = exitDegC + 0.5;
        }

        if (entryThresholdSet || exitThresholdSet)
        {
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "FAILED to enable factory mode" << std::endl;
            }

            if (sensor.setThermalProtectionThresholds(std::make_tuple(entryDegC, exitDegC)))
            {
                std::cout << "SET thresholds. Entry: " << entryDegC << " degC; Exit: " << exitDegC << " degC" << std::endl;
            }
            else
            {
                std::cerr << "Unable to SET thermal thresholds" << std::endl;
            }

            if (!sensor.setFactoryMode(false))
            {
                std::cerr << "FAILED to disable factory mode" << std::endl;
            }
        }
        /*
         * Logic to read back thresholds
         */
        auto thermThresholds = sensor.getThermalProtectionThresholds();

        if (thermThresholds)
        {
            entryDegC = std::get<0>(*thermThresholds);
            exitDegC = std::get<1>(*thermThresholds);
            std::cout << "Entry: " << entryDegC << " degC; Exit: " << exitDegC << " degC" << std::endl;
        }
        else
        {
            std::cerr << "Unable to GET thermal thresholds" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
