/**
 * @file tof-ipv4.cpp
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
using namespace std::chrono_literals;
using namespace std::chrono;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static std::vector<unsigned> ipv4Data {};
static std::vector<unsigned> logIpv4 {};
static bool persist { false };


static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc(
                "IPv4 write[optional]/read utility for sensor IPV4 and sensor logging IPV4 data\n\n"
                "  Usage: [options] [-i IPv4 address quartet, IPv4 mask quartet, IPv4 gateway quartet] [-l IPv4 address quartet, IPv4 port]\n\n"
                "  EXAMPLE1: To set the IPv4 address, mask, gateway to 10.10.31.180, 255.255.255.0, 10.10.31.1\n\n"
                "     tof-ipv4 -i 10 10 31 180 255 255 255 0 10 10 31 1\n\n"
                "  EXAMPLE2: To set the IPv4 destination address:port for the UDP log to 10.10.31.100:5001\n\n"
                "     tof-ipv4 -l 10 10 31 100 5001\n\n"
                "  NOTE:\n"
                "    1) If no -i option is specified, the IPv4 values are only read.\n"
                "    2) If no -l option is specified, the IPv4 values for the UDP log output are only read.\n"
                );
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("ipv4Data,i", po::value<std::vector<unsigned>>(&ipv4Data)->multitoken(), "Set IPv4 (requires 12 unsigned values)")
        ("logIpv4,l", po::value<std::vector<unsigned>>(&logIpv4)->multitoken(), "Set UDP log IPv4 (requires 5 unsigned values)")
        ("persist,w", po::value<bool>(&persist)->default_value(false), "Write sensor's settings to persistent memory")
        ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

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
        tofcore::Sensor sensor { devicePort, baudRate };
        // Set the IPv4 values
        if (ipv4Data.size() == 12)
        {
            const std::array<std::byte, 4> ipv4Addr { (std::byte)ipv4Data[0], (std::byte)ipv4Data[1], (std::byte)ipv4Data[2], (std::byte)ipv4Data[3] };
            const std::array<std::byte, 4> ipv4Mask { (std::byte)ipv4Data[4], (std::byte)ipv4Data[5], (std::byte)ipv4Data[6], (std::byte)ipv4Data[7] };
            const std::array<std::byte, 4> ipv4Gway { (std::byte)ipv4Data[8], (std::byte)ipv4Data[9], (std::byte)ipv4Data[10], (std::byte)ipv4Data[11] };
            if (sensor.setIPv4Settings(ipv4Addr, ipv4Mask, ipv4Gway))
            {
                std::cout << "SUCCESS in setting:" << std::endl;
            }
            else
            {
                std::cout << "FAILED in setting:" << std::endl;
            }
            std::cout << "  ipv4Addr: " << ipv4Data[0] << "." << ipv4Data[1] << "." << ipv4Data[2] << "."  << ipv4Data[3] << std::endl;
            std::cout << "  ipv4Mask: " << ipv4Data[4] << "." << ipv4Data[5] << "." << ipv4Data[6] << "."  << ipv4Data[7] << std::endl;
            std::cout << "  ipv4GW:   " << ipv4Data[8] << "." << ipv4Data[9] << "." << ipv4Data[10] << "."  << ipv4Data[11] << std::endl;
        }
        else
        {
            std::cout << "Skipping setting of IPv4 data since 12 values were not provided" << std::endl;
        }
        // Set the IPv4 destination address:port for the UDP log
        if (logIpv4.size() == 5)
        {
            const std::array<std::byte, 4> logIpv4Addr { (std::byte)logIpv4[0], (std::byte)logIpv4[1], (std::byte)logIpv4[2], (std::byte)logIpv4[3] };
            const uint16_t port = logIpv4[4];
            if (sensor.setLogIPv4Destination(logIpv4Addr, port))
            {
                std::cout << "SUCCESS in setting:" << std::endl;
            }
            else
            {
                std::cout << "FAILED in setting:" << std::endl;
            }
            std::cout << "  UDP log destination: " << logIpv4[0] << "." << logIpv4[1] << "." << logIpv4[2]
                      << "."  << logIpv4[3] << ":" << port << std::endl;
        }
        else
        {
            std::cout << "UDP log (" << logIpv4.size() << "): ";
            for (auto v : logIpv4)
            {
                std::cout << " " << (unsigned)v;
            }
            std::cout << "\n";
            std::cout << "Skipping setting of UDP log IPv4 address:port since 5 values were not provided" << std::endl;
        }

        std::this_thread::sleep_for(500ms);

        if (persist)
        {
            if (sensor.storeSettings())
            {
                std::cout << "SUCCESS in updating sensor's persistent memory" << std::endl;
            }
            else
            {
                std::cout << "FAILED to update sensor's persistent memory" << std::endl;
            }
        }

        // Read the IPv4 values
        std::array<std::byte, 4> adrs;
        std::array<std::byte, 4> mask;
        std::array<std::byte, 4> gway;
        auto ipv4ValuesRead = sensor.getIPv4Settings(adrs, mask, gway);

        if (ipv4ValuesRead)
        {
            std::cout << "IPv4 values reported:" << std::endl;
            std::cout << "  ipv4Addr: " << (unsigned)adrs[0] << "."<< (unsigned)adrs[1] << "."<< (unsigned)adrs[2] << "."<< (unsigned)adrs[3] << std::endl;
            std::cout << "  ipv4Mask: " << (unsigned)mask[0] << "."<< (unsigned)mask[1] << "."<< (unsigned)mask[2] << "."<< (unsigned)mask[3] << std::endl;
            std::cout << "  ipv4GW:   " << (unsigned)gway[0] << "."<< (unsigned)gway[1] << "."<< (unsigned)gway[2] << "."<< (unsigned)gway[3] << std::endl;
        }
        else
        {
            std::cout << "FAILED read of IPv4 values" << std::endl;
        }

        auto logIpv4Destination = sensor.getLogIpv4Destination();
        if (logIpv4Destination)
        {
            auto&& [logAddr, logPort] = *logIpv4Destination;
            std::cout << "UDP log destination: " << (unsigned)logAddr[0] << "." << (unsigned)logAddr[1] << "."
                      << (unsigned)logAddr[2] << "." << (unsigned)logAddr[3] << ":" << logPort << std::endl;
        }
        else
        {
            std::cout << "FAILED read of UDP log destination" << std::endl;
        }


    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
