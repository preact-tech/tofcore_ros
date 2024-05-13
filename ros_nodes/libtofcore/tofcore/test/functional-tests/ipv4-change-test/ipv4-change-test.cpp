/**
 * @file ipv4-change-test.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that sets IPv4 address and then communicates to the new address
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


static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc(
                "Test ability to change IPv4 address and then communicate at that new address\n\n"
                "  Usage: [options]\n\n"
                );
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
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

    std::array<std::byte, 4> ipv4Addr { (std::byte)10, (std::byte)10, (std::byte)31, (std::byte)180 };
    const std::array<std::byte, 4> ipv4Mask { (std::byte)255, (std::byte)255, (std::byte)255, (std::byte)0 };
    const std::array<std::byte, 4> ipv4Gway { (std::byte)10, (std::byte)10, (std::byte)31, (std::byte)1 };

    for (int addrOffset = 2; addrOffset >= 0; --addrOffset)
    {
        {
            tofcore::Sensor sensor { devicePort, baudRate };
            ipv4Addr[3] = (std::byte)(180 + addrOffset);
            if (sensor.setIPv4Settings(ipv4Addr, ipv4Mask, ipv4Gway))
            {
                std::cout << "SUCCESS in setting:" << std::endl;
            }
            else
            {
                std::cout << "FAILED in setting:" << std::endl;
            }
            std::cout << "  ipv4Addr: " << (unsigned)ipv4Addr[0] << "." << (unsigned)ipv4Addr[1] << "." << (unsigned)ipv4Addr[2] << "." << (unsigned)ipv4Addr[3] << std::endl;
            std::cout << "  ipv4Mask: " << (unsigned)ipv4Mask[0] << "." << (unsigned)ipv4Mask[1] << "." << (unsigned)ipv4Mask[2] << "." << (unsigned)ipv4Mask[3] << std::endl;
            std::cout << "  ipv4GW:   " << (unsigned)ipv4Gway[0] << "." << (unsigned)ipv4Gway[1] << "." << (unsigned)ipv4Gway[2] << "." << (unsigned)ipv4Gway[3] << std::endl;
        }

        std::this_thread::sleep_for(1s);

        {
            tofcore::Sensor sensor { devicePort, baudRate };
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
        }
    }

    return 0;
}
