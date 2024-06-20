/**
 * @file stream-before-set-vsm.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program for changing VSM after starting streaming.
 */
#include "tofcore/tof_sensor.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>
#include <thread>

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace TofComm;
using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static uint32_t delayAfterStart { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool startStreamingOnce { false };

static std::atomic<uint32_t> ambientCount;
static std::atomic<uint32_t> amplitudeCount;
static std::atomic<uint32_t> dcsCount;
static std::atomic<uint32_t> dcsDiffCount;
static std::atomic<uint32_t> distanceCount;


static void measurement_callback(std::shared_ptr<tofcore::Measurement_T> pData)
{
    static high_resolution_clock::time_point lastTime { high_resolution_clock::now() };
    high_resolution_clock::time_point timeNow = high_resolution_clock::now();

    duration<double> framePeriod { duration_cast<duration<double>>(timeNow - lastTime) };
    lastTime = timeNow;

    unsigned integration_time { 0 };

    auto optIntTime = pData->integration_time();
    if (optIntTime)
    {
        integration_time = *optIntTime;
    }

    using DataType = tofcore::Measurement_T::DataType;
    switch (pData->type())
    {
        case DataType::DISTANCE_AMPLITUDE:
            ++amplitudeCount;
            ++distanceCount;
            std::cout << "[" << framePeriod.count() << "] DISTANCE-AMPLITUDE measurement data, packet size "
                      << (pData->pixel_buffer().size()) << "; integration time = " << integration_time << " uS" << std::endl;
            break;
        case DataType::DCS:
            ++dcsCount;
            std::cout << "[" << framePeriod.count() << "] DCS measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            break;
        case DataType::DISTANCE:
            ++distanceCount;
            std::cout << "[" << framePeriod.count() << "] DISTANCE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            break;
        case DataType::AMPLITUDE:
            ++amplitudeCount;
            std::cout << "[" << framePeriod.count() << "] AMPLITUDE measurement data, packet size "
                        << (pData->pixel_buffer().size()) << std::endl;
            break;
        case DataType::AMBIENT:
            ++ambientCount;
            std::cout << "[" << framePeriod.count() << "] AMBIENT measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            break;
        case DataType::DCS_DIFF_AMBIENT:
            ++ambientCount;
            ++dcsDiffCount;
            std::cout << "[" << framePeriod.count() << "] DCS_DIFF+AMBIENT measurement data, packet size "
                      << (pData->pixel_buffer().size()) << std::endl;
            break;

        default:
            std::cout << "[" << framePeriod.count() << "] UNRECOGNIZED data type: " << static_cast<int16_t>(pData->type()) << std::endl;
    }
}

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Get Lens Information Test");
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("delay,D", po::value<uint32_t>(&delayAfterStart)->default_value(0), "Delay (mS) after start streaming")
        ("help,h", "produce help message")
        ("once,o", po::value<bool>(&startStreamingOnce)->default_value(false), "Only start streaming once")
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
        tofcore::Sensor sensor { devicePort, baudRate };

        VsmControl_T vsmControl { };
        vsmControl.m_numberOfElements = 2;
        vsmControl.m_elements[0].m_integrationTimeUs = 100;
        vsmControl.m_elements[1].m_integrationTimeUs = 1000;

        sensor.subscribeMeasurement(&measurement_callback);

        while (!exitRequested)
        {
            static bool startedStreaming { false };

            if (!startStreamingOnce || !startedStreaming)
            {
                if (!sensor.streamDCSAmbient())
                {
                    std::cerr << "\nstreamDCSAmbient() FAILED\n";
                    break;
                }
                if (startStreamingOnce)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                startedStreaming = true;
            }
            if (delayAfterStart > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(delayAfterStart));
            }
            if (!sensor.setVsm(vsmControl))
            {
                std::cerr << "\nsetVsm() FAILED\n";
                break;
            }
            std::cout << "\nVSM set\n\n";
            if (!startStreamingOnce)
            {
                sensor.stopStream();
            }
       }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
