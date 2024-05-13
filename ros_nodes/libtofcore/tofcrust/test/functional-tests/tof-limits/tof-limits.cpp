/**
 * @file tof-limits.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcrust to set sensor limits
 */
#include "tofcrust/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using namespace tofcrust;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };

static uint32_t framePeriodMsMin { 0 };
static bool framePeriodMinSet { false };
static uint32_t framePeriodMsMax { 0 };
static bool framePeriodMaxSet { false };

static uint16_t integTimeUsMin { 0 };
static bool integTimeMinSet { false };
static uint16_t integTimeUsMax { 0 };
static bool integTimeMaxSet { false };

static uint16_t minAmplitudeMin { 0 };
static bool minAmplitudeMinSet { false };
static uint16_t minAmplitudeMax { 0 };
static bool minAmplitudeMaxSet { false };

static uint16_t modFreqKhzMin { 0 };
static bool modFreqMinSet { false };
static uint16_t modFreqKhzMax { 0 };
static bool modFreqMaxSet { false };
static uint16_t modFreqKhzStep { tofcore::DEFAULT_MOD_FREQ_STEP_KHZ };
static bool modFreqStepSet { false };


static volatile bool exitRequested { false };
static bool skipFactoryMode { false };
static bool writeToPersistentMemory { false };

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Set/Get sensor limits.");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("min-amplitude-min,a", po::value<uint16_t>(&minAmplitudeMin), "Minimum minimum amplitude setting.")
        ("min-amplitude-max,A", po::value<uint16_t>(&minAmplitudeMax), "Maximum minimum amplitude setting.")
        ("mod-freq-khz-min,f", po::value<uint16_t>(&modFreqKhzMin), "Minimum modulation frequency in kHz.")
        ("mod-freq-khz-max,F", po::value<uint16_t>(&modFreqKhzMax), "Maximum modulation frequency in kHz.")
        ("mod-freq-khz-step,S", po::value<uint16_t>(&modFreqKhzStep), "Modulation frequency step size in kHz.")
        ("integ-tim-us-min,i", po::value<uint16_t>(&integTimeUsMin), "Minimum integration time in uS.")
        ("integ-tim-us-max,I", po::value<uint16_t>(&integTimeUsMax), "Maximum integration time in uS.")
        ("frame-period-ms-min,m", po::value<uint32_t>(&framePeriodMsMin), "Minimum frame period in mS.")
        ("frame-period-ms-max,M", po::value<uint32_t>(&framePeriodMsMax), "Maximum frame period in mS.")
        ("skip-factory-mode,s", po::value<bool>(&skipFactoryMode), "Skip factory mode")
        ("write-to-persistent-memory,w", po::value<bool>(&writeToPersistentMemory), "Write limits to persistent memory")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    framePeriodMinSet = (vm.count("frame-period-ms-min") != 0);
    framePeriodMaxSet = (vm.count("frame-period-ms-max") != 0);
    integTimeMinSet = (vm.count("integ-tim-us-min") != 0);
    integTimeMaxSet = (vm.count("integ-tim-us-max") != 0);
    minAmplitudeMinSet = (vm.count("min-amplitude-min") != 0);
    minAmplitudeMaxSet = (vm.count("min-amplitude-max") != 0);
    modFreqMinSet = (vm.count("mod-freq-khz-min") != 0);
    modFreqMaxSet = (vm.count("mod-freq-khz-max") != 0);
    modFreqStepSet = (vm.count("mod-freq-khz-step") != 0);
}

static void signalHandler(int signum)
{
    (void)signum;
    exitRequested = true;
}

static void setFramePeriodLimits(tofcrust::Sensor& sensor)
{
    if ((framePeriodMinSet && !framePeriodMaxSet) || (!framePeriodMinSet && framePeriodMaxSet))
    {
        std::cout << "\nERROR: You must specify BOTH the min. and max. frame period in order to change either of them.\n";
        exit(-1);
    }
    else if (framePeriodMinSet || framePeriodMaxSet)
    {
        if (!skipFactoryMode)
        {
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "FAILED to enable factory mode\n";
            }
        }
        if (sensor.setFramePeriodMsLimits(std::make_tuple(framePeriodMsMin, framePeriodMsMax)))
        {
            std::cout << "SET frame period limits. MIN: " << framePeriodMsMin << " mS; MAX: " << framePeriodMsMax << " mS\n";
        }
        else
        {
            std::cerr << "FAILED to SET frame period limits.\n";
        }
        if (!sensor.setFactoryMode(false))
        {
            std::cerr << "FAILED to disable factory mode\n";
        }
    }
}

static void setIntegrationTimeLimits(tofcrust::Sensor& sensor)
{
    if ((integTimeMinSet && !integTimeMaxSet) || (!integTimeMinSet && integTimeMaxSet))
    {
        std::cout << "\nERROR: You must specify BOTH the min. and max. integration time in order to change either of them.\n";
        exit(-1);
    }
    else if (integTimeMinSet || integTimeMaxSet)
    {
        if (!skipFactoryMode)
        {
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "FAILED to enable factory mode\n";
            }
        }
        if (sensor.setIntegrationTimeUsLimits(std::make_tuple(integTimeUsMin, integTimeUsMax)))
        {
            std::cout << "SET integration time limits. MIN: " << integTimeUsMin << " uS; MAX: " << integTimeUsMax << " uS\n";
        }
        else
        {
            std::cerr << "FAILED to SET integration time limits.\n";
        }
        if (!sensor.setFactoryMode(false))
        {
            std::cerr << "FAILED to disable factory mode\n";
        }
    }
}

static void setMinAmplitudeLimits(tofcrust::Sensor& sensor)
{
    if ((minAmplitudeMinSet && !minAmplitudeMaxSet) || (!minAmplitudeMinSet && minAmplitudeMaxSet))
    {
        std::cout << "\nERROR: You must specify BOTH the min. and max. minimum amplitude in order to change either of them.\n";
        exit(-1);
    }
    else if (minAmplitudeMinSet || minAmplitudeMaxSet)
    {
        if (!skipFactoryMode)
        {
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "FAILED to enable factory mode\n";
            }
        }
        if (sensor.setMinAmplitudeLimits(std::make_tuple(minAmplitudeMin, minAmplitudeMax)))
        {
            std::cout << "SET minimum amplitude limits. MIN: " << minAmplitudeMin << "; MAX: " << minAmplitudeMax << "\n";
        }
        else
        {
            std::cerr << "FAILED to SET minimum amplitude limits.\n";
        }
        if (!sensor.setFactoryMode(false))
        {
            std::cerr << "FAILED to disable factory mode\n";
        }
    }
}

static void setModulationFrequencyLimits(tofcrust::Sensor& sensor)
{
    // Must specify both min and max
    if ((modFreqMinSet && !modFreqMaxSet) || (!modFreqMinSet && modFreqMaxSet))
    {
        std::cout << "\nERROR: You must specify BOTH the min. and max. modulation frequency in order to change either of them.\n";
        exit(-1);
    }
    // If either frequency parameter is zero, reject
    else if (  (modFreqMinSet  && (modFreqKhzMin == 0) ) ||
               (modFreqMaxSet  && (modFreqKhzMax == 0) ) ||
               (modFreqStepSet && (modFreqKhzStep == 0))   )
    {
        std::cout << "\nERROR: Min, max, and step frequency must be non-zero.\n";
        exit(-1);
    }
    else if (modFreqMinSet && modFreqMaxSet)
    {
        if (!skipFactoryMode)
        {
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "FAILED to enable factory mode\n";
            }
        }
        if (sensor.setModulationFreqKhzLimitsAndStep(std::make_tuple(modFreqKhzMin, modFreqKhzMax, modFreqKhzStep)))
        {
            std::cout << "SET modulation frequency limits. MIN: " << modFreqKhzMin
                      << " kHz; MAX: " << modFreqKhzMax << " kHz; STEP: " << modFreqKhzStep << " kHz\n";
        }
        else
        {
            std::cerr << "FAILED to SET integration time limits.\n";
        }
        if (!sensor.setFactoryMode(false))
        {
            std::cerr << "FAILED to disable factory mode\n";
        }
    }
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

        setFramePeriodLimits(sensor);

        setIntegrationTimeLimits(sensor);

        setModulationFrequencyLimits(sensor);

        setMinAmplitudeLimits(sensor);

        if (writeToPersistentMemory && !sensor.storeSettings())
        {
                std::cerr << "FAILED to save settings\n";
        }
        /*
         * Logic to read back thresholds
         */
        auto framePeriodMsLimits = sensor.getFramePeriodMsAndLimits();
        if (framePeriodMsLimits)
        {
            auto [current, minMs, maxMs] = *framePeriodMsLimits;
            std::cout << "Frame Period\n\tcurrent:\t" << current
                      << " mS\n\tminimum:\t" << minMs
                      << " mS\n\tmaximum:\t" << maxMs
                      << " mS\n";
        }
        else
        {
            std::cerr << "FAILED to GET frame period limits\n";
        }

        auto integTimeUsLimits = sensor.getIntegrationTimeUsAndLimits();
        if (integTimeUsLimits)
        {
            auto [current, minUs, maxUs] = *integTimeUsLimits;
            std::cout << "Integration Time\n\tcurrent:\t" << current
                      << " uS\n\tminimum:\t" << minUs
                      << " uS\n\tmaximum:\t" << maxUs
                      << " uS\n";
        }
        else
        {
            std::cerr << "FAILED to GET frame period limits\n";
        }

        auto modFreqKhzLimits = sensor.getModulationFreqKhzAndLimitsAndStepSize();
        if (modFreqKhzLimits)
        {
            auto [current, minKhz, maxKhz, stepKhz] = *modFreqKhzLimits;
            std::cout << "Modulation Frequency\n\tcurrent:\t" << current
                      << " kHz\n\tminimum:\t" << minKhz
                      << " kHz\n\tmaximum:\t" << maxKhz
                      << " kHz\n\tstep size:\t" << stepKhz
                      << " kHz\n";
        }
        else
        {
            std::cerr << "FAILED to GET frame period limits\n";
        }

        auto vsmLimits = sensor.getVsmMaxNumberOfElements();
        if (vsmLimits)
        {
            std::cout << "VSM\n\tMaximum # of elements:\t" << *vsmLimits << "\n";
        }
        else
        {
            std::cerr << "FAILED to GET maximum number of VSM elements\n";
        }

        auto minAmplitudeLimits = sensor.getMinAmplitudeAndLimits();
        if (minAmplitudeLimits)
        {
            auto [current, minUs, maxUs] = *minAmplitudeLimits;
            std::cout << "Minimum Amplitude\n\tcurrent:\t" << current
                      << "\n\tminimum:\t" << minUs
                      << "\n\tmaximum:\t" << maxUs
                      << "\n";
        }
        else
        {
            std::cerr << "FAILED to GET minimum amplitude limits\n";
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
