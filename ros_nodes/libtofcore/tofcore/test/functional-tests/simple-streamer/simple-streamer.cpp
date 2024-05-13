/**
 * @file simple-streamer.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to stream DCS or DCS+Ambient data
 */
#include "tofcore/tof_sensor.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <thread>
#include <boost/program_options.hpp>


using namespace std::chrono_literals;
using namespace std::chrono;
using namespace tofcore;
using namespace TofComm;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool captureAmbient { false };
static bool captureAmplitude { false };
static bool captureDcsDiff { false };
static bool captureDistance { false };
static std::string devicePort { DEFAULT_PORT_NAME };
static bool enableBinning { false };
static volatile bool exitRequested { false };
static size_t verbosity { 0 };
static uint16_t modulation { 0 };
static uint16_t integration_time { 0 };
static bool setFramePeriod { false };
static uint32_t framePeriodMs { 1000 };

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

    using DataType = tofcore::Measurement_T::DataType;
    switch (pData->type())
    {
        case DataType::DISTANCE_AMPLITUDE:
            ++amplitudeCount;
            ++distanceCount;
            if (verbosity > 0)
            {
                std::cout << "[" << framePeriod.count() << "] DISTANCE-AMPLITUDE measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
                auto distanceData = pData->distance();
                uint32_t adcErrors { 0 };
                uint32_t amplitudeErrors { 0 };
                uint32_t goodPixels { 0 };
                uint32_t ignoredPixels { 0 };
                uint32_t pixelErrors { 0 };
                uint32_t saturatedPixels { 0 };
                for (auto p = distanceData.begin(); p != distanceData.end(); ++p)
                {
                    auto distance = *p;
                    if ((distance & 0x0001) != 0)
                    {
                        ++pixelErrors;
                        auto errCode = (distance & 0x000E);
                        if (errCode == 0x0002)
                        {
                            ++saturatedPixels;
                        }
                        else if (errCode == 0x0004)
                        {
                            ++ignoredPixels;
                        }
                        else if (errCode == 0x0008)
                        {
                            ++amplitudeErrors;
                        }
                        else
                        {
                            ++adcErrors;
                        }
                    }
                    else
                    {
                        ++goodPixels;
                    }
                }
                std::cout << "\tGood pixels: " << goodPixels << "; Errors: " << pixelErrors
                          << "; amplitude: " << amplitudeErrors << "; saturated: " << saturatedPixels
                          << "; ADC: " << adcErrors << "\n";

            }
            break;
        case DataType::DCS:
            ++dcsCount;
            if (verbosity > 0)
            {
                std::cout << "[" << framePeriod.count() << "] DCS measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::DISTANCE:
            ++distanceCount;
            if (verbosity > 0)
            {
                std::cout << "[" << framePeriod.count() << "] DISTANCE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::AMPLITUDE:
            ++amplitudeCount;
            if (verbosity > 0)
            {
                std::cout << "[" << framePeriod.count() << "] AMPLITUDE measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        case DataType::AMBIENT:
            ++ambientCount;
            if (verbosity > 0)
            {
                std::cout << "[" << framePeriod.count() << "] AMBIENT measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        case DataType::DCS_DIFF_AMBIENT:
            ++ambientCount;
            ++dcsDiffCount;
            if (verbosity > 0)
            {
                std::cout << "[" << framePeriod.count() << "] DCS_DIFF+AMBIENT measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;

        default:
            std::cout << "[" << framePeriod.count() << "] UNRECOGNIZED data type: " << static_cast<int16_t>(pData->type()) << std::endl;
    }
    if(verbosity > 0)
    {
        auto chip_temps = pData->sensor_temperatures();
        if(chip_temps) 
        {
            std::cout << "  Sensor temperatures: " << (*chip_temps)[0] << ", " << (*chip_temps)[1] << ", "<< (*chip_temps)[2] << ", "<< (*chip_temps)[3] << std::endl;
        } 
        else 
        {
            std::cout << "  No sensor temperature data" << std::endl;
        }
        auto integration_time = pData->integration_time();
        if(integration_time)
        {
            std::cout << "  Integration time setting (uS): " << *integration_time << std::endl;
        }
        else 
        {
            std::cout << "  No integration time data" << std::endl;
        }
        auto mod_frequency = pData->modulation_frequency();
        if(mod_frequency)
        {
            std::cout << "  Modulation Frequency setting (Hz): " << *mod_frequency << std::endl;
        }
        else 
        {
            std::cout << "  No modulation frequency data" << std::endl;
        }
        auto v_binning = pData->vertical_binning();
        auto h_binning = pData->horizontal_binning();
        if(v_binning && h_binning)
        {
            std::cout << "  Binning settings: " << (int)(*h_binning) << " " << (int)(*v_binning) << std::endl;
        }
        else 
        {
            std::cout << "  No binning data" << std::endl;
        }

        auto dll_settings = pData->dll_settings();
        if(dll_settings)
        {
            std::cout << "  DLL settings: " << ((*dll_settings)[0] != 0 ? "True " : "False ") << (int)(*dll_settings)[1] << " " <<  (int)(*dll_settings)[2] << " " <<  (int)(*dll_settings)[3] << std::endl;
        }
        else 
        {
            std::cout << "  No DLL settings" << std::endl;
        }
        auto illum = pData->illuminator_info();
        if(illum)
        {
            const auto& illum_info = *illum;
            std::cout << "  Illuminator info: 0x" << std::hex << (int)illum_info.led_segments_enabled << std::dec << " "
                      << illum_info.temperature_c << "C " 
                      <<  illum_info.vled_v << "V " 
                      << illum_info.photodiode_v << "V" 
                      << std::endl;
        }
        else
        {
            std::cout << "  No Illuminator information" << std::endl;
        }

        auto vsmControl = pData->vsm_info();
        if(vsmControl)
        {
            std::cout << "  VSM: Flags=" << vsmControl->m_vsmFlags << "; N = "
                      << (unsigned)vsmControl->m_numberOfElements  << "; I = "
                      << (unsigned)vsmControl->m_vsmIndex << ";";
            uint8_t numElements = std::min(vsmControl->m_numberOfElements, (uint8_t) VSM_MAX_NUMBER_OF_ELEMENTS);
            for (decltype(numElements) n = 0; n < numElements; ++n)
            {
                VsmElement_T& element = vsmControl->m_elements[n];
                std::cout << " [" << element.m_integrationTimeUs << ", " << element.m_modulationFreqKhz << "]";
            }
            std::cout << std::endl;
        }
        else
        {
            std::cout << "  No VSM data" << std::endl;
        }

        auto timestamp = pData->frame_timestamp();
        if(timestamp)
        {
            std::cout << "  Frame timestamp: " << (int) *timestamp << std::endl;
        }
        else
        {
            std::cout << "No timestamp found in frame data" << std::endl;
        }

        std::cout << "\n\n";
    }
}

namespace po = boost::program_options;

class CountValue : public po::typed_value<std::size_t>
{
public:
    CountValue():
        CountValue(nullptr)
    {
    }

    CountValue(std::size_t* store):
        po::typed_value<std::size_t>(store)
    {
        // Ensure that no tokens may be passed as a value.
        default_value(0);
        zero_tokens();
    }

private:

    virtual void xparse(boost::any& store, const std::vector<std::string>& /*tokens*/) const
    {
        // Replace the stored value with the access count.
        store = boost::any(++count_);
    }

    mutable std::size_t count_{ 0 };
};

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Simple Streamer Test");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("Binning,B", po::bool_switch(&enableBinning)->default_value(false),"Enable full binning")
        ("amplitude,a", po::bool_switch(&captureAmplitude), "Capture DCS+Ambient or Distance+Amplitude frames, (not just DCS or Distance)")
        ("ambient,A", po::bool_switch(&captureAmbient), "Capture DCS+Ambient or Distance+Amplitude frames, (not just DCS or Distance)")
        ("distance,d", po::bool_switch(&captureDistance),  "Capture distance (or amplitude) frames instead of DCS frames")
        ("Dcs-diff,D", po::bool_switch(&captureDcsDiff),  "Capture DCS_DIFF + Ambient frames")
        ("modulation,m", po::value<uint16_t>(&modulation)->default_value(0),"Set modulation frequency to this value (kHz)")
        ("integration,i", po::value<uint16_t>(&integration_time)->default_value(0),"Set integration time to this value (uS)")
        ("frame-period,f", po::value<uint32_t>(&framePeriodMs),"Set target frame period (mS)")
        ("verbose,V",               
           new  CountValue(&verbosity),
            "Increase verbosity of output")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("frame-period"))
    {
        setFramePeriod = true;
    }
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
    try
    {
        parseArgs(argc, argv);
    }
    catch (po::error &x)
    {
        std::cerr << x.what() << std::endl;
        return 1;
    }
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

        if (enableBinning)
        {
            sensor.setBinning(true);
        }
        else
        {
            sensor.setBinning(false);
        }
        if(modulation)
        {
            sensor.setModulation(modulation);
        }

        if(integration_time)
        {
            sensor.setIntegrationTime(integration_time);
        }

        if (setFramePeriod)
        {
            if (!sensor.setFramePeriodMs(framePeriodMs))
            {
                std::cerr << "FAILED to set frame period to " << framePeriodMs << " mS\n";
            }
        }

        auto framePeriodData = sensor.getFramePeriodMsAndLimits();
        if (framePeriodData)
        {
            auto [framePeriodMs, framePeriodMsMin, framePeriodMsMax] = *framePeriodData;
            std::cout << "Frame Period: " << framePeriodMs << " mS; Min: " << framePeriodMsMin
                      << " mS; Max: " << framePeriodMsMax << " mS\n";
        }
        else
        {
            std::cerr << "FAILED to get frame period data\n";
        }

        sensor.subscribeMeasurement(&measurement_callback); // callback is called from background thread
        if (captureDcsDiff)
        {
            sensor.streamDCSDiffAmbient();
        }
        else if (captureDistance)
        {
            if (captureAmbient || captureAmplitude)
            {
                sensor.streamDistanceAmplitude();
            }
            else
            {
                sensor.streamDistance();
            }
        }
        else
        {
            if (captureAmbient || captureAmplitude)
            {
                sensor.streamDCSAmbient();
            }
            else
            {
                sensor.streamDCS();
            }
        }
        auto lastTime = steady_clock::now();
        uint32_t outputCount = 1;
        while (!exitRequested) // wait for ^\ or ^C
        {
            std::this_thread::sleep_until(lastTime + 1000ms);
            lastTime = steady_clock::now();
            const uint32_t ambient { ambientCount };
            ambientCount = 0;
            const uint32_t amplitude { amplitudeCount };
            amplitudeCount = 0;
            const uint32_t dcs { 4 * dcsCount };
            dcsCount = 0;
            const uint32_t dcsDiff { 2 * dcsDiffCount };
            dcsDiffCount = 0;
            const uint32_t distance { distanceCount };
            distanceCount = 0;
            std::cout << "[" << std::setw(5) << std::setfill('0') << outputCount
                      <<"] FPS: ambient = " << std::setw(2) << ambient
                      << "; amplitude = " << std::setw(2) << amplitude
                      << "; dcs = " << std::setw(2) << dcs
                      << "; dcsDiff = " << std::setw(2) << dcsDiff
                      << "; distance = " << std::setw(2) << distance
                      << "; total = " << std::setw(3) << (ambient + amplitude + dcs + dcsDiff + distance) << std::endl;
            ++outputCount;
        }
        std::cout << "Shutting down..." << std::endl;
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
