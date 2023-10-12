/**
 * @file set-lens-info.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that libtofcrust to set Lens information.
 */
#include "tofcrust/tof_sensor.hpp"
#include <array>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace tofcrust;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { 1 };
static bool resetToDefault { false };

static bool setLensInfo { false };
static double rowOffset { -0.5 };
static double columnOffset { -0.5 };
static double rowFocalLength { 247.041 };
static double columnFocalLength { 247.041 };
static std::array<double, 5> undistortionCoeffs { 0.206392, 0.131967, 0, 0, -0.0755417 };

static void parseArgs(int argc, char *argv[])
{
    std::vector<double> parameters {};

    namespace po = boost::program_options;
    po::options_description desc(
                "Lens info write[optional]/read utility\n\n"
                "Usage: [options] [-i rowOffset columnOffset rowFocalLength columnFocalLength undistortionCoeffs0 ... undistortionCoeffs4]\n\n"
                "  EXAMPLE:  set-lens-info -i 0.5 -i -0.5 -i 242.123 -i 245.678 -i 0.234 -i -0.455 -i 0.789 -i -0.02468 -i 0.3579\n\n"
                "  NOTE:\n"
                "    1) If no -i option is specified, the Lens Info is only read.\n"
                "    2) Multiple -i s are required when negative values are present due to a bug in the boost logic.\n"
    );
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("reset-to-default,r",  po::bool_switch(&resetToDefault), "Reset Lens Info to default values")
        ("parameters,i", po::value<std::vector<double>>(&parameters),
            "Set Lens Info, requires 9 doubles: rowOffset columnOffset rowFocalLength columnFocalLength undistortionCoeffs0 ... undistortionCoeffs4")
        ;

    po::positional_options_description pos_desc;
    pos_desc.add("parameters", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);

    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    if (!resetToDefault)
    {
        if (parameters.size() != 9 )
        {
            std::cerr << "ERROR: All 9 lens info parameters must be supplied" << std::endl;
            exit(-1);
        } else 
        {
            rowOffset = parameters[0];
            columnOffset = parameters[1];
            rowFocalLength = parameters[2];
            columnFocalLength = parameters[3];
            undistortionCoeffs[0] = parameters[4];
            undistortionCoeffs[1] = parameters[5];
            undistortionCoeffs[2] = parameters[6];
            undistortionCoeffs[3] = parameters[7];
            undistortionCoeffs[4] = parameters[8];
            setLensInfo = true;
        }
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

        if (resetToDefault)
        {
            rowOffset = columnOffset = rowFocalLength = columnFocalLength = 0.0;
            for (size_t n = 0; n < 5; ++n)
            {
                undistortionCoeffs[n] = 0.0;
            }
            std::cout << "Resetting Lens Info to default values" << std::endl;
            if (sensor.setLensInfo(rowOffset, columnOffset, rowFocalLength, columnFocalLength, undistortionCoeffs))
            {
                std::cout << "SUCCESS" << std::endl;
            }
            else
            {
                std::cerr << "FAILED to reset Lens info to defaults" << std::endl;
            }
        }
        else if (setLensInfo)
        {
            std::cout << "Setting Lens info: rowOffset="           << rowOffset
                      << ", columnOffset="      << columnOffset
                      << ", rowFocalLength="    << rowFocalLength
                      << ", columnFocalLength=" << columnFocalLength
                      << ", undistortionCoeff=[";
            size_t n;
            for (n = 0; n < 4; ++n)
            {
                std::cout << undistortionCoeffs[n] << ", ";
            }
            std::cout << undistortionCoeffs[n] << "]" << std::endl;
            if (sensor.setLensInfo(rowOffset, columnOffset, rowFocalLength, columnFocalLength, undistortionCoeffs))
            {
                std::cout << "SUCCESS" << std::endl;
            }
            else
            {
                std::cerr << "FAILED to set Lens info" << std::endl;
            }
        }
        else
        {
            std::cout << "Skipping setting of Lens info" << std::endl;
        }

        auto lensInfo = sensor.getLensIntrinsics();
        if (lensInfo)
        {
            std::cout << "rowOffset="           << lensInfo->m_rowOffset
                      << ", columnOffset="      << lensInfo->m_columnOffset
                      << ", rowFocalLength="    << lensInfo->m_rowFocalLength
                      << ", columnFocalLength=" << lensInfo->m_columnFocalLength
                      << ", undistortionCoeff=[";
            size_t n;
            for (n = 0; n < 4; ++n)
            {
                std::cout << lensInfo->m_undistortionCoeffs[n] << ", ";
            }
            std::cout << lensInfo->m_undistortionCoeffs[n] << "]" << std::endl;
        }
        else
        {
            std::cerr << "Unable to read Lens Info data" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
