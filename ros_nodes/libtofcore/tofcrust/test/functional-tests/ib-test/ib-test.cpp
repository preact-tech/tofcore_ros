/**
 * @file ib-test.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libt10 to test the illuminator board.
 */
#include "tofcrust/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>

using namespace tofcrust;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool runRgbTest { false };
static int16_t rgbColor { -1 };
static uint16_t rgbBlinkMs { 0 };
static int16_t emitterEnable {-1};

//illuminator board stats
static bool setSerial{false};
static std::string serialNumber;
static uint16_t vled {0};
static uint16_t psu_5v { 0 };
static uint16_t pdMv { 0 };
static int32_t temp { 0 };

static std::vector<uint16_t> vledLimits;

constexpr uint8_t TEST_VAL_8BIT {0xBB};
constexpr uint16_t TEST_VAL_16BIT {0xAACC};

/*********************************************
        RGB bitmask bit positions
*********************************************/
#define RGB_OFF_BP     0
#define RGB_RED_BP     1
#define RGB_GREEN_BP   2
#define RGB_YELLOW_BP  3 
#define RGB_BLUE_BP    4
#define RGB_MAGENTA_BP 5 
#define RGB_CYAN_BP    6 
#define RGB_WHITE_BP   7 // Issues with Illuminator board producing white 

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("illuminator board test");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("color,c", po::value<int16_t>(&rgbColor)->default_value(rgbColor), "Set RBG color of status LED")
        ("enable,l", po::value<int16_t>(&emitterEnable)->default_value(emitterEnable), "Enable emitters")
        ("blink,m", po::value<uint16_t>(&rgbBlinkMs)->default_value(rgbBlinkMs), "Set the blink rate of the status LED")
        ("test,r", po::bool_switch(&runRgbTest),"Run RGB test")
        ("serialNumber,s", po::value(&serialNumber), "Set serial number on the Illuminator board")
        ("vled,v", po::value(&vled)->default_value(vled), "Set the VLED volatage setpoint")
        ("vled-limits,L", po::value<std::vector<uint16_t>>(&vledLimits)->multitoken(), "Set the min & max VLED allowed by the sensor.")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << "\n" << desc
                  << "EXAMPLE - Set VLED min/max limits to 5000 & 6000 mV\n\n"
                  << "  ib-test -L 5000 6000\n\n";
        exit(0);
    }

    //If serialNumber option was provided then set setSerial to true so that the serial number gets written.
    setSerial = vm.count("serialNumber") != 0;
}

static void signalHandler(int signum)
{
    (void)signum;
    /*
    Put shutdown code here.
    */
    exit(0);
}

/*
Convenience functions to check the return values.
*/
static std::string checkError(bool pass, uint16_t& buf)
{
    std::string ret {};
    if(pass){
        ret = std::to_string(buf); 
    }
    else {
        ret = "ERROR";
    }
    return ret;
}

static std::string checkError(bool pass, uint8_t& buf)
{
    std::string ret {};
    if(pass){
        ret = std::to_string(buf); 
    }
    else {
        ret = "ERROR";
    }
    return ret;
}

void dumpStats(Sensor& sensor)
{
    TofComm::versionData_t versionData;
    bool ok = sensor.getSensorInfo(versionData);
    
    std::cout << "Firmware version: ";
    if(ok) {
        std::cout << versionData.m_illuminatorSwVersion << '.' << versionData.m_illuminatorSwSourceId << std::endl;
    }
    else
    {
        std::cout << "ERROR" << std::endl;
    }

    std::cout << "Board Config: ";
    if(ok) {
        std::cout << (unsigned) versionData.m_illuminatorHwCfg << std::endl;
    }
    else {
        std::cout << "ERROR" <<std::endl;
    }

    std::cout << "IB Serial: ";
    TofComm::versionData_t sensorInfo;
    if(sensor.getSensorInfo(sensorInfo))
    {
        std::cout << sensorInfo.m_illuminatorBoardSerialNumber << std::endl;
    }
    else
    {
        std::cout << "ERROR" << std::endl;
    }

    std::cout << std::endl;

    std::cout << "Temperature: ";
    if(sensor.getIllmnTemperature(temp))
    {
        float tempDegC = temp/1000.0;
        std::cout << std::to_string(tempDegC) << std::endl;
    }
    else
    {
        std::cout << "ERROR" << std::endl;
    }
    std::cout << std::endl;

    /********************************************************************
                    Read VLED settings and limits
    *********************************************************************/
    std::optional<std::tuple<uint16_t, uint16_t, uint16_t>> vledSettings = sensor.getVledSettingAndLimits();
    if(vledSettings.has_value()) 
    {
        std::cout << "Illuminator Board Limits and setttings (mV)" << std::endl;
        std::cout << "VLED Setting (mV requested or stored in NVM, see below for ADC reading): " << std::get<0>(vledSettings.value()) << std::endl;
        std::cout << "MIN Allowable VLED: " << std::get<1>(vledSettings.value()) << std::endl;
        std::cout << "MAX Allowable VLED: " << std::get<2>(vledSettings.value()) << std::endl;
    }
    else
    {
        std::cout << "Failed to read Illuminator Board Limits!" << std::endl;
    }
    std::cout <<std::endl;

    /********************************************************************
                    Read actual board voltages (via ADC)
    *********************************************************************/
    std::cout << "Illuminator Board Voltages (mV)" << std::endl;
    std::cout << "VLED ADC reading: " << checkError(sensor.getVled(vled), vled) << std::endl;
    std::cout << "5V: " << checkError(sensor.getIb5V(psu_5v), psu_5v) << std::endl;
    std::cout << "V PD: " << checkError(sensor.getIbPd(pdMv), pdMv) << std::endl;
    std::cout << std::endl;
}

void testRgbs(Sensor& sensor)
{
    std::cout << "Starting RGB test. (Note: colors may vary between LEDs)" << std::endl << std::endl;

    uint8_t rgbState { 0 };
    std::cout << "Turning RGBs OFF (state 0)" << std::endl;
    sensor.setIbRgb(0, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color RED (state 1)" << std::endl;
    sensor.setIbRgb(RGB_RED_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color GREEN (state 2)" << std::endl;
    sensor.setIbRgb(RGB_GREEN_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color YELLOW (state 3)" << std::endl;
    sensor.setIbRgb(RGB_YELLOW_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color BLUE (state 4)" << std::endl;
    sensor.setIbRgb(RGB_BLUE_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color MAGENTA (state 5)" << std::endl;
    sensor.setIbRgb(RGB_MAGENTA_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color CYAN (state 6)" << std::endl;
    sensor.setIbRgb(RGB_CYAN_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color WHITE (state 7). Note: due to Illuminator processor limitation on driving R, G, and B may not be white" << std::endl;
    sensor.setIbRgb(RGB_WHITE_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << std::endl << "Finished RGB test." << std::endl;
}

bool setRgb(Sensor& sensor, uint8_t state) 
{
    //Make sure we only set one rgb at a time.
    if(state == 0)
    {
        return sensor.setIbRgb(RGB_OFF_BP, rgbBlinkMs);
    }
    else if(state & RGB_RED_BP)
    {
        return sensor.setIbRgb(RGB_RED_BP, rgbBlinkMs);
    }
    else if(state & RGB_GREEN_BP)
    {
        return sensor.setIbRgb(RGB_GREEN_BP, rgbBlinkMs);
    }
    else if(state & RGB_YELLOW_BP)
    {
        return sensor.setIbRgb(RGB_YELLOW_BP, rgbBlinkMs);
    }
    else if(state & RGB_BLUE_BP)
    {
        return sensor.setIbRgb(RGB_BLUE_BP, rgbBlinkMs);
    }
    else if(state & RGB_MAGENTA_BP)
    {
        return sensor.setIbRgb(RGB_MAGENTA_BP, rgbBlinkMs);
    }
    else if(state & RGB_CYAN_BP)
    {
        return sensor.setIbRgb(RGB_CYAN_BP, rgbBlinkMs);
    }
    else if(state & RGB_WHITE_BP)
    {
        return sensor.setIbRgb(RGB_WHITE_BP, rgbBlinkMs);
    }

    return false;
}

/**
 * @brief write and retrieve 8 and 16 bit values from the sensor
 * 
 * @param sensor reference to the sensor under test.
 * @return true comms ok
 * @return false comm failure
 */
bool testCommsI2C(Sensor& sensor)
{
    bool ok {true};
    std::cout << std::hex;
    std::cout << "Sending test val: " << static_cast<unsigned int>(TEST_VAL_8BIT) << std::endl;
    sensor.setTestVal(TEST_VAL_8BIT);
    uint8_t test8 {0};
    std::cout << "Received test val: ";
    if(sensor.getTestVal(test8))
    {
        std::cout << (unsigned int) test8;
    }
    else
    {
        std::cout << "ERROR";
    } 
    std::cout << std::endl;
    ok &= (test8 == TEST_VAL_8BIT);

    std::cout << "Sending test val: " << static_cast<unsigned int>(TEST_VAL_16BIT) << std::endl;
    sensor.setTestVal(TEST_VAL_16BIT);
    uint16_t test16 {0};
    std::cout << "Received test val: ";
    if(sensor.getTestVal(test16))
    {
        std::cout << (unsigned int) test16;
    } 
    else
    {
        std::cout << "ERROR";
    }

    std::cout << std::endl;
    std::cout << std::dec;
    ok &= (test16 == TEST_VAL_16BIT);

    return ok;
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
        Sensor sensor { devicePort, baudRate };

        //Exit here if we're just setting the RGB.
        if(rgbColor >= 0)
        {
            if(!setRgb(sensor, (int8_t)rgbColor))
            {
                std::cout << "Failed to set color!" << std::endl;
            }
            exit(0);
        }

        if(setSerial)
        {
            std::cout << "Setting illuminator serial number: " << serialNumber << std::endl;
            if(!sensor.setIbSerial(serialNumber.c_str()))
            {
                std::cout << "Failed to set serial number!" << std::endl;
            }
            exit(0);
        }

        if(vled) 
        {
            std::cout << "Setting VLED: " << vled << "mV" << std::endl;
            if(!sensor.setVled(vled))
            {
                std::cout << "Failed to set VLED!" << std::endl;
            }
            exit(0);
        }

        if(vledLimits.size() == 2)
        {
            std::cout << "Setting VLED Limits. [MIN: " << vledLimits[0] << " MAX: " << vledLimits[1] << "]" << std::endl;
            sensor.setFactoryMode(true);
            auto limits = std::make_tuple(vledLimits[0], vledLimits[1]);
            if(!sensor.setVledMinMax(limits))
            {
                std::cout << "Failed to set VLED Limits!" << std::endl;
            }
            exit(0);
        }
        else if((vledLimits.size() > 0))
        {
            std::cout << "Please provide exactly two arguments to vled-limits in the form <min, max>" << std::endl;
            exit(0);
        }

        if(emitterEnable >= 0)
        {
            std::cout << "Setting emitter enable: " << (unsigned) emitterEnable << std::endl;
            bool setOk = sensor.setVledEnables((uint8_t) emitterEnable);
            int8_t emitterEnableState { -1 };
            sensor.getVledEnables((uint8_t&) emitterEnableState);
            if(!setOk || (emitterEnableState != emitterEnable))
            {
                std::cout << "Failed to set emitter enable!" << std::endl;
            }
            exit(0);
        }

        std::cout << "Starting Illuminator Board Test!" << std::endl;
        if(testCommsI2C(sensor))
        {
            std::cout << "Passed Comms test." << std::endl;
        }
        else
        {
            std::cout << "Failed comms test!" << std::endl;
        }

        dumpStats(sensor);

        if(runRgbTest)
        {
            testRgbs(sensor);
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
