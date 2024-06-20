/**
 * @file tof-rw.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program to read Tof persistent storage
 */
#include "crc32.h"
#include "tofcrust/tof_sensor.hpp"
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

enum class OutputMode_e { BINARY, TEXT, HEX };

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string calFileName { };
static std::string devicePort { DEFAULT_PORT_NAME };
static bool eraseStorage { false };
static volatile bool exitRequested { false };
static bool getMetadata { false };
static bool getTestStationData { false };
static uint32_t hexBytesPerLine { 16 };
static ManufacturingData_T manufacturingData { };
static std::string manufacturingDataString {};
static uint32_t maxSizePerRead { 0 };
static std::string modelNameStr { };
static OutputMode_e outputMode { OutputMode_e::BINARY };
static std::string refFrameFileName { };
static bool setModelName { false };
static bool setTestStationData { false };
static StorageId_e storageId { StorageId_e::PERSISTENT_SETTINGS };
static StorageMode_e storageMode { StorageMode_e::EXTRACTED };
static uint32_t storageOffset { 0 };
static uint32_t storageSize { 0 };
static std::string testStationData {};
static bool writeManufacturingData { false };

static bool doErase(tofcrust::Sensor& sensor)
{
    const bool ok { sensor.setFactoryMode(true) && sensor.storageWriteStart(storageId, StorageMode_e::EXTRACTED) };
    return ok;
}

static std::tuple<bool, std::vector<std::byte>> doRead(tofcrust::Sensor& sensor, const uint32_t offset, const uint32_t size)
{
    bool ok { true };
    std::vector<std::byte> data {};

    uint32_t totalBytesRead { 0 };

    while (true)
    {
        // If maxSizePerRead was specified, limit read size to it; otherwise, use the size passed in
        const uint32_t bytesToRead
            { (0 == maxSizePerRead) ?
                        size : ((0 == size) ? maxSizePerRead : std::min(maxSizePerRead, size)) };

        const auto readResult = sensor.storageRead(storageId, storageMode, offset + totalBytesRead, bytesToRead);

        ok = readResult.has_value();
        if (!ok)
        {
            break; // done if there was an error
        }
        /*
         * Append the data read to the result.
         */
        const auto& v = *readResult;
        data.insert(data.end(), v.begin(), v.end()); // Append to vector that will be returned
        const size_t bytesRead { v.size() };
        totalBytesRead += bytesRead;
        /*
         * Check to see if multiple reads are being used to limit the size of
         * each one. We're done when either no size was specified for the read
         * or when that number of bytes have been read.
         */
        if (    (0 == bytesToRead)          // Done if no read size was specified (we got "all")
             || (bytesRead < bytesToRead)   // .. or if we got less than requested (we got "all that remained")
             || ( (size != 0) && (totalBytesRead >= size) ) ) // .. or we got the number requested
        {
            break;
        }
    }

    return std::make_tuple(ok, data);
}

static void outputBinary(const std::vector<std::byte>& pData)
{
    for (auto b : pData)
    {
        std::cout << std::to_integer<uint8_t>(b);
    }
}

static void outputHex(const std::vector<std::byte>& pData)
{
    uint32_t count { 0 };
    std::string pchars { "  " };
    uint32_t offset { 0 };
    std::cout << "[" << std::hex << std::setw(6) << std::setfill('0') << offset << "]";
    for (auto b : pData)
    {
        std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << (unsigned)b;
        const char c { (char)b };
        constexpr char period { '.' };
        if (isprint(c)) pchars.push_back(c);
        else            pchars.push_back(period);
        if ((++count % hexBytesPerLine) == 0)
        {
            offset += hexBytesPerLine;
            std::cout << pchars.c_str() << std::endl
                      << "[" << std::hex << std::setw(6) << std::setfill('0') << offset << "]";
            pchars = "  ";
        }
    }
    std::cout << std::endl << std::dec;
}

static void outputText(const std::vector<std::byte>& pData)
{
    std::cout << reinterpret_cast<const char*>(pData.data()) << std::endl;
}

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Presistent data read/write utility");

    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("binary-output,B", "Output BINARY data (DEFAULT)")
        ("c", "Read calibration storage")
        ("cal-file,P", po::value<std::string>(&calFileName), "Program calibration data from file AND EXIT")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("erase,E", po::bool_switch(&eraseStorage)->default_value(false), "ERASE the specified storage instead of read; e.g., '--m -E' erases manufacturing data storage")
        ("extracted-mode,e", "EXTRACTED mode access (DEFAULT)")
        ("help,h", "produce help message")
        ("hex-output,x", "Output HEX")
        ("l", "Read Lens data storage")
        ("L", "Read Log storage")
        ("m", "Read Manufacturing data storage")
        ("M", po::value<std::string>(&manufacturingDataString), "Store manufacturing data AND EXIT.\n"
                    "Manufacturing data consists of the following comma-separated items:\n"
                    "   IPV4Addr,Mask,Gateway: uu.uu.uu.uu\n"
                    "   device serial#: string\n"
                    "   cpu board serial number: string\n"
                    "   model name, test station information: strings\n"
                    "EXAMPLE: tof-rw --M \"10.10.31.180,255.255.255.0,10.10.31.1,2468135790,9876543210,Mojave,{\\\"foo\\\":\\\"bar\\\"}\"")
        ("meta-data,C", po::bool_switch(&getMetadata)->default_value(false), "Read CRC32 & size metadata instead of actual data")
        ("offset,o", po::value<uint32_t>(&storageOffset), "Start reading <offset> bytes in (default is 0)")
        ("read-ref-frame,r", "Read reference frame")
        ("write-ref-frame,R", po::value<std::string>(&refFrameFileName), "Write reference frame from file AND EXIT")
        ("size,s", po::value<uint32_t>(&storageSize), "Read <size> bytes (default is 0, meaning all)")
        ("S", "Read Persistent settings storage")
        ("text-output,t", "Output TEXT")
        ("w", po::value<uint32_t>(&hexBytesPerLine), "In HEX mode, output N bytes per line")
        ("raw-mode,W", "storage mode RAW")
        ("X", po::value<uint32_t>(&maxSizePerRead), "Maximum size of reads; break up larger reads if necessary")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    if(vm.count("M"))
    {
        unsigned ipv4Address[4] {0};
        unsigned ipv4Mask[4] {0};
        unsigned ipv4GW[4] {0};
        char deviceSerialNum[SERIAL_NUMBER_SIZE] {};
        char cpuBoardSerialNum[SERIAL_NUMBER_SIZE] {};
        char modelName[SERIAL_NUMBER_SIZE] {};
        char testStationData[TEST_STATION_DATA_SIZE] {};
        const auto formatStr =
            "%u.%u.%u.%u,%u.%u.%u.%u,%u.%u.%u.%u,%" +
            std::to_string(SERIAL_NUMBER_SIZE - 1) + "[^,],%" +
            std::to_string(SERIAL_NUMBER_SIZE - 1) + "[^,],%" +
            std::to_string(SERIAL_NUMBER_SIZE - 1) + "[^,],%" +
            std::to_string(TEST_STATION_DATA_SIZE - 1) + "[^,]";
        int n = sscanf(manufacturingDataString.c_str(), formatStr.c_str(),
                       ipv4Address + 0, ipv4Address + 1, ipv4Address + 2,
                       ipv4Address + 3, ipv4Mask + 0, ipv4Mask + 1,
                       ipv4Mask + 2, ipv4Mask + 3, ipv4GW + 0, ipv4GW + 1,
                       ipv4GW + 2, ipv4GW + 3, deviceSerialNum,
                       cpuBoardSerialNum, modelName, testStationData);
        if(16 == n)
        {
            for (size_t i = 0; i < sizeof(manufacturingData.m_ipv4Address); ++i)
            {
                manufacturingData.m_ipv4Address[i] = ipv4Address[i];
                manufacturingData.m_ipv4Mask[i] = ipv4Mask[i];
                manufacturingData.m_ipv4Gateway[i] = ipv4GW[i];
            }
            strncpy(manufacturingData.m_deviceSerialNumber, deviceSerialNum, sizeof(manufacturingData.m_deviceSerialNumber));
            strncpy(manufacturingData.m_modelName, modelName, sizeof(manufacturingData.m_modelName));
            strncpy(manufacturingData.m_cpuBoardSerialNumber, cpuBoardSerialNum, sizeof(manufacturingData.m_cpuBoardSerialNumber));
            strncpy(manufacturingData.m_testStationData, testStationData, sizeof(manufacturingData.m_testStationData));
            writeManufacturingData = true;
        }
        else
        {
            std::cerr << "Invalid manufacturing data: '" << manufacturingDataString.c_str() << "' - " << n << std::endl;
            exit(-1);
        }
    }
    if(vm.count("c")) storageId = StorageId_e::CALIBRATON;
    if(vm.count("l")) storageId = StorageId_e::LENS_DATA;
    if(vm.count("L")) storageId = StorageId_e::LOG;
    if(vm.count("m")) storageId = StorageId_e::MANUFACTURING_DATA;
    if(vm.count("S")) storageId = StorageId_e::PERSISTENT_SETTINGS;
    if(vm.count("read-ref-frame")) storageId = StorageId_e::REFERENCE_FRAME;

    if(vm.count("binary-output")) outputMode = OutputMode_e::BINARY;
    if(vm.count("text-output")) outputMode = OutputMode_e::TEXT;
    if(vm.count("hex-output")) outputMode = OutputMode_e::HEX;

    if(vm.count("extracted-mode")) storageMode = StorageMode_e::EXTRACTED;
    if(vm.count("raw-mode")) storageMode = StorageMode_e::RAW;
}

static int programCalibration(tofcrust::Sensor& sensor)
{
    std::ifstream inFile { };
    inFile = std::ifstream(calFileName, (std::ios::in | std::ios::binary));
    if (!inFile.is_open())
    {
        std::cerr << "ERROR: Failed to open input file for calibration data: " << calFileName << std::endl;
        return -1;
    }
    /*
     * Erase existing cal
     */
    if (!sensor.storageWriteStart(StorageId_e::CALIBRATON, StorageMode_e::EXTRACTED))
    {
        std::cerr << "FAILED to erase sensor calibration" << std::endl;
        return -1;
    }
    std::cout << "Sensor calibration metadata erased." << std::endl;
    /*
     * Program cal data 4k at a time
     */
    uint32_t offset { 0 };
    uint32_t crc { 0 };
    const auto startTime { high_resolution_clock::now() };
    while (inFile)
    {
        constexpr uint32_t BUFFER_SIZE { 4096 }; // must match sensor NOR's sector size
        std::byte readBuffer[BUFFER_SIZE];
        auto numRead = inFile.read(reinterpret_cast<char*>(readBuffer), BUFFER_SIZE).gcount();
        if (0 == numRead)
        {
            break; // This happens if file size is an exact multiple of 4K
        }
        if (!sensor.storageWriteData(StorageId_e::CALIBRATON, StorageMode_e::EXTRACTED, offset, readBuffer, numRead))
        {
            std::cerr << std::endl << "FAILED writing " << numRead << " bytes to calibration offset " << offset << std::endl;
            return -1;
        }
        crc = updateCrc32(crc, (uint8_t*)readBuffer, numRead);
        offset += numRead;
        std::cout << std::dec << offset << " (0x" << std::hex << offset << ") bytes programmed\r" << std::flush;
    }
    /*
     * Update size/crc metadata
     */
    if (!sensor.storageWriteFinish(StorageId_e::CALIBRATON, StorageMode_e::EXTRACTED, offset, crc))
    {
        std::cerr << "FAILED to update calibration metadata" << std::endl;
        return -1;
    }
    const auto endTime { high_resolution_clock::now() };
    std::cout << std::endl << "Calibration metadata updated: size = " << std::dec << offset << " (0x" << std::hex << offset
              << "); CRC = 0x" << crc << std::dec << std::endl;
    const auto durationMs = duration_cast<milliseconds>(endTime - startTime);
    const double durationSec { durationMs.count()/1000.0 };
    std::cout << "SUCCESS: Sensor calibration programmed in " << durationSec << " seconds ("
              << (offset/durationSec) << " bytes/sec)" << std::endl;

    return 0;
}

static int programReferenceFrame(tofcrust::Sensor& sensor)
{
    std::ifstream inFile { };
    inFile = std::ifstream(refFrameFileName, (std::ios::in | std::ios::binary));
    if (!inFile.is_open())
    {
        std::cerr << "ERROR: Failed to open input file for reference frame data: " << refFrameFileName << std::endl;
        return -1;
    }
    /*
     * Erase existing reference frame
     */
    if (!sensor.storageWriteStart(StorageId_e::REFERENCE_FRAME, StorageMode_e::EXTRACTED))
    {
        std::cerr << "FAILED to erase sensor reference frame" << std::endl;
        return -1;
    }
    std::cout << "Sensor reference frame metadata erased." << std::endl;
    /*
     * Program reference frame data 4k at a time
     */
    uint32_t offset { 0 };
    uint32_t crc { 0 };
    const auto startTime { high_resolution_clock::now() };
    while (inFile)
    {
        constexpr uint32_t BUFFER_SIZE { 4096 }; // must match sensor NOR's sector size
        std::byte readBuffer[BUFFER_SIZE];
        auto numRead = inFile.read(reinterpret_cast<char*>(readBuffer), BUFFER_SIZE).gcount();
        if (0 == numRead)
        {
            break; // This happens if file size is an exact multiple of 4K
        }
        if (!sensor.storageWriteData(StorageId_e::REFERENCE_FRAME, StorageMode_e::EXTRACTED, offset, readBuffer, numRead))
        {
            std::cerr << std::endl << "FAILED writing " << numRead << " bytes to reference frame offset " << offset << std::endl;
            return -1;
        }
        crc = updateCrc32(crc, (uint8_t*)readBuffer, numRead);
        offset += numRead;
        std::cout << std::dec << offset << " (0x" << std::hex << offset << ") bytes programmed\r" << std::flush;
    }
    /*
     * Update size/crc metadata
     */
    if (!sensor.storageWriteFinish(StorageId_e::REFERENCE_FRAME, StorageMode_e::EXTRACTED, offset, crc))
    {
        std::cerr << "FAILED to update reference frame metadata" << std::endl;
        return -1;
    }
    const auto endTime { high_resolution_clock::now() };
    std::cout << std::endl << "Reference frame metadata updated: size = " << std::dec << offset << " (0x" << std::hex << offset
              << "); CRC = 0x" << crc << std::dec << std::endl;
    const auto durationMs = duration_cast<milliseconds>(endTime - startTime);
    const double durationSec { durationMs.count()/1000.0 };
    std::cout << "SUCCESS: Sensor reference frame programmed in " << durationSec << " seconds ("
              << (offset/durationSec) << " bytes/sec)" << std::endl;

    return 0;
}

static void readMetadata(tofcrust::Sensor& sensor)
{
    auto result = sensor.getStorageMetadata(storageId, storageMode);
    if (!result)
    {
        std::cerr << "Failed to read metadata" << std::endl;
    }
    else
    {
        StorageMetadata_T& metadata { *result };
        std::cout << "size: " << metadata.m_dataSize << " bytes  CRC32: 0x"
                  << std::setw(8) << std::setfill('0') << std::hex << metadata.m_dataCrc32 << std::dec;
        if (StorageMode_e::EXTRACTED == metadata.m_mode)
        {
            std::cout << "  type: " << metadata.m_dataType << "  version: " << metadata.m_typeVersion;
        }
        std::cout << std::endl;
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
        bool noOtherCommand { true };
        if (calFileName.size() != 0)
        {
            noOtherCommand = false;
            if (programCalibration(sensor) != 0)
            {
                exit(-1);
            }
        }
        if (refFrameFileName.size() != 0)
        {
            noOtherCommand = false;
            if (programReferenceFrame(sensor) != 0)
            {
                exit(-1);
            }
        }
        if (writeManufacturingData)
        {
            noOtherCommand = false;
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "setFactoryMode() FAILED" << std::endl;
                exit(-1);
            }
            if (!sensor.storageWriteStart(StorageId_e::MANUFACTURING_DATA, StorageMode_e::EXTRACTED))
            {
                std::cerr << "storageWriteStart() FAILED" << std::endl;
                exit(-1);
            }
            if (!sensor.storageWriteData(StorageId_e::MANUFACTURING_DATA, StorageMode_e::EXTRACTED,
                                         0, (std::byte*)&manufacturingData, sizeof(manufacturingData)))
            {
                std::cerr << "storageWriteData() FAILED" << std::endl;
                exit(-1);
            }
            const uint32_t crc { calcCrc32((const uint8_t*)&manufacturingData, sizeof(manufacturingData)) };
            if (!sensor.storageWriteFinish(StorageId_e::MANUFACTURING_DATA, StorageMode_e::EXTRACTED, sizeof(manufacturingData), crc))
            {
                std::cerr << "storageWriteFinish() FAILED" << std::endl;
                exit(-1);
            }
        }
        if (eraseStorage)
        {
            noOtherCommand = false;
            if (doErase(sensor))
            {
                std::cout << "Erase successful" << std::endl;
            }
            else
            {
                std::cerr << "Erase failed" << std::endl;
            }
        }
        if (getMetadata)
        {
            noOtherCommand = false;
            readMetadata(sensor);
        }
        if (setModelName)
        {
            noOtherCommand = false;
            if (!sensor.setFactoryMode(true) || !sensor.setModelName(modelNameStr.c_str()))
            {
                std::cerr << "FAILED to set model name" << std::endl;
            }
        }
        if (setTestStationData)
        {
            noOtherCommand = false;
            if (!sensor.setFactoryMode(true) || !sensor.setTestStationData(testStationData.c_str()))
            {
                std::cerr << "FAILED to set test station data" << std::endl;
                exit(-1);
            }
            else
            {
                std::cout << "Set " << testStationData.size() << " bytes of test station data: '" << testStationData.c_str() << "'" << std::endl;
            }
        }
        if (getTestStationData)
        {
            noOtherCommand = false;
            auto tsData = sensor.getTestStationData();
            if (tsData)
            {
                std::cout << "Test station data: '" << tsData->c_str() << "'" << std::endl;
            }
            else
            {
                std::cerr << "FAILED to get test station data" << std::endl;
                exit(-1);
            }
        }
        if (noOtherCommand)
        {
            auto readResult = doRead(sensor, storageOffset, storageSize);

            if (std::get<bool>(readResult))
            {
                const auto &pData = std::get<1>(readResult);
                if (OutputMode_e::BINARY == outputMode)
                {
                    outputBinary(pData);
                }
                else if (OutputMode_e::HEX == outputMode)
                {
                    outputHex(pData);
                }
                else
                {
                    outputText(pData);
                }
            }
            else
            {
                std::cerr << "Failed to read storage data" << std::endl;
            }
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
