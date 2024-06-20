
#include "tof_sensor.hpp"
#include "TofCommand_IF.hpp"
#include "tofcore/TofEndian.hpp"
#include <boost/endian/conversion.hpp>
#include <boost/scope_exit.hpp>
#include <cmath>
#include <stdio.h>

namespace tofcrust
{

using namespace std;
using namespace boost::endian;
using namespace TofComm;

std::optional<std::string> Sensor::getLogSettings()
{
    auto result = send_receive(COMMAND_GET_LOG_SETTINGS);
    bool isOk = result.has_value();
    if (!isOk)
    {
        return std::nullopt;
    }
    else
    {
        const char *begin = reinterpret_cast<const char*>(result->data());
        std::string tsData { begin, result->size() };
        return { tsData };
    }
}

std::optional<std::string> Sensor::getTestStationData()
{
    auto result = send_receive(COMMAND_GET_TEST_STATION_DATA);
    bool isOk = result.has_value();
    if (!isOk)
    {
        return std::nullopt;
    }
    else
    {
        const char *begin = reinterpret_cast<const char*>(result->data());
        std::string tsData { begin, result->size() };
        return { tsData };
    }
}

std::optional<StorageMetadata_T> Sensor::getStorageMetadata(TofComm::StorageId_e id, TofComm::StorageMode_e mode)
{
    StorageMetadata_T metaData {id, mode };
    auto isOk { false };
    auto result = this->send_receive(COMMAND_STORAGE_GET_METADATA, {(std::byte*)&metaData, sizeof(metaData)});
    isOk = result.has_value();
    if (!isOk)
    {
        return std::nullopt;
    }
    const auto data = (*result).data();
    {
        uint32_t v32;
        BE_Get(v32, data + offsetof(StorageMetadata_T, m_dataSize));
        metaData.m_dataSize = v32;
        BE_Get(v32, data + offsetof(StorageMetadata_T, m_dataCrc32));
        metaData.m_dataCrc32 = v32;
    }
    {
        uint16_t v16;
        BE_Get(v16, data + offsetof(StorageMetadata_T, m_dataType));
        metaData.m_dataType = v16;
        BE_Get(v16, data + offsetof(StorageMetadata_T, m_typeVersion));
        metaData.m_typeVersion = v16;
    }
    return {metaData};
}

std::optional<std::tuple<double, double>> Sensor::getThermalProtectionThresholds()
{
    auto result = this->send_receive(COMMAND_GET_THERMAL_PROTECTION_SETTINGS);
    bool isOk = result.has_value();
    if (!isOk)
    {
        return std::nullopt;
    }
    const auto dataSize = result->size();
    if (dataSize != 2 * sizeof(int32_t))
    {
        return std::nullopt;
    }
    const auto data = (*result).data();
    int32_t entryMilliDegC { 0 };
    BE_Get(entryMilliDegC, data);
    int32_t exitMilliDegC { 0 };
    BE_Get(exitMilliDegC, data + sizeof(int32_t));

    return std::make_tuple(entryMilliDegC/1000.0, exitMilliDegC/1000.0);
}

/// @brief Set the lens correction intrinsics with pre-scaled integer values.
/// @param offset_x lens center X offset from the center pixel of the sensor array scaled by 1e-2 to fit in int16_t
/// @param offset_y lens center Y offset from the center pixel of the sensor array scaled by 1e-2 to fit in int16_t
/// @param focal_len_x X focal length of the lens scaled by 1e-6 to fit in uint32_t
/// @param focal_len_y Y focal length of the lens scaled by 1e-6 to fit in uint32_t
/// @param undistortion_coeff 5 undistortion coefficents each value is scaled by 1e-8 to fit in uint32_t
/// @return true on success.
bool Sensor::setLensInfo(int16_t offset_x, int16_t offset_y, uint32_t focal_len_x, uint32_t focal_len_y, const std::array<int32_t, 5>& undistortion_coeff)
{
    std::array<std::byte, RAW_SENSOR_INFO_DATA_SIZE> buffer;
    auto ptr = buffer.data();
    ptr = BE_Append(ptr, offset_x);
    ptr = BE_Append(ptr, offset_y);
    ptr = BE_Append(ptr, focal_len_x);
    ptr = BE_Append(ptr, focal_len_y);
    for(const auto& coeff : undistortion_coeff) 
    {
        ptr = BE_Append(ptr, coeff);
    }
    auto result = this->send_receive(COMMAND_SET_LENS_INFO, {buffer.data(), buffer.size()}, 35s);

    return result.has_value();
}


/// @brief Set the lens correction intrinsics
/// @param offset_x lens center X offset from the center pixel of the sensor array
/// @param offset_y lens center Y offset from the center pixel of the sensor array
/// @param focal_len_x X focal length of the lens
/// @param focal_len_y Y focal length of the lens
/// @param undistortion_coeff 5 undistortion coefficents
/// @return true on success.
bool Sensor::setLensInfo(double offset_x, double offset_y, double focal_len_x, double focal_len_y, const std::array<double, 5>& undistortion_coeff)
{
    int16_t scaled_offset_x = std::floor(offset_x * 1.0e2 + 0.5);
    int16_t scaled_offset_y = std::floor(offset_y * 1.0e2 + 0.5);
    uint32_t scaled_focal_len_x = std::floor(focal_len_x * 1.0e6 + 0.5);
    uint32_t scaled_focal_len_y  = std::floor(focal_len_y * 1.0e6 + 0.5);
    std::array<int32_t, 5> scaled_undistortion_coeff;
    std::transform(undistortion_coeff.begin(), undistortion_coeff.end(), 
                    scaled_undistortion_coeff.begin(),
                    [](const auto& in) {
                        return std::floor( in * 1.0e8 + 0.5);
                    });
    return this->setLensInfo(scaled_offset_x, scaled_offset_y, scaled_focal_len_x, scaled_focal_len_y,
                             scaled_undistortion_coeff);
}


std::optional<std::vector<std::byte> > Sensor::storageRead(StorageId_e id, TofComm::StorageMode_e mode,
                                                           uint32_t storageOffset, uint32_t numBytes)
{
    StorageReadCommand_T readCommand { id, mode };
    BE_Put(&readCommand.m_offset, storageOffset);
    BE_Put(&readCommand.m_numBytes, numBytes);
    
    auto result = this->send_receive(COMMAND_STORAGE_READ, {(std::byte*)&readCommand, sizeof(readCommand)});

    return result;
}

bool Sensor::storageWriteData(StorageId_e id, TofComm::StorageMode_e mode,
                              uint32_t storageOffset, std::byte *data, uint32_t numBytes)
{
    const WriteContinueCommand_T cmd { id, mode, native_to_big(storageOffset) };
    const std::vector<send_receive_payload_t> cmdAndData
    {
        { (std::byte*)&cmd, sizeof(cmd) },
        { data, (size_t)numBytes}
    };
    auto result = this->send_receive(COMMAND_STORAGE_WRITE, cmdAndData);
    return result.has_value();
}

bool Sensor::storageWriteFinish(TofComm::StorageId_e id, TofComm::StorageMode_e mode, uint32_t totalSize, uint32_t crc32)
{
    const WriteFinishCommand_T cmd { id, mode, native_to_big(totalSize), native_to_big(crc32) };
    auto result = this->send_receive(COMMAND_STORAGE_WRITE, {(std::byte*)&cmd, sizeof(cmd)});
    return result.has_value();
}

bool Sensor::storageWriteStart(TofComm::StorageId_e id, TofComm::StorageMode_e mode)
{
    const WriteStartCommand_T cmd { id, mode };
    auto result = this->send_receive(COMMAND_STORAGE_WRITE, {(std::byte*)&cmd, sizeof(cmd)});
    return result.has_value();
}


bool Sensor::setDllStep(bool enable, uint8_t coarseStep, uint8_t fineStep, uint8_t finestStep){

    uint8_t params[] = {(uint8_t)enable, coarseStep, fineStep, finestStep};

    return this->send_receive(COMMAND_SET_DLL_STEP, {(std::byte*)params, sizeof(params)}).has_value();  
}

bool Sensor::readRegister(uint8_t regAddress, uint8_t& regData){

    auto result = this->send_receive(COMMAND_READ_REGISTER, regAddress);

    auto ok = bool{result};

    const auto& payload = *result;

    ok &= (payload.size() == READ_REGISTER_DATA_SIZE); // Data type field is 6 for REG READ

    if (ok)
    {
        regData = (uint8_t)payload[READ_REGISTER_DATA_OFFSET];
    }

    return ok;
}

bool Sensor::sdramTestRequest()
{
    auto ok = bool{ this->send_receive(COMMAND_SDRAM_TEST) };
    return ok;
}

std::optional<uint16_t> Sensor::sequencerGetVersion() const
{
    auto result = this->send_receive(COMMAND_SEQU_GET_VERSION);

    auto ok = bool{result};
    const auto& payload = *result;

    ok &= (payload.size() == sizeof(uint16_t));
    if (!ok)
    {
        return std::nullopt;
    }
    uint16_t version { 0 };
    BE_Get(version, &payload[0]);

    return {version};
}

std::optional<bool> Sensor::sequencerIsVersionSupported(uint16_t version) const
{
    auto result = this->send_receive(COMMAND_SEQU_VERSION_IS_SUPPORTED, version);

    auto ok = bool {result};
    const auto& payload = *result;

    ok &= (payload.size() == sizeof(uint8_t));
    if (!ok)
    {
        return std::nullopt;
    }
    bool isSupported { false };
    isSupported = (payload[0] != (std::byte)0);

    return {isSupported};
}

bool Sensor::sequencerSetVersion(uint16_t version)
{
    auto result = this->send_receive(COMMAND_SEQU_SET_VERSION, version);

    bool ok = bool{result};
    const auto& payload = *result;

    ok &= (payload.size() == sizeof(uint8_t)) && ((std::byte)0 == payload[0]);

    return ok;
}

bool Sensor::setFactoryMode(bool enable)
{
    uint8_t payload { (uint8_t)(enable ? 1 : 0) };
    return this->send_receive(COMMAND_FACTORY_MODE, payload).has_value();
}

bool Sensor::setFramePeriodMsLimits(std::tuple<uint32_t, uint32_t> minAndMaxMs)
{
    auto [minMs, maxMs] = minAndMaxMs;
    std::array<std::byte, 2 * sizeof(uint32_t)> buffer;
    auto ptr = buffer.data();
    ptr = BE_Append(ptr, minMs);
    ptr = BE_Append(ptr, maxMs);
    auto result = this->send_receive(COMMAND_SET_FRAME_PERIOD_LIMITS, {buffer.data(), buffer.size()});
    return result.has_value();
}

bool Sensor::setIntegrationTimeUsLimits(std::tuple<uint16_t, uint16_t> minMaxUs)
{
    auto [minUs, maxUs] = minMaxUs;
    std::array<std::byte, 2 * sizeof(uint16_t)> buffer;
    auto ptr = buffer.data();
    ptr = BE_Append(ptr, minUs);
    ptr = BE_Append(ptr, maxUs);
    auto result = this->send_receive(COMMAND_SET_INTEG_TIME_LIMITS, {buffer.data(), buffer.size()});
    return result.has_value();
}

bool Sensor::setLogSettings(std::optional<std::string> globalEnables, std::optional<std::string> categoryLevels)
{
    std::string jsonToSend { "{" };
    if (globalEnables)
    {
        jsonToSend.append("\"globalEnabledLevels\":").append(*globalEnables);
    }
    if (categoryLevels)
    {
        if (globalEnables)
        {
            jsonToSend.append(",");
        }
        jsonToSend.append("\"categoryLevels\":").append(*categoryLevels);
    }
    jsonToSend.append("}");

    return this->send_receive(COMMAND_SET_LOG_SETTINGS, {(std::byte*)jsonToSend.c_str(), jsonToSend.size()}).has_value();
}

bool Sensor::setMinAmplitudeLimits(std::tuple<uint16_t, uint16_t> minMaxAmplitude)
{
    auto [minAmplitude, maxAmplitude] = minMaxAmplitude;
    std::array<std::byte, 2 * sizeof(uint16_t)> buffer;
    auto ptr = buffer.data();
    ptr = BE_Append(ptr, minAmplitude);
    ptr = BE_Append(ptr, maxAmplitude);
    auto result = this->send_receive(COMMAND_SET_MIN_AMPLITUDE_LIMITS, {buffer.data(), buffer.size()});
    return result.has_value();
}

bool Sensor::setModelName(const char *modelName)
{
    return this->send_receive(COMMAND_SET_MODEL_NAME, {(std::byte*)modelName, SERIAL_NUMBER_SIZE}).has_value();
}

bool Sensor::setModulationFreqKhzLimitsAndStep(std::tuple<uint16_t, uint16_t, uint16_t> minMaxStepKhz)
{
    auto [minKhz, maxKhz, stepKhz] = minMaxStepKhz;
    std::array<std::byte, 3 * sizeof(uint16_t)> buffer;
    auto ptr = buffer.data();
    ptr = BE_Append(ptr, minKhz);
    ptr = BE_Append(ptr, maxKhz);
    ptr = BE_Append(ptr, stepKhz);
    auto result = this->send_receive(COMMAND_SET_MOD_FREQ_LIMITS, {buffer.data(), buffer.size()});
    return result.has_value();
}

bool Sensor::setSerialCpuBoard(const char *serial)
{
    return this->send_receive(COMMAND_SET_CPU_SERIAL, {(std::byte*)serial, SERIAL_NUMBER_SIZE}).has_value();
}

bool Sensor::setSerialDevice(const char *serial)
{
    return this->send_receive(COMMAND_SET_DEV_SERIAL, {(std::byte*)serial, SERIAL_NUMBER_SIZE}).has_value();
}

bool Sensor::setTestStationData(const char *tsData)
{
    size_t length = strlen(tsData);
    return this->send_receive(COMMAND_SET_TEST_STATION_DATA, {(std::byte*)tsData, length+1}).has_value();
}

bool Sensor::setThermalProtectionThresholds(std::tuple<double, double> thresholds)
{
    std::array<std::byte, 2 * sizeof(int32_t)> buffer;
    auto ptr = buffer.data();
    int32_t milliDegC = std::get<0>(thresholds) * 1000.0;
    ptr = BE_Append(ptr, milliDegC);
    milliDegC = std::get<1>(thresholds) * 1000.0;
    ptr = BE_Append(ptr, milliDegC);
    auto result = this->send_receive(COMMAND_SET_THERMAL_PROTECTION_SETTINGS, {buffer.data(), buffer.size()});
    return result.has_value();
}

bool Sensor::setVledMinMax(std::tuple<uint16_t, uint16_t> vledLimits)
{
    uint16_t vledMin = std::get<0>(vledLimits);
    uint16_t vledMax = std::get<1>(vledLimits);

    uint16_t params[] = {native_to_big(vledMin), native_to_big(vledMax)};
    return this->send_receive(COMMAND_SET_VLED_LIMITS, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::writeRegister(uint8_t regAddress, uint8_t regData){

    uint8_t params[] = {regAddress, regData};

    return this->send_receive(COMMAND_WRITE_REGISTER, {(std::byte*)params, sizeof(params)}).has_value();
}


/* #########################################################################
* 
* Illuminator Board Commands
*
* ########################################################################## */

bool Sensor::setVledEnables(uint8_t vledEnables)
{
    return this->send_receive(COMMAND_SET_VLED_ENABLES, vledEnables).has_value();
}

bool Sensor::getVledEnables(uint8_t& vledEnables)
{
    auto result = this->send_receive(COMMAND_GET_VLED_ENABLES);

    auto ok = bool{result};

    const auto& payload = *result;

    ok &= (payload.size() == VLED_ENABLES_DATA_SIZE); 

    if (ok)
    {
        vledEnables = (uint8_t)payload[VLED_ENABLES_DATA_OFFSET];
    }

    return ok;
}

bool Sensor::setVled(uint16_t vledMv)
{
    return this->send_receive(COMMAND_SET_VLED, vledMv).has_value();
}

bool Sensor::getVled(uint16_t& vledMv)
{
    auto result = this->send_receive(COMMAND_GET_VLED);

    auto ok = bool{result};
    const auto& payload = *result;
    ok &= (payload.size() == VLED_DATA_SIZE);
    if (ok)
    {
        BE_Get(vledMv, &payload[VLED_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::getIb5V(uint16_t& v5Mv)
{
    auto result = this->send_receive(COMMAND_GET_IB_5V);

    auto ok = bool{result};
    const auto& payload = *result;
    ok &= (payload.size() == IB_5V_DATA_SIZE);
    if (ok)
    {
        BE_Get(v5Mv, &payload[IB_5V_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::getIllmnTemperature(int32_t& tempMdegC)
{
    auto result = this->send_receive(COMMAND_GET_IB_TEMPERATURE);

    auto ok = bool{result};
    const auto& payload = *result;
    ok &= (payload.size() == IB_TEMPERATURE_DATA_SIZE);
    if (ok)
    {
        BE_Get(tempMdegC, &payload[IB_TEMPERATURE_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::getIbPd(uint16_t& photodiodeMv)
{
    auto result = this->send_receive(COMMAND_GET_IB_PD);

    auto ok = bool{result};
    const auto& payload = *result;
    ok &= (payload.size() == IB_PD_DATA_SIZE);
    if (ok)
    {
        BE_Get(photodiodeMv, &payload[IB_PD_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::setIbSerial(const char *serialNum)
{
    char serial[ILLMN_SERIAL_STRING_SIZE];
    snprintf(serial, ILLMN_SERIAL_STRING_SIZE, "%s", serialNum);
    return this->send_receive(COMMAND_SET_SERIAL_NUM, {(std::byte*)serial, sizeof(serial)}).has_value();
}

bool Sensor::setIbRgb(uint8_t rgbBitmap, uint16_t rgbBlinkMs)
{
    uint8_t params[sizeof(rgbBitmap) + sizeof(rgbBlinkMs)];
    params[0] = rgbBitmap;
    BE_Put(&params[sizeof(rgbBitmap)], rgbBlinkMs);

    return this->send_receive(COMMAND_SET_RGB, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::getIbRgb(uint8_t& rgbBitmap)
{
    auto result = this->send_receive(COMMAND_GET_RGB);

    auto ok = bool{result};
    const auto& payload = *result;
    ok &= (payload.size() == RGB_DATA_SIZE);
    if (ok)
    {
        rgbBitmap = (uint8_t)payload[RGB_DATA_OFFSET];
    }

    return ok;
}

bool Sensor::setTestVal(uint8_t testVal)
{
    return this->send_receive(COMMAND_SET_IB_TEST_8BIT, testVal).has_value();
}

bool Sensor::setTestVal(uint16_t testVal)
{
    return this->send_receive(COMMAND_SET_IB_TEST_16BIT, testVal).has_value();
}

bool Sensor::getTestVal(uint8_t& testVal)
{
    auto result = this->send_receive(COMMAND_GET_IB_TEST_8BIT);

    auto ok = bool{result};
    const auto& payload = *result;
    ok &= (payload.size() == IB_TEST_8BIT_SIZE);
    if (ok)
    {
        BE_Get(testVal, &payload[IB_TEST_8BIT_OFFSET]);
    }

    return ok;
}

bool Sensor::getTestVal(uint16_t& testVal)
{
    auto result = this->send_receive(COMMAND_GET_IB_TEST_16BIT);

    auto ok = bool{result};
    const auto& payload = *result;
    ok &= (payload.size() == IB_TEST_16BIT_SIZE);
    if (ok)
    {
        BE_Get(testVal, &payload[IB_TEST_16BIT_OFFSET]);
    }

    return ok;
}


} //end namespace tofcrust
