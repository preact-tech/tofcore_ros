#ifndef _TOF_SENSOR_ENG_HPP_
#define _TOF_SENSOR_ENG_HPP_

#include <tofcore/tof_sensor.hpp>
#include "tofcrust/CommandTypes.hpp"
#include <array>
#include <cstdint>
#include <optional>

namespace tofcrust
{

using tofcore::DEFAULT_BAUD_RATE;
using tofcore::DEFAULT_PORT_NAME;
using tofcore::DEFAULT_PROTOCOL_VERSION;

class Sensor : public tofcore::Sensor
{
public:
    using tofcore::Sensor::Sensor;

    std::optional<std::string> getLogSettings();

    std::optional<TofComm::StorageMetadata_T> getStorageMetadata(TofComm::StorageId_e id, TofComm::StorageMode_e mode);

    std::optional<std::string> getTestStationData();

    bool readRegister(uint8_t regAddress, uint8_t &regData);

    bool sdramTestRequest();

    std::optional<uint16_t> sequencerGetVersion() const;
    std::optional<bool> sequencerIsVersionSupported(uint16_t version) const;
    bool sequencerSetVersion(uint16_t version);

    bool setDllStep(bool enable, uint8_t coarseStep, uint8_t fineStep, uint8_t finestStep);
    bool setFactoryMode(bool enable);
    bool setLogSettings(std::optional<std::string> globalEnables, std::optional<std::string> categoryLevels);
    bool setModelName(const char *modelName);
    bool setSerialCpuBoard(const char *serial);
    bool setSerialDevice(const char *serial);
    bool setLensInfo(double offset_x, double offset_y, double focal_len_x, double focal_len_y, const std::array<double, 5>& undistortion_coeff);
    bool setLensInfo(int16_t offset_x, int16_t offset_y, uint32_t focal_len_x, uint32_t focal_len_y, const std::array<int32_t, 5>& undistortion_coeff);
    bool setTestStationData(const char *serial);

    std::optional<std::vector<std::byte> > storageRead(TofComm::StorageId_e id, TofComm::StorageMode_e mode,
                                                       uint32_t storageOffset, uint32_t numBytes);
    bool storageWriteData(TofComm::StorageId_e id, TofComm::StorageMode_e mode,
                          uint32_t storageOffset, std::byte *data, uint32_t numBytes);
    bool storageWriteFinish(TofComm::StorageId_e id, TofComm::StorageMode_e mode, uint32_t totalSize, uint32_t crc32);
    bool storageWriteStart(TofComm::StorageId_e id, TofComm::StorageMode_e mode);

    bool writeRegister(uint8_t regAddress, uint8_t regData);

    //Illuminator board commands
    bool getIbRgb(uint8_t& rgbBitmap);
    bool getIb5V(uint16_t& v5Mv);
    bool getIllmnTemperature(int32_t& tempMdegC);
    bool getIbPd(uint16_t& photodiodeMv);
    bool getTestVal(uint8_t& testVal);
    bool getTestVal(uint16_t& testVal);
    bool getVled(uint16_t& vledMv);
    bool getVledEnables(uint8_t& vledEnables);
    bool setVled(uint16_t vledMv);
    bool setVledEnables(uint8_t vledEnables);
    bool setIbRgb(uint8_t rgbBitmap, uint16_t rgbBlinkMs);
    bool setIbSerial(const char *serialNum);
    bool setTestVal(uint8_t testVal);
    bool setTestVal(uint16_t testVal);
};

} // end namespce tofcrust


#endif //_TOF_SENSOR_ENG_HPP_
