/**
 * @file tof_sensor.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Implements API for libtofcore
 */
#include "CommandTypes.hpp"
#include "tof_sensor.hpp"
#include "connection.hpp"
#include "comm_serial/serial_connection.hpp"
#include "tofcore/device_discovery.hpp"
#include "tofcore/TofEndian.hpp"
#include "TofCommand_IF.hpp"
#include "TofEndian.hpp"
#include "Measurement_T.hpp"
#include <boost/endian/conversion.hpp>
#include <boost/scope_exit.hpp>
#include <iostream>
#include <mutex>
#include <thread>

#if !defined(PACK_START)
#define PACK_START __pragma( pack(push, 1) )
#define PACK_END __pragma( pack(pop))
#endif

namespace tofcore {

using namespace std;
using namespace std::chrono_literals;
using namespace boost::endian;
using namespace TofComm;

struct Sensor::Impl
{
    Impl(const std::string &uri) :
        connection(Connection_T::create(ioService, uri)),
        measurement_timer_(ioService)
    {
    }

    Impl(std::unique_ptr<Connection_T> connection) : 
        connection(std::move(connection)),
        measurement_timer_(ioService)
    {
    }

    void init();

    boost::asio::io_service ioService;
    std::thread serverThread_;
    std::mutex measurementReadyMutex;
    on_measurement_ready_t measurementReady;
    std::unique_ptr<Connection_T> connection;

    /// The items below are specifically used for streaming via polling.
    /// Note: This mode is currently only used on Windows systems due to a strange Windows only phenomenon.
    ///  Data from the sensor can be randomly dropped for no reason that we can discern.
    ///  When data is dropped our parser gets out sync and cannot easily recover.
    ///  By using polling to request each frame we can easily detected a dropped frame (via a timeout)
    ///  at which time we can reset our parser and then request the next frame.
    ///  Some day we hope to figure out the real problem and or make our parser more resilient to dropped data
    //   but for now this is what we have.
    bool stream_via_polling_ { false };
    uint16_t measurement_command_ = 0;
    boost::asio::steady_timer measurement_timer_;

    /// Method used to initiate streaming of measurement data
    /// 
    /// @param measurement_command The measurement command ID to to send
    /// @return true on successful
    bool request_measurement_stream(uint16_t measurement_command)
    {
        this->measurement_command_ = measurement_command;
#if defined(_WIN32)
        stream_via_polling_ = (dynamic_cast<SerialConnection*>(this->connection.get()) != nullptr);
#else
        stream_via_polling_ = false;
#endif

        if(stream_via_polling_)
        {
            return this->request_next_measurement();
        }
        else
        {
            return this->connection->send_receive(this->measurement_command_,
                                                  &TofComm::CONTINUOUS_MEASUREMENT,
                                                  sizeof(TofComm::CONTINUOUS_MEASUREMENT),
                                                  std::chrono::seconds(5)).has_value();
        }
    }

    /// Method used to request the next measurement item when stream_via_polling_ is true.
    ///
    /// @return true on successful
    bool request_next_measurement()
    {
        auto f = [this](const boost::system::error_code error)
        {
            if ((error != boost::asio::error::operation_aborted) && this->stream_via_polling_)
            {
                this->connection->reset_parser();
                this->request_next_measurement();
            }
        };

        this->measurement_timer_.cancel();
        this->measurement_timer_.expires_after(250ms);
        this->measurement_timer_.async_wait(f);

        this->connection->send(this->measurement_command_, &TofComm::SINGLE_MEASUREMENT, sizeof(TofComm::SINGLE_MEASUREMENT));
        //Assume success: Note we can't use send_receive() here because this method could be called
        // from the context of the measurement callback and we would deadlock.
        return true;
    }
};

/* #########################################################################
 *
 * Sensor Class Implementation
 *
 * ######################################################################### */

Sensor::Sensor(const std::string& uri /*= std::string()*/)
{
    std::string connection_uri = uri;
    if (connection_uri.empty())
    {
        // Get all PreAct Devices
        const auto devices = find_all_devices();
        
        // No PreAct Devices
        if (devices.empty())
        {
            throw std::runtime_error("No PreAct Devices Found");
        }

        connection_uri = devices[0].connector_uri;
    }
    this->pimpl = std::unique_ptr<Impl>(new Impl(connection_uri));
    pimpl->init();
}


Sensor::Sensor(const std::string &portName, uint32_t baudrate) 
{
    std::string connection_uri = portName;
    // If no port name given. Find first PreAct device avaiable
    if (connection_uri.empty()){

        // Get all PreAct Devices
        std::vector<device_info_t> devices = find_all_devices();
        
        // No PreAct Devices
        if (devices.empty()){
            throw std::runtime_error("No PreAct Devices Found");
        }
        connection_uri = devices[0].connector_uri;
    }

    //Add baudrate to uri query parameters
    connection_uri += "?baudrate=";
    connection_uri += std::to_string(baudrate);
    this->pimpl = std::unique_ptr<Impl>(new Impl(connection_uri));
    pimpl->init();
}

Sensor::Sensor(std::unique_ptr<Connection_T> connection)
{
    this->pimpl = std::make_unique<Impl>(std::move(connection));
    pimpl->init();
}

Sensor::~Sensor()
{
    pimpl->ioService.stop();
    pimpl->serverThread_.join();
}

std::optional<uint32_t> Sensor::getFramePeriodMs()
{
    auto result = this->send_receive(COMMAND_GET_FRAME_PERIOD_MS);

    if (!result || (result->size() != sizeof(uint32_t)))
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint32_t framePeriodMs { 0 };
    BE_Get(framePeriodMs, &payload[0]);
    return { framePeriodMs };
}

std::optional<std::tuple<uint32_t, uint32_t, uint32_t>> Sensor::getFramePeriodMsAndLimits()
{
    auto result = this->send_receive(COMMAND_GET_FRAME_PERIOD_LIMITS);

    if (!result || (result->size() != (4 * sizeof(uint32_t)))) // NOTE: 4th piece of data (step size) is ignored (always 1)
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint32_t framePeriodMs { 0 };
    uint32_t framePeriodMsMin { 0 };
    uint32_t framePeriodMsMax { 0 };
    BE_Get(framePeriodMs, &payload[0]);
    BE_Get(framePeriodMsMin, &payload[sizeof(uint32_t)]);
    BE_Get(framePeriodMsMax, &payload[2 * sizeof(uint32_t)]);

    return std::make_optional(std::tuple(framePeriodMs, framePeriodMsMin, framePeriodMsMax));
}

std::optional<TofComm::ImuScaledData_T> Sensor::getImuInfo()
{
    /* This is the structure of the data bytes from the sensor.
     * It isn't identical to the oasis definition because this
     * included the data identification byte. */
    PACK_START struct ImuData_T
    {
        int8_t idByte;
        std::array<int16_t, 3> accel { 0, 0, 0 };
        int8_t accelRange {0};  
        std::array<int16_t, 3> gyro { 0, 0, 0 }; 
        int8_t gyroRange {0};  
        int16_t temperature { 0 };
        uint16_t temperatureScaling {0};  
        uint32_t timestamp;
    }PACK_END ;

    constexpr std::size_t IMU_DATA_SIZE_BYTES {sizeof(ImuData_T)};

    TofComm::ImuScaledData_T scaledData;
    int16_t data;
    int32_t data32;

    auto result = this->send_receive(COMMAND_READ_IMU_INFO);
    const auto &payload = *result;

    if (!result || (result->size() != IMU_DATA_SIZE_BYTES))
    {
        return std::nullopt;
    }

    // Lambda to convert raw accelerometer value to scaled value of milli-g.
    auto scaleAccel = [&] (int16_t data, int8_t scale) -> int16_t
    {
        float counts {32768};
        float conversion {1};

        switch(scale)
        {
            case 0:
                // 2g scale
                conversion = 2./counts;
                break;
            case 1:
                // 4g scale
                conversion = 4./counts;
                break;
            case 2:
                // 8g scale
                conversion = 8./counts;
                break;
            case 3:
                // 16g scale
                conversion = 16./counts;
                break;
        }

        // Convert from the raw value and scaled it to milli-g.
        float converted = (data * conversion) * 1000;
       
        // Take floating point and convert it to an integer.
        return (static_cast<int16_t>(converted));
    };

    int8_t accelometerRange = static_cast<int8_t>(payload[offsetof(ImuData_T, accelRange)]);

    for (int i = 0, payloadIndex = offsetof(ImuData_T, accel); i < 3; i++, payloadIndex += 2 )
    {
        BE_Get(data, &payload[payloadIndex]);
        scaledData.accelerometer_millig[i] = scaleAccel(data, accelometerRange); 
    }

    // Lambda to convert raw gyro value to scaled value of milli-degrees/second.
    auto scaleGyro = [&] (int16_t data, int8_t scale) -> int16_t
    {
        auto counts {32768};
        float conversion {1};

        switch(scale)
        {
            case 0:
                // 2000 deg/sec
                conversion = 2000./counts;
                break;
            case 1:
                // 1000 deg/sec
                conversion = 1000./counts;
                break;
            case 2:
                // 500 deg/sec
                conversion = 500./counts;
                break;
            case 3:
                // 250 deg/sec
                conversion = 250./counts;
                break;
            case 4:
                // 125 deg/sec
                conversion = 125./counts;
                break;
        }

        // Convert from the raw value and scale it to milli-deg/sec.
        float converted = (data * conversion) * 1000;

        // Take floating point and convert to an integer.
        return (static_cast<int16_t>(converted));
    };

    int8_t gyroRange = static_cast<int8_t>(payload[offsetof(ImuData_T, gyroRange)]);

    for (int i = 0, payloadIndex = offsetof(ImuData_T, gyro); i < 3; i++, payloadIndex += 2 )
    {
        BE_Get(data, &payload[payloadIndex]);
        scaledData.gyro_milliDegreesPerSecond[i] = scaleGyro(data, gyroRange); 
    }
    
    // Lambda to convert raw temperature value to scaled value of milli-degreesC.
    auto scaleTemperature = [&] (int16_t data, uint16_t scale) -> int32_t
    {
        float t { (static_cast<float>(data) / scale) * 1000 };
        
        return (static_cast<int32_t>(t));
    };

    uint16_t temperatureScaleFactor { 0 };
    BE_Get(temperatureScaleFactor, &payload[offsetof(ImuData_T, temperatureScaling)]);
 
    BE_Get(data, &payload[offsetof(ImuData_T, temperature)]);
    scaledData.temperature_milliDegreesC = scaleTemperature(data, temperatureScaleFactor);

    // IMU data timestamp
    BE_Get(data32, &payload[offsetof(ImuData_T, timestamp)]);
    scaledData.timestamp = data32;
    
    return {scaledData};
}

std::optional<uint16_t> Sensor::getIntegrationTime()
{
    auto result = this->send_receive(COMMAND_GET_INT_TIMES);

    if (!result || (result->size() != sizeof(uint16_t)))
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint16_t integrationTime { 0 };
    BE_Get(integrationTime, &payload[0]);
    return { integrationTime };
}

std::optional<std::tuple<uint16_t, uint16_t, uint16_t>> Sensor::getIntegrationTimeUsAndLimits()
{
    auto result = this->send_receive(COMMAND_GET_INTEG_TIME_LIMITS);

    if (!result || (result->size() != (4 * sizeof(uint16_t)))) // the 4th parameter is the "step size", which is not used
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint16_t integrationTime { 0 };
    uint16_t integrationTimeMin { 0 };
    uint16_t integrationTimeMax { 0 };
    BE_Get(integrationTime, &payload[0]);
    BE_Get(integrationTimeMin, &payload[sizeof(uint16_t)]);
    BE_Get(integrationTimeMax, &payload[2 * sizeof(uint16_t)]);

    return std::make_optional(std::tuple(integrationTime, integrationTimeMin, integrationTimeMax));
}

std::optional<std::tuple<std::array<std::byte, 4>, uint16_t>> Sensor::getLogIpv4Destination()
{
    auto result = this->send_receive(COMMAND_GET_UDP_LOG_SETUP);
    if(result && result->size() >= 6)
    {
        const auto& payload = *result;
        uint16_t port {};
        BE_Get(port, &payload[4]);

        auto addr = std::array<std::byte, 4>();
        std::copy_n((std::byte*)(payload.data()), addr.size(), addr.begin());

        return std::make_optional(std::make_tuple(addr, port));
    }
    return std::nullopt;

}

std::optional<uint16_t> Sensor::getMinAmplitude()
{
    auto result = this->send_receive(COMMAND_GET_MIN_AMPLITUDE);

    if (!result || (result->size() != sizeof(uint16_t)))
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint16_t minAmplitude { 0 };
    BE_Get(minAmplitude, &payload[0]);
    return { minAmplitude };
}

std::optional<std::tuple<uint16_t, uint16_t, uint16_t>> Sensor::getMinAmplitudeAndLimits()
{
    auto result = this->send_receive(COMMAND_GET_MIN_AMPLITUDE_LIMITS);

    if (!result || (result->size() != (4 * sizeof(uint16_t)))) // the 4th parameter is the "step size", which is not used
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint16_t minAmplitude { 0 };
    uint16_t minMinAmplitude { 0 };
    uint16_t maxMinAmplitude { 0 };
    BE_Get(minAmplitude, &payload[0]);
    BE_Get(minMinAmplitude, &payload[sizeof(uint16_t)]);
    BE_Get(maxMinAmplitude, &payload[2 * sizeof(uint16_t)]);

    return std::make_optional(std::tuple(minAmplitude, minMinAmplitude, maxMinAmplitude));
}

std::optional<std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>> Sensor::getModulationFreqKhzAndLimitsAndStepSize()
{   
    auto result = this->send_receive(COMMAND_GET_MOD_FREQ_LIMITS);

    if (!result || (result->size() != (4 * sizeof(uint16_t))))
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint16_t modFreq { 0 };
    uint16_t modFreqMin { 0 };
    uint16_t modFreqMax { 0 };
    uint16_t modFreqStep { 0 };
    BE_Get(modFreq, &payload[0]);
    BE_Get(modFreqMin, &payload[sizeof(uint16_t)]);
    BE_Get(modFreqMax, &payload[2 * sizeof(uint16_t)]);
    BE_Get(modFreqStep, &payload[3 * sizeof(uint16_t)]);

    return std::make_optional(std::tuple(modFreq, modFreqMin, modFreqMax, modFreqStep));
}

bool Sensor::getLensInfo(std::vector<double>& rays_x, std::vector<double>& rays_y, std::vector<double>& rays_z)
{
    std::byte REQUEST_PIXEL_RAYS {1};
    auto payload = send_receive_payload_t(&REQUEST_PIXEL_RAYS, 1);
    //Increased timeout due to larger dataset (takes extra time over Ethernet)
    auto result = this->send_receive(COMMAND_GET_LENS_INFO, payload, std::chrono::seconds(30));

    if (!result)
    {
        return false; // failed to get information
    }

    const auto& answer = *result;
    const auto size = answer.size();

    //The first byte in the answer is the message ID, so we can skip that to get to the data words.
    //The data is packed x,y,z scaled int32_t (big-endian) in row major order.
    const int16_t* begin = reinterpret_cast<const int16_t*>(answer.data()+1);
    const int16_t* end = begin + (size/sizeof(int16_t));
    const double imax = std::numeric_limits<int16_t>::max();
    const auto ray_count = std::distance(begin, end)/3;
    rays_x.reserve(ray_count);
    rays_y.reserve(ray_count);
    rays_z.reserve(ray_count);
    int16_t v;
    for( auto i = begin; i != end;)
    {
        BE_Get(v, i++);
        rays_x.push_back(v / imax);
        BE_Get(v, i++);
        rays_y.push_back(v / imax);
        BE_Get(v, i++);
        rays_z.push_back(v / imax);
    }
    return true;
}

std::optional<LensIntrinsics_t> Sensor::getLensIntrinsics()
{
    auto result = this->send_receive(COMMAND_GET_LENS_INFO, (uint8_t)0);

    if (!result || (result->size() != RAW_SENSOR_INFO_DATA_SIZE))
    {
        return std::nullopt; // failed to get information
    }
    LensIntrinsics_t li { };

    const uint8_t* rawPtr = reinterpret_cast<const uint8_t*>(result->data());
    int16_t s16 { 0 };
    // Row offset
    BE_Get(s16, rawPtr);
    li.m_rowOffset = s16 / 1.0E2;
    rawPtr += sizeof(s16);
    // Column offset
    BE_Get(s16, rawPtr);
    li.m_columnOffset = s16 / 1.0E2;
    rawPtr += sizeof(s16);
    // Row focal length
    uint32_t u32 { 0 };
    BE_Get(u32, rawPtr);
    li.m_rowFocalLength = u32 / 1.0E6;
    rawPtr += sizeof(u32);
    // Column focal length
    BE_Get(u32, rawPtr);
    li.m_columnFocalLength = u32 / 1.0E6;
    rawPtr += sizeof(u32);
    // Un-distortion coefficients
    for (size_t n = 0; n < 5; ++n)
    {
        int32_t i32 { 0 };
        BE_Get(i32, rawPtr);
        li.m_undistortionCoeffs[n] = i32 / 1.0E8;
        rawPtr += sizeof(i32);
    }
    return { li };
}

bool Sensor::getSensorInfo(TofComm::versionData_t &versionData)
{
    auto result = this->send_receive(COMMAND_READ_SENSOR_INFO);
    auto ok = bool { result };
    const auto &payload = *result;

    ok &= (payload.size() == sizeof(versionData));

    if (ok)
    {
        memcpy((void*) &versionData, (void*) payload.data(), sizeof(versionData));
    }
    return ok;
}

std::optional<std::string> Sensor::getSensorLocation()
{
    auto result = this->send_receive(COMMAND_GET_SENSOR_LOCATION);
    if (!result)
    {
        return std::nullopt; // failed to get information
    }
    else
    {
        return { std::string(reinterpret_cast<const char*>(result->data()), result->size()) };
    }
}

std::optional<std::string> Sensor::getSensorName()
{
    auto result = this->send_receive(COMMAND_GET_SENSOR_NAME);
    if (!result)
    {
        return std::nullopt; // failed to get information
    }
    else
    {
        return { std::string(reinterpret_cast<const char*>(result->data()), result->size()) };
    }
}

bool Sensor::getSensorStatus(TofComm::Sensor_Status_t &sensorStatus)
{
    auto result = this->send_receive(COMMAND_READ_SENSOR_STATUS);
    auto ok = bool { result };
    const auto &payload = *result;

    ok &= (payload.size() == sizeof(sensorStatus));

    if (ok)
    {
        memcpy((void*) &sensorStatus, (void*) payload.data(), sizeof(sensorStatus));

        sensorStatus.lastTemperature = boost::endian::big_to_native(sensorStatus.lastTemperature);
        sensorStatus.USB_Current = boost::endian::big_to_native(sensorStatus.USB_Current);
        sensorStatus.BIT_Status = boost::endian::big_to_native(sensorStatus.BIT_Status);

    }
    return ok;
}

bool Sensor::getSettings(std::string& jsonSettings)
{
    auto result = this->send_receive(COMMAND_READ_SETTINGS);

    if (!result)
    {
        return false; // failed to get information
    }

    const auto& answer = *result;
    const auto size = answer.size();

    jsonSettings = std::string(reinterpret_cast<const char*>(answer.data()+1), size-1);

    return true;
}

std::optional<std::tuple<uint16_t, uint16_t, uint16_t>> Sensor::getVledSettingAndLimits()
{
    auto result = this->send_receive(COMMAND_GET_VLED_LIMITS);

    if (!result || (result->size() != (4 * sizeof(uint16_t)))) // the 4th parameter is the "step size", which is not used
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    uint16_t vledSetting { 0 };
    uint16_t vledMax { 0 };
    uint16_t vledMin { 0 };
    BE_Get(vledSetting, &payload[0]);
    BE_Get(vledMax, &payload[sizeof(uint16_t)]);
    BE_Get(vledMin, &payload[2 * sizeof(uint16_t)]);

    return std::make_optional(std::tuple(vledSetting, vledMax, vledMin));
}

std::optional<VsmControl_T> Sensor::getVsmSettings()
{
    auto result = this->send_receive(COMMAND_GET_VSM);
    if(result && (result->size() == sizeof(VsmControl_T)))
    {
        VsmControl_T vsmControl {};
        memcpy((void*)&vsmControl, result->data(), sizeof(vsmControl));
        vsmEndianConversion(vsmControl);
        return { vsmControl };
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<uint32_t> Sensor::getVsmMaxNumberOfElements()
{
    auto result = this->send_receive(COMMAND_GET_VSM_MAX_NUMBER_ELEMENTS);
    if(result && (result->size() == sizeof(uint32_t)))
    {
        const auto& payload = *result;
        uint32_t vsmLimit { 0 };
        BE_Get(vsmLimit, &payload[0]);
        return { vsmLimit };
    }
    else
    {
        return std::nullopt;
    }
}

bool Sensor::setVsm(const VsmControl_T& vsmControl)
{
    VsmControl_T vsmPayload = vsmControl;
    vsmEndianConversion(vsmPayload);

    return this->send_receive(COMMAND_SET_VSM,
                              {reinterpret_cast<std::byte*>(&vsmPayload), sizeof(VsmControl_T)}).has_value();
}

/**
 * @brief Check if Horizontal flip is active.
 * 
 * @return std::tuple<bool, bool> - <message success, Hflip active>
 */
std::optional<bool> Sensor::isFlipHorizontallyActive()
{
    bool hIsFlipped { false };
    auto result = this->send_receive(COMMAND_GET_HORIZ_FLIP_STATE);
    if (result)
    {
        const auto& answer = *result;
        if (answer.size() > 0)
        {
            hIsFlipped = (*reinterpret_cast<const uint8_t*>(answer.data()) != 0);
            return hIsFlipped;
        }
    }
    return std::nullopt;
}

/**
 * @brief Check if Vertical flip is active.
 * 
 * @return std::tuple<bool, bool> - <message success, Vflip active>
 */
std::optional<bool> Sensor::isFlipVerticallyActive()
{
    bool vIsFlipped { false };
    auto result = this->send_receive(COMMAND_GET_VERT_FLIP_STATE);
    if (result)
    {
        const auto& answer = *result;
        if (answer.size() > 0)
        {
            vIsFlipped = (*reinterpret_cast<const uint8_t*>(answer.data()) != 0);
            return vIsFlipped;
        }
    }
    return std::nullopt;
}

std::optional<SensorControlStatus> Sensor::getSensorControlState()
{
    SensorControlStatus state{ SensorControlStatus::ERROR };
    auto result = this->send_receive(COMMAND_GET_STREAMING_STATE);
    if (result)
    {
        decltype(auto) answer = result.value();
        if (answer.size() > 0)
        {
            const uint8_t data = (*reinterpret_cast<const uint8_t*>(answer.data()));
            state = static_cast<SensorControlStatus>(data);
            return state;
        }
    }
    return std::nullopt; 
}

void Sensor::jumpToBootloader()
{
    this->send_receive(COMMAND_JUMP_TO_BOOLOADER);
}


void Sensor::jumpToBootloader(uint16_t token)
{
    this->send_receive(COMMAND_JUMP_TO_BOOLOADER, token);
}

/**
 * @brief Enable binning if *either* value is true.
 * @note we do not support binning individual axes. This method is left in the 
 * interface for backward compatibility.
 * 
 * @param vertical 
 * @param horizontal 
 * @return true Success
 * @return false Failure.
 */
bool Sensor::setBinning(const bool vertical, const bool horizontal)
{
    bool enable = (vertical || horizontal);
    return setBinning(enable);
}

bool Sensor::setBinning(const bool binning)
{
    uint8_t byte = (binning) ? 3 : 0;

    return bool{this->send_receive(COMMAND_SET_BINNING, byte)};
}

std::optional<uint8_t> Sensor::getBinning()
{
    auto result = this->send_receive(COMMAND_GET_BINNING);

    if (!result)
    {
        return std::nullopt;
    }
    const auto &payload = *result;

    uint8_t binning = 0;
    BE_Get(binning, &payload[0]);

    return uint8_t{binning};
}

bool Sensor::setFlipHorizontally(bool flip)
{
    const uint8_t data = (flip ? 1 : 0);
    return bool{this->send_receive(COMMAND_SET_HORIZ_FLIP_STATE, data)};
}

bool Sensor::setFlipVertically(bool flip)
{
    const uint8_t data = (flip ? 1 : 0);
    return bool{this->send_receive(COMMAND_SET_VERT_FLIP_STATE, data)};
}

bool Sensor::setFramePeriodMs(uint32_t periodMs)
{
    uint32_t params[] = {native_to_big(periodMs)};
    return this->send_receive(COMMAND_SET_FRAME_PERIOD_MS, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setHdr(bool enable, bool useSpatial)
{
    uint8_t data = ((useSpatial ? 1 : 0) << 1) | (enable ? 1: 0);
    return bool{this->send_receive(COMMAND_SET_HDR, data)};
}

std::optional<HdrSettings_T> Sensor::getHdrSettings()
{
    HdrSettings_T settings {};
    auto result = this->send_receive(COMMAND_GET_HDR);
    if (result)
    {
        decltype(auto) answer = result.value();
        if (answer.size() > 0)
        {
            const uint8_t data = (*reinterpret_cast<const uint8_t*>(answer.data()));
            if(data & 1) {
                settings.enabled = true;
            }
            if(data& (1<<1)) {
                settings.mode = HdrMode_e::SPATIAL;
            }
            else {
                settings.mode = HdrMode_e::TEMPORAL;
            }
            return settings;
        }
    }
    return std::nullopt; 
}

bool Sensor::setIntegrationTime(uint16_t low)
{
    uint16_t params[] = {native_to_big(low)};
    return this->send_receive(COMMAND_SET_INT_TIMES, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setIntegrationTimes(uint16_t low, uint16_t mid, uint16_t high)
{
    uint16_t params[] = {native_to_big(low), native_to_big(mid), native_to_big(high)};
    return this->send_receive(COMMAND_SET_INT_TIMES, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setIPv4Settings(const std::array<std::byte, 4>& adrs, const std::array<std::byte, 4>& mask, const std::array<std::byte, 4>& gateway)
{
    std::vector<std::byte> ipData { std::begin(adrs), std::end(adrs) };
    std::copy(std::begin(mask), std::end(mask), std::back_inserter(ipData));
    std::copy(std::begin(gateway), std::end(gateway), std::back_inserter(ipData));
    return this->send_receive(COMMAND_SET_CAMERA_IP_SETTINGS, {ipData.data(), ipData.size()}).has_value();
}

bool Sensor::setLogIPv4Destination(const std::array<std::byte, 4>& adrs, const uint16_t port)
{
    std::vector<std::byte> ipData { std::begin(adrs), std::end(adrs) };
    ipData.push_back((std::byte)(port >> 8));
    ipData.push_back((std::byte)(port));
    return this->send_receive(COMMAND_SETUP_UDP_LOG, {ipData.data(), ipData.size()}).has_value();
}

bool Sensor::setMinAmplitude(uint16_t minAmplitude)
{
    return this->send_receive(COMMAND_SET_MIN_AMPLITUDE, minAmplitude).has_value();
}

bool Sensor::setModulation(const uint16_t modFreqkHz)
{
    return this->send_receive(COMMAND_SET_MODULATION, modFreqkHz).has_value();
}

std::optional<uint16_t> Sensor::getModulation(void)
{
    auto result = this->send_receive(COMMAND_GET_MODULATION);

    if (!result)
    {
        return std::nullopt;
    }
    const auto &payload = *result;

    uint16_t modFreqkHz = 0;
    BE_Get(modFreqkHz, &payload[0]);

    return {modFreqkHz};
}

bool Sensor::setOffset(int16_t offset)
{
    return this->send_receive(COMMAND_SET_OFFSET, offset).has_value();
}

bool Sensor::setSensorLocation(std::string location)
{
    return this->send_receive(COMMAND_SET_SENSOR_LOCATION, {(std::byte*)location.data(), location.size()}).has_value();
}

bool Sensor::setSensorName(std::string name)
{
    return this->send_receive(COMMAND_SET_SENSOR_NAME, {(std::byte*)name.data(), name.length()}).has_value();
}

bool Sensor:: getIPv4Settings(std::array<std::byte, 4>& adrs, std::array<std::byte, 4>& mask, std::array<std::byte, 4>& gateway)
{
    auto result = this->send_receive(COMMAND_GET_CAMERA_IP_SETTINGS);
    if (result && (result->size() == 12))
    {
        auto it = std::begin(*result);
        std::copy(it, it + 4, std::begin(adrs));
        it += 4;
        std::copy(it, it + 4, std::begin(mask));
        it += 4;
        std::copy(it, it + 4, std::begin(gateway));
        return true;
    }
    else
    {
        return false;
    }
}


bool Sensor::setIPMeasurementEndpoint(std::array<std::byte,4> address, uint16_t dataPort)
{
    std::byte payload[address.size() + sizeof(dataPort)];
    std::copy(address.begin(), address.end(), std::begin(payload));
    BE_Put(payload + address.size(), dataPort);
    return this->send_receive(COMMAND_SET_DATA_IP_ADDRESS, {payload, sizeof(payload)}).has_value();
}


std::optional<std::tuple<std::array<std::byte, 4>, uint16_t>> Sensor::getIPMeasurementEndpoint()
{
    auto result = this->send_receive(COMMAND_GET_DATA_IP_ADDRESS);
    if(result && result->size() >= 6)
    {
        const auto& payload = *result;
        uint16_t port {};
        uint32_t address {};
        BE_Get(address, payload.data());
        BE_Get(port, &payload[4]);

        auto addr = std::array<std::byte, 4>();
        std::copy_n((std::byte*)(&address), addr.size(), addr.begin());

        return std::make_optional(std::make_tuple(addr, port));
    }
    return std::nullopt;
}

bool Sensor::stopStream()
{
    this->pimpl->stream_via_polling_ = false;
    this->pimpl->measurement_timer_.cancel();
    return this->send_receive(COMMAND_STOP_STREAM).has_value();
}

bool Sensor::storeSettings()
{
    return this->send_receive(COMMAND_STORE_SETTINGS).has_value();
}

bool Sensor::streamDCS()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DCS);
}

bool Sensor::streamDCSAmbient()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DCS_AMBIENT);
}

bool Sensor::streamDCSDiffAmbient()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DCS_DIFF_AMBIENT);
}

bool Sensor::streamDistance()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DISTANCE);
}

bool Sensor::streamDistanceAmplitude()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DIST_AND_AMP);
}

void Sensor::subscribeMeasurement(std::function<void (std::shared_ptr<Measurement_T>)> onMeasurementReady)
{
    //Hook the callback so that if stream via polling is enabled we can request
    // the next measurement just before before delivering to the caller
    auto hook = [this, onMeasurementReady](std::shared_ptr<Measurement_T> measurement) -> void
    {
        if (pimpl->stream_via_polling_)
        {
            pimpl->request_next_measurement();
        }
        
        //Now pass the data on to our client
        if (onMeasurementReady)
        {
            onMeasurementReady(measurement);
        }
    };
    std::lock_guard<std::mutex> guard {pimpl->measurementReadyMutex};
    pimpl->measurementReady = hook;
}

/* #########################################################################
 *
 * Sensor::Impl Implementation
 *
 * ######################################################################### */

void Sensor::Impl::init()
{
    auto server_f = [this]() 
        {
            try
            { 
                ioService.run(); 
            }
            catch(std::exception& e)
            {
                std::cerr << "caught exception in background thread: " << e.what() << std::endl;
            }
        };

    serverThread_ = std::thread{ server_f };

    auto f = [&](const std::vector<std::byte>& p)
        {
            on_measurement_ready_t usr_callback;
            { std::lock_guard<std::mutex> guard {measurementReadyMutex};
                usr_callback = measurementReady;
            }

            if (usr_callback)
            {
                auto new_measurement = tofcore::create_measurement(p);
                try {
                    usr_callback(new_measurement);
                } catch(std::exception& err) {
                    std::string what = err.what();
                    std::cerr << "caught unhandled exception from user callback: \n\t" << what << std::endl;
                }
            }
        };
    connection->subscribe(f);
}


Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, const std::vector<Sensor::send_receive_payload_t>& payload,
        std::chrono::steady_clock::duration timeout /*= 5s*/) const
{
    auto result = pimpl->connection->send_receive(command, payload, timeout);
    if(!result)
    {
        return std::nullopt;
    }
    return {*result};
}


Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, const send_receive_payload_t& payload,
        std::chrono::steady_clock::duration timeout /*= 5s*/) const
{
    const std::vector<Sensor::send_receive_payload_t> one{payload};
    return this->send_receive(command, one, timeout);
}


Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command) const
{
    return this->send_receive(command, send_receive_payload_t{});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, uint8_t value) const
{
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, int8_t value) const
{
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, uint16_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, int16_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, uint32_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, int32_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

} //end namespace tofcore
