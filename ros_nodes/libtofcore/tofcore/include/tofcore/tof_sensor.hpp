#ifndef __TOFCORE_T10_SENSOR_H__
#define __TOFCORE_T10_SENSOR_H__
/**
 * @file tof_sensor.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * API for libtofcore control
 */
#include "CommandTypes.hpp"
#include "Measurement_T.hpp"
#include "device_discovery.hpp"
#include "span.hpp"
#include "connection.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <tuple>
#include <optional>

namespace tofcore
{

constexpr const char*   DEFAULT_URI                 { "" }; 
constexpr uint32_t      DEFAULT_BAUD_RATE           { 115200 };
constexpr const char*   DEFAULT_PORT_NAME           { "" }; 

struct LensIntrinsics_t
{
    double m_rowOffset { 0.0 };
    double m_columnOffset { 0.0 };
    double m_rowFocalLength { 0.0 };
    double m_columnFocalLength { 0.0 };
    std::array<double, 5> m_undistortionCoeffs { 0.0, 0.0, 0.0, 0.0, 0.0 };
};

enum class SensorControlStatus : uint8_t
{
    IDLE,               ///< No ToF activity
    CAPTURE,            ///< Capturing data internally
    SEND,               ///< Sending a single result
    STREAM,             ///< Continually streaming data
    OVERTEMPERATURE,    ///< Disabled due to temperature
    ERROR,              ///< ToF unavailable due to an internal error
};

class Sensor
{
public:
    /// @brief Connect to a ToF sensor using the supplied uri string to locate the device and configure
    ///  the connection.
    /// @param uri The following uri schemes/formats are supported
    ///   - If uri is empty a scan for connected devices is done and a connection is made to the first device found.
    ///   - tofserial: Specifies a serial type connection (i.e. UART, virtual USB COM, or emulated pts device).
    ///                The connection baudrate can be specified as an argument.
    ///                Examples: tofserial:/dev/ttyACM0?baudrate=19200  (Linux only)
    ///                          tofserial:COM1    (Windows only)
    ///   - tofnet: Specifies a IP network type connection (i.e. Ethernet address or hostname and optional port).
    ///                Examples:  tofnet://10.10.31.180:50660
    ///                           tofnet://localhost  (e.g. to connect to emulator)
    ///
    ///   If the uri scheme is not provided then some attempt is made to deduce the scheme based on
    ///   uri content.
    Sensor(const std::string& uri);

    /// @brief Old style constructor for typically used for connecting over serial type devices. 
    ///   (Note: this constructor will accept any URI.)
    Sensor(const std::string &portName, uint32_t baudrate);

    Sensor(std::unique_ptr<Connection_T> connection);

    typedef std::function<void (std::shared_ptr<Measurement_T>)> on_measurement_ready_t;

    ~Sensor();

    std::optional<uint16_t> getIntegrationTime();
    bool getLensInfo(std::vector<double> &rays_x, std::vector<double> &rays_y, std::vector<double> &rays_z);
    std::optional<LensIntrinsics_t> getLensIntrinsics();
    std::optional<std::tuple<std::array<std::byte, 4>, uint16_t>> getIPMeasurementEndpoint();
    bool getIPv4Settings(std::array<std::byte, 4>& adrs, std::array<std::byte, 4>& mask, std::array<std::byte, 4>& gateway);
    bool getSensorInfo(TofComm::versionData_t &versionData);
    std::optional<std::string> getSensorLocation();
    std::optional<std::string> getSensorName();
    bool getSensorStatus(TofComm::Sensor_Status_t &sensorStatus);
    bool getSettings(std::string& jsonSettings);
    std::optional<TofComm::VsmControl_T> getVsmSettings();


    std::optional<SensorControlStatus> getSensorControlState();

    std::optional<bool> isFlipHorizontallyActive();
    std::optional<bool> isFlipVerticallyActive();

    void jumpToBootloader();
    void jumpToBootloader(uint16_t token);

    bool setBinning(const bool vertical, const bool horizontal);
    bool setFlipHorizontally(bool flip);
    bool setFlipVertically(bool flip);
    bool setIntegrationTime(uint16_t);
    bool setIntegrationTimes(uint16_t, uint16_t, uint16_t);
    bool setIPMeasurementEndpoint(std::array<std::byte,4> address, uint16_t port);
    bool setIPv4Settings(const std::array<std::byte, 4>& adrs, const std::array<std::byte, 4>& mask, const std::array<std::byte, 4>& gateway);
    bool setMinAmplitude(uint16_t minAmplitude);
    bool setModulation(uint16_t modFreqkHz);
    std::optional<uint16_t> getModulation();
    bool setOffset(int16_t offset);
    bool setSensorLocation(std::string location);
    bool setSensorName(std::string name);
    bool setVsm(const TofComm::VsmControl_T& vsmControl);

    bool stopStream();
    bool storeSettings();

    bool streamDCS();
    bool streamDistance();
    bool streamDistanceAmplitude();
    bool streamDCSAmbient();

    void subscribeMeasurement(on_measurement_ready_t);

    typedef std::optional<std::vector<std::byte>> send_receive_result_t;
    typedef tcb::span<std::byte> send_receive_payload_t;

    send_receive_result_t send_receive(const uint16_t command, const send_receive_payload_t& payload,
                                       std::chrono::steady_clock::duration timeout = std::chrono::seconds(5)) const;

protected:

    send_receive_result_t send_receive(const uint16_t command, const std::vector<send_receive_payload_t>& payload, 
                                       std::chrono::steady_clock::duration timeout = std::chrono::seconds(5)) const;
    send_receive_result_t send_receive(const uint16_t command) const;
    send_receive_result_t send_receive(const uint16_t command, uint32_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int32_t value) const;
    send_receive_result_t send_receive(const uint16_t command, uint16_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int16_t value) const;
    send_receive_result_t send_receive(const uint16_t command, uint8_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int8_t value) const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} //end namespace tofcore

#endif
