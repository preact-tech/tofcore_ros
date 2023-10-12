#include "tofcrust/tof_sensor.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

constexpr auto VLED_VOLTAGE_DOCSTRING = 
  "Voltage applied to LEDs during capture\n"
  "\n"
  "When property is the voltage set point is changed for the next capture.\n"
  "when property is read actual voltage is read via ADC";

////////////////////////////////////////////////////////////////////////
///Helper methods used to provided Pythonic methods to the Sensor class.

static auto setDllStep(tofcrust::Sensor &s, bool enableDll, uint16_t courseStep, uint16_t fineStep, uint16_t finestStep)
{
    if(!s.setDllStep(enableDll, courseStep, fineStep, finestStep))
    {
        throw std::runtime_error("An error occured while setting DLL manual enable state and DLL step");
    }

    return true;
}

static auto readRegister(tofcrust::Sensor &s, uint8_t regAddress)
{        
    //Use static and a lambda to create the RegRead namedtuple type only once. 
    static auto RegRead_type = []() {
        auto namedTuple_attr = pybind11::module::import("collections").attr("namedtuple");
        py::list fields;
        fields.append("address");
        fields.append("data");
        return namedTuple_attr("RegRead", fields);
    }();

    uint8_t regData;
    bool result = false;
    {
        py::gil_scoped_release gsr;
        result = s.readRegister(regAddress, regData);
    }
    if (!result)
    {
        throw std::runtime_error("An error occcured trying to read a sensor register");
    }

    return RegRead_type(regAddress, regData);
}

static void writeRegister(tofcrust::Sensor &s, uint8_t regAddress, uint8_t regData){

    if (!s.writeRegister(regAddress, regData)){

        throw std::runtime_error("An error occcured trying to write a sensor register");
    }
}

static auto setVledEnables(tofcrust::Sensor &s, uint8_t vledEnables){

    if (!s.setVledEnables(vledEnables)){

        throw std::runtime_error("An error occcured trying to VLED enables");
    }

    return true;
}

static auto getVledEnables(tofcrust::Sensor &s)
{
    uint8_t vledEnables;
    if (!s.getVledEnables(vledEnables))
    {
        throw std::runtime_error("An error occcured trying to read state of VLED enables");
    }

    return vledEnables;
}

static auto getSequencerVersion(tofcrust::Sensor &s)
{
    py::gil_scoped_release gsr;

    const auto result { s.sequencerGetVersion() };
    if (!result)
    {
        throw std::runtime_error("An error occcured trying to read the sequencer version");
    }
    return *result;

}

static void setSequencerVersion(tofcrust::Sensor &s, uint16_t version)
{
    py::gil_scoped_release gsr;

    const auto ok = s.sequencerSetVersion(version);
    if(!ok)
    {
        throw std::runtime_error("An error occcured setting sequencer version");
    }
}


static auto sequencerIsVersionSupported(const tofcrust::Sensor &s, uint16_t version)
{
    const auto result = s.sequencerIsVersionSupported(version);
    if(!result)
    {
        throw std::runtime_error("An error occcured querying sequencer support");
    }
    return *result;
}

/// @brief Helper function to set the Vled voltage on a sensor
/// @param vled_v value in volts to send.
static void setVled(tofcrust::Sensor &sensor, float vled_v)
{
    py::gil_scoped_release gsr;

    //covert to mV prior to sending
    auto vled_mv = static_cast<uint16_t>(std::round(vled_v * 1000));
    if(!sensor.setVled(vled_mv))
    {
        throw std::runtime_error("An error occcured attempting to set VLED voltage");
    }
}

/// @brief Helper function to read the active Vled voltage
/// @return the sensed voltage in volts
static auto readVled(tofcrust::Sensor &sensor)
{
    py::gil_scoped_release gsr;

    uint16_t mV {0};
    if (sensor.getVled(mV))
    {
        //convert from mV to V
        return mV / 1000.0;
    }
    else
    {
        throw std::runtime_error("An error occcured attempting to read VLED voltage");
    }
}


/// @brief Helper function to set the sensor lens intrinsics
/// @param offset_x lens center X offset from the center pixel of the sensor array
/// @param offset_y lens center Y offset from the center pixel of the sensor array
/// @param focal_len_x X focal length of the lens
/// @param focal_len_y Y focal length of the lens
/// @param undistortion_coeff 5 undistortion coefficents
/// @return true on success.
static void setLensInfo(tofcrust::Sensor& sensor, double offset_x, double offset_y, double focal_len_x, double focal_len_y, std::array<double, 5> undistortion_coeff)
{
    if(!sensor.setLensInfo(offset_x, offset_y, focal_len_x, focal_len_y, undistortion_coeff))
    {
        throw std::runtime_error("An error occcured attempting to set the lens info intrinsics");
    }
}


/// @brief Helper function to set factory mode
/// @param enable. true enables factory mode
static auto setFactoryMode(tofcrust::Sensor &sensor, bool enable)
{
    if (!sensor.setFactoryMode(enable))
    {
        throw std::runtime_error("An error occcured attempting to set factory mode");
    }
}


/// @brief Helper function to request SDRAM test
static auto sdramTestRequest(tofcrust::Sensor &sensor)
{
    if (!sensor.sdramTestRequest())
    {
        throw std::runtime_error("An error occcured attempting to request sdram test");
    }

    constexpr uint16_t FULL_RESET_TOKEN { 0x0155 };
    sensor.jumpToBootloader(FULL_RESET_TOKEN);
}

static bool setSerialNumberCpuBoard(tofcrust::Sensor &sensor, const char *serial)
{
    char writeBuf[TofComm::SERIAL_NUMBER_SIZE] {};
    strncpy(writeBuf, serial, sizeof(writeBuf));
    return sensor.setSerialCpuBoard(writeBuf);
}

static bool setSerialNumberDevice(tofcrust::Sensor &sensor, const char *serial)
{
    char writeBuf[TofComm::SERIAL_NUMBER_SIZE] {};
    strncpy(writeBuf, serial, sizeof(writeBuf));
    return sensor.setSerialDevice(writeBuf);
}

static bool setSerialNumberIlluminator(tofcrust::Sensor &sensor, const char *serial)
{
    return sensor.setIbSerial(serial);
}

static auto testStationDataGet(tofcrust::Sensor &sensor)
{
    return sensor.getTestStationData();
}

static bool testStationDataSet(tofcrust::Sensor &sensor, const char *tsData)
{
    return sensor.setTestStationData(tsData);
}

static auto logSettingsGet(tofcrust::Sensor &s)
{
    auto result = s.getLogSettings();
    if(!result)
    {
        throw std::runtime_error("An error ocurred trying to read the log settings.");
    }
    return *result;
}

static void logSettingsSet(tofcrust::Sensor &s, const char *globalEnables, const char *categoryLevels)
{
    std::optional<std::string> g = std::nullopt;
    std::optional<std::string> c = std::nullopt;
    if ((globalEnables  != nullptr) && (*globalEnables  != '\0')) g = globalEnables;
    if ((categoryLevels != nullptr) && (*categoryLevels != '\0')) c = categoryLevels;

    if(!s.setLogSettings(g, c))
    {
        throw std::runtime_error("An error ocurred trying to set the log settings.");
    }
}

////////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(pytofcrust, m) {
    //import pytofcore to gain acceses to base class and utility types.
    py::module_::import("pytofcore");

    m.doc() = "Sensor object that represents a connect to a TOF depth sensor (production/engineering version)";

    //import the DataType and Measurement type from pytofcore. 
    m.attr("DataType") = py::module_::import("pytofcore").attr("DataType");
    m.attr("Measurement") = py::module_::import("pytofcore").attr("Measurement");

    //pytofcrust.Sensor class is just like pytofcore.Sensor but with more ... 
    py::class_<tofcrust::Sensor, tofcore::Sensor>(m, "Sensor")
        .def(py::init<const std::string&>(), py::arg("uri")=tofcore::DEFAULT_URI)
        .def(py::init<uint16_t, const std::string&, uint32_t>(), py::arg("protocol_version")=tofcore::DEFAULT_PROTOCOL_VERSION, py::arg("port_name")=tofcore::DEFAULT_PORT_NAME, py::arg("baud_rate")=tofcore::DEFAULT_BAUD_RATE)
        .def_property("sequencer_version", &getSequencerVersion, &setSequencerVersion, "Get or set the sequencer version used by imaging chip. A sensor reboot is required for a change to take effect")
        .def("is_sequencer_version_supported", &sequencerIsVersionSupported, "Check if a specific sequencer version is supported by the sensor", py::call_guard<py::gil_scoped_release>())
        .def("set_dll_step", &setDllStep, "Set the DLL (delayed lock loop) step of the sensor", py::arg("enable_dll"), py::arg("coarse_step"), py::arg("fine_step"), py::arg("finest_step"), py::call_guard<py::gil_scoped_release>())
        .def("read_sensor_register", &readRegister, "Read sensor register. Returns byte data from register", py::arg("reg_address"))
        .def("write_sensor_register", &writeRegister, "Write sensor register. Provide register address and data to be writen", py::arg("reg_address"), py::arg("reg_data"), py::call_guard<py::gil_scoped_release>())
        .def("set_vled_enables", &setVledEnables, "Set the individual VLED enables", py::arg("vled_enables"), py::call_guard<py::gil_scoped_release>())
        .def("get_vled_enables", &getVledEnables, "Get the state of individual VLED enables", py::call_guard<py::gil_scoped_release>())
        .def("set_lens_info", &setLensInfo, "Set the lens intrinsic values", py::call_guard<py::gil_scoped_release>())
        .def_property("vled_voltage", &readVled, &setVled, VLED_VOLTAGE_DOCSTRING, py::call_guard<py::gil_scoped_release>())
        .def("set_factory_mode", &setFactoryMode, "Set the sensor factory mode", py::arg("enable"), py::call_guard<py::gil_scoped_release>())
        .def("sdram_test_request", &sdramTestRequest, "Request sdram test. A sensor reboot is required to execute test", py::call_guard<py::gil_scoped_release>())
        .def("set_cpu_board_serial", &setSerialNumberCpuBoard, "Set the serial number of the main PCBA (CPU Board), and store it in persistent memory.", py::call_guard<py::gil_scoped_release>())
        .def("set_device_serial", &setSerialNumberDevice, "Set the serial number of the device and store in persistent memory.", py::call_guard<py::gil_scoped_release>())
        .def("set_illuminator_serial", &setSerialNumberIlluminator, "Set the serial number of the Illuminator Board and store in IB persistent memory", py::call_guard<py::gil_scoped_release>())
        .def("get_test_station_data", &testStationDataGet, "Returns a string that holds production test station information of the device.", py::call_guard<py::gil_scoped_release>())
        .def("set_test_station_data", &testStationDataSet, "Store the production test station information on the device as a string.", py::arg("test_data"), py::call_guard<py::gil_scoped_release>())
        .def("get_log_settings", &logSettingsGet, "Returns a string holding the current log settings of the sensor.", py::call_guard<py::gil_scoped_release>())
        .def("set_log_settings", &logSettingsSet, "Set the current log settings of the sensor.", py::arg("global_enables"), py::arg("category_levels"), py::call_guard<py::gil_scoped_release>())
        ;

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
