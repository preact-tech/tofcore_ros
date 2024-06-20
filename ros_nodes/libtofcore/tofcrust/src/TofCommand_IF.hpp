#ifndef TOFCOMMAND_IF_HPP
#define TOFCOMMAND_IF_HPP
/**
 * @file TofCommand_IF.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Declares constants and types associated with the TOF communication interface.
 */
#include <cstdint>

#if !defined(PACKED)
#    define PACKED __attribute__((__packed__))
#endif

namespace TofComm
{
constexpr uint16_t COMMAND_SET_DLL_STEP             = 0x1D;

constexpr uint16_t COMMAND_WRITE_REGISTER           = 0x2A;
constexpr uint16_t COMMAND_READ_REGISTER            = 0x2B;
    constexpr size_t  READ_REGISTER_DATA_OFFSET         = 1;
    constexpr size_t  READ_REGISTER_DATA_SIZE           = 2; // Account for data type in return
constexpr uint16_t COMMAND_GET_THERMAL_PROTECTION_SETTINGS  = 0x2C;
constexpr uint16_t COMMAND_SET_THERMAL_PROTECTION_SETTINGS  = 0x2D;

constexpr uint16_t COMMAND_SET_LENS_INFO            = 0x35; ///<Set the lens information (intrinsics data)
    constexpr size_t RAW_SENSOR_INFO_DATA_SIZE          = 32;

constexpr uint16_t COMMAND_SEQU_GET_VERSION         = 0x38;
constexpr uint16_t COMMAND_SEQU_SET_VERSION         = 0x39;
constexpr uint16_t COMMAND_SEQU_VERSION_IS_SUPPORTED = 0x3A;

constexpr uint16_t COMMAND_SDRAM_TEST               = 0x6E;

constexpr uint16_t COMMAND_SET_CPU_SERIAL           = 0x73;
constexpr uint16_t COMMAND_SET_DEV_SERIAL           = 0x74;
constexpr uint16_t COMMAND_SET_MODEL_NAME           = 0x75;
constexpr uint16_t COMMAND_SET_TEST_STATION_DATA    = 0x76;
constexpr uint16_t COMMAND_GET_TEST_STATION_DATA    = 0x77;

/*
* Illuminator board commands
*/
constexpr uint16_t COMMAND_SET_VLED_ENABLES         = 0x80;
constexpr uint16_t COMMAND_GET_VLED_ENABLES         = 0x81;
    constexpr size_t  VLED_ENABLES_DATA_OFFSET          = 1;
    constexpr size_t  VLED_ENABLES_DATA_SIZE            = 2;
constexpr uint16_t COMMAND_SET_VLED                 = 0x82;
constexpr uint16_t COMMAND_GET_VLED                 = 0x83;
    constexpr size_t VLED_DATA_OFFSET                   = 0;
    constexpr size_t VLED_DATA_SIZE                     = 2;
constexpr uint16_t COMMAND_GET_IB_5V                = 0x84;
    constexpr size_t IB_5V_DATA_OFFSET                  = 0;
    constexpr size_t IB_5V_DATA_SIZE                    = 2;
constexpr uint16_t COMMAND_GET_IB_TEMPERATURE       = 0x85;
    constexpr size_t IB_TEMPERATURE_DATA_OFFSET         = 0;
    constexpr size_t IB_TEMPERATURE_DATA_SIZE           = 4;
constexpr uint16_t COMMAND_GET_IB_PD                = 0x86;
    constexpr size_t IB_PD_DATA_OFFSET                  = 0;
    constexpr size_t IB_PD_DATA_SIZE                    = 2;
constexpr uint16_t COMMAND_SET_RGB                  = 0x88;
    constexpr size_t RGB_COLOR_OFFSET                   = 0;
    constexpr size_t RGB_BLINK_OFFSET                   = 1;
    constexpr size_t RGB_SET_DATA_SIZE                  = 3;
constexpr uint16_t COMMAND_GET_RGB                  = 0x89;
    constexpr size_t RGB_DATA_OFFSET                    = 0;
    constexpr size_t RGB_DATA_SIZE                      = 1;
constexpr uint16_t COMMAND_SET_IB_TEST_8BIT         = 0x8A;
constexpr uint16_t COMMAND_SET_IB_TEST_16BIT        = 0x8B;
constexpr uint16_t COMMAND_GET_IB_TEST_8BIT         = 0x8C;
    constexpr size_t IB_TEST_8BIT_OFFSET                = 0;
    constexpr size_t IB_TEST_8BIT_SIZE                  = 1;
constexpr uint16_t COMMAND_GET_IB_TEST_16BIT        = 0x8D;
    constexpr size_t IB_TEST_16BIT_OFFSET               = 0;
    constexpr size_t IB_TEST_16BIT_SIZE                 = 2;
constexpr uint16_t COMMAND_SET_SERIAL_NUM           = 0x8F;
constexpr size_t ILLMN_SERIAL_STRING_SIZE               = 16;

// Sensor Limits
const uint16_t COMMAND_SET_INTEG_TIME_LIMITS        = 0xB1; ///<Set integration time min/max
const uint16_t COMMAND_SET_MOD_FREQ_LIMITS          = 0xB3; ///<Set mod freq min/max/step
const uint16_t COMMAND_SET_FRAME_PERIOD_LIMITS      = 0xB5; ///<Set frame rate min/max
const uint16_t COMMAND_SET_MIN_AMPLITUDE_LIMITS     = 0xB9; ///<Set minimum amplitude min/max
const uint16_t COMMAND_SET_VLED_LIMITS              = 0xBB; ///<Set the max/min VLED allowed for this unit

constexpr uint16_t COMMAND_STORAGE_GET_METADATA     = 0xCC;
constexpr uint16_t COMMAND_STORAGE_WRITE            = 0xCD;
constexpr uint16_t COMMAND_STORAGE_READ             = 0xCE;
constexpr uint16_t COMMAND_FACTORY_MODE             = 0xCF;

constexpr uint16_t COMMAND_GET_LOG_SETTINGS         = 0xE0;
constexpr uint16_t COMMAND_SET_LOG_SETTINGS         = 0xE1;

} //end namespace TofComm

#endif
