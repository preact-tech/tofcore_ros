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
constexpr uint16_t COMMAND_SET_INT_TIMES            = 0x01;
constexpr uint16_t COMMAND_GET_DIST_AND_AMP         = 0x02;
constexpr uint16_t COMMAND_GET_DISTANCE             = 0x03;
constexpr uint16_t COMMAND_GET_DCS_DIFF_AMBIENT     = 0x04;
constexpr uint16_t COMMAND_STOP_STREAM              = 0x06;
constexpr uint16_t COMMAND_GET_DCS                  = 0x07;
constexpr uint16_t COMMAND_GET_INT_TIMES            = 0x08;
constexpr uint16_t COMMAND_GET_STREAMING_STATE      = 0x09;
constexpr uint16_t COMMAND_GET_DCS_AMBIENT          = 0x0B;
    constexpr std::uint8_t SINGLE_MEASUREMENT       = 0;
    constexpr std::uint8_t CONTINUOUS_MEASUREMENT   = 1;

constexpr uint16_t COMMAND_SET_FRAME_PERIOD_MS      = 0x10;
constexpr uint16_t COMMAND_GET_FRAME_PERIOD_MS      = 0x11;

constexpr uint16_t COMMAND_SET_OFFSET               = 0x14;
constexpr uint16_t COMMAND_SET_MIN_AMPLITUDE        = 0x15;
constexpr uint16_t COMMAND_GET_MIN_AMPLITUDE        = 0x16;

constexpr uint16_t COMMAND_SET_MODULATION           = 0x17;
constexpr uint16_t COMMAND_SET_BINNING              = 0x18;
constexpr uint16_t COMMAND_SET_HDR                  = 0x19;
constexpr uint16_t COMMAND_GET_HDR                  = 0x1A;

constexpr uint16_t COMMAND_SET_VSM                  = 0x1B;
constexpr uint16_t COMMAND_GET_VSM                  = 0x1C;

constexpr uint16_t COMMAND_GET_BINNING              = 0X20;

constexpr uint16_t COMMAND_GET_DATA_IP_ADDRESS      = 0x25;
constexpr uint16_t COMMAND_SET_DATA_IP_ADDRESS      = 0x26;

constexpr uint16_t COMMAND_SET_CAMERA_IP_SETTINGS   = 0x28;
constexpr uint16_t COMMAND_GET_CAMERA_IP_SETTINGS   = 0x29;

constexpr uint16_t COMMAND_GET_MODULATION           = 0x2F;

constexpr uint16_t COMMAND_GET_HORIZ_FLIP_STATE     = 0x30;
constexpr uint16_t COMMAND_GET_VERT_FLIP_STATE      = 0x31;

constexpr uint16_t COMMAND_SET_HORIZ_FLIP_STATE     = 0x33;

constexpr uint16_t COMMAND_SET_VERT_FLIP_STATE      = 0x36;

constexpr uint16_t COMMAND_GET_LENS_INFO            = 0x50;
    constexpr size_t RAW_SENSOR_INFO_DATA_SIZE          = 32;

const uint16_t COMMAND_SETUP_UDP_LOG                = 0x6C;
const uint16_t COMMAND_GET_UDP_LOG_SETUP            = 0x6D;

constexpr uint16_t COMMAND_JUMP_TO_BOOLOADER        = 0x6F;

constexpr uint16_t COMMAND_STORE_SETTINGS           = 0x71;
constexpr uint16_t COMMAND_READ_SETTINGS            = 0x72;

constexpr uint16_t COMMAND_SET_SENSOR_NAME          = 0x78;
constexpr uint16_t COMMAND_GET_SENSOR_NAME          = 0x79;
constexpr uint16_t COMMAND_SET_SENSOR_LOCATION      = 0x7A;
constexpr uint16_t COMMAND_GET_SENSOR_LOCATION      = 0x7B;

// Sensor Limits
const uint16_t COMMAND_GET_INTEG_TIME_LIMITS        = 0xB0; ///<Get integration time current/min/max
const uint16_t COMMAND_GET_MOD_FREQ_LIMITS          = 0xB2; ///<Get mod freq current/min/max/step
const uint16_t COMMAND_GET_FRAME_PERIOD_LIMITS      = 0xB4; ///<Get frame period current/min/max
const uint16_t COMMAND_GET_AVAILABLE_FEATURES       = 0xB6; ///<Get list of available features
const uint16_t COMMAND_GET_VSM_MAX_NUMBER_ELEMENTS  = 0xB7; ///<Get max number of VSM elements
const uint16_t COMMAND_GET_MIN_AMPLITUDE_LIMITS     = 0xB8; ///<Get minAmplitude current/min/max
const uint16_t COMMAND_GET_VLED_LIMITS              = 0xBA; ///<Get the max/min VLED allowed for this unit

constexpr uint16_t COMMAND_READ_SENSOR_INFO         = 0xD0;
constexpr uint16_t COMMAND_READ_SENSOR_STATUS       = 0xD1;

constexpr uint16_t COMMAND_READ_IMU_INFO           = 0xD2;


constexpr uint8_t DATA_TYPE_ID_OFFSET = 0;

/*
 * Offsets into version 0 protocol type 1 (measurement data) header
 */
constexpr uint32_t V0_T1_VERSION_INDEX = 0;                 ///<Index of the header version
constexpr uint32_t V0_T1_TYPE_INDEX = 1;                    ///<Index of the data type
constexpr uint32_t V0_T1_WIDTH_INDEX = 3;                   ///<Index of the width
constexpr uint32_t V0_T1_HEIGHT_INDEX = 5;                  ///<Index of the height
constexpr uint32_t V0_T1_ORIGIN_X_INDEX = 7;                ///<Index of the origin X
constexpr uint32_t V0_T1_ORIGIN_Y_INDEX = 9;                ///<Index of the origin y
constexpr uint32_t V0_T1_INTEGRATION_TIME_3D_0_INDEX = 15;  ///<Index of the integration time 3D 0
constexpr uint32_t V0_T1_INTEGRATION_TIME_3D_1_INDEX = 17;  ///<Index of the integration time 3D 1
constexpr uint32_t V0_T1_INTEGRATION_TIME_3D_2_INDEX = 19;  ///<Index of the integration time 3D 2
constexpr uint32_t V0_T1_TEMPERATURE_INDEX = 21;            ///<Index of the last temperature
constexpr uint32_t V0_T1_DATA_OFFSET_INDEX = 23;            ///<Index of the data offset_INDEX = offset where the measurement data starts
constexpr uint32_t V0_T1_USER_DATA_INDEX = 25;              ///<Index of the user data (start of KLV data)

} //end namespace TofComm

#endif
