#ifndef TOFENG_COMMANDTYPES_HPP
#define TOFENG_COMMANDTYPES_HPP
/**
 * @file CommandTypes.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Declares constants and types associated with the TOF communication interface.
 */
#include <cstdint>


#ifdef __GNUC__
#define PACK_START
#define PACK_END __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK_START __pragma( pack(push, 1) )
#define PACK_END __pragma( pack(pop))
//#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

// #if !defined(PACKED)
// #    define PACKED __attribute__((__packed__))
// #endif

namespace TofComm
{

enum class StorageId_e: uint8_t
{
    UNDEFINED, CALIBRATON, LENS_DATA, MANUFACTURING_DATA, PERSISTENT_SETTINGS, LOG
};

enum class StorageMode_e: uint8_t
{
    RAW, EXTRACTED
};

enum class WriteOperation_e: uint8_t
{
    START, CONTINUE, FINISH
};

PACK_START struct StorageMetadata_T
{
    StorageMetadata_T( StorageId_e id, StorageMode_e mode ) :
        m_storageId(id), m_mode(mode)
    {
    }
    StorageId_e m_storageId { StorageId_e::UNDEFINED };
    StorageMode_e m_mode { StorageMode_e::RAW };
    uint16_t m_dataType { 0 };      ///< Identifies the type of data in the data set
    uint16_t m_typeVersion { 0 };   ///< The version of this data set's type
    uint32_t m_dataSize { 0 };      ///< The number of bytes in the data set (following this header's page)
    uint32_t m_dataCrc32 { 0 };     ///< The CRC32 of the data
} PACK_END;

PACK_START struct StorageReadCommand_T
{
    StorageReadCommand_T( StorageId_e id, StorageMode_e mode ) :
        m_storageId(id), m_mode(mode)
    {
    }
    StorageId_e m_storageId { StorageId_e::UNDEFINED };
    StorageMode_e m_mode { StorageMode_e::RAW };
    uint32_t m_offset { 0 };    ///< Offset into related storage of first byte to read
    uint32_t m_numBytes { 0 };  ///< Number of bytes to read (all of related storage ID if 0)
} PACK_END;

PACK_START struct WriteStartCommand_T
{
    WriteStartCommand_T(StorageId_e id, StorageMode_e mode) :
                m_storageId(id), m_mode(mode)
    {
    }
    const WriteOperation_e m_operation { WriteOperation_e::START };
    const StorageId_e m_storageId { StorageId_e::UNDEFINED };
    const StorageMode_e m_mode { StorageMode_e::RAW };
    const uint8_t m_dummy[1] { 0 }; // force 4 byte alignment of data that follows
    // DO NOT CHANGE THE ORDER OF THE ABOVE MEMBERS !
} PACK_END;

PACK_START struct WriteContinueCommand_T
{
    WriteContinueCommand_T(StorageId_e id, StorageMode_e mode, uint32_t offset) :
                m_storageId(id), m_mode(mode), m_offset(offset)
    {
    }
    const WriteOperation_e m_operation { WriteOperation_e::CONTINUE };
    const StorageId_e m_storageId { StorageId_e::UNDEFINED };
    const StorageMode_e m_mode { StorageMode_e::RAW };
    const uint8_t m_dummy[1] { 0 }; // force 4 byte alignment of data that follows
    // DO NOT CHANGE THE ORDER OF THE ABOVE MEMBERS !
    const uint32_t m_offset { 0 };    ///< Offset associated with first byte of data that follows
} PACK_END;

PACK_START struct WriteFinishCommand_T
{
    WriteFinishCommand_T(StorageId_e id, StorageMode_e mode, uint32_t size, uint32_t crc32) :
                m_storageId(id), m_mode(mode), m_size(size), m_crc32(crc32)
    {
    }
    const WriteOperation_e m_operation { WriteOperation_e::FINISH };
    const StorageId_e m_storageId { StorageId_e::UNDEFINED };
    const StorageMode_e m_mode { StorageMode_e::RAW };
    const uint8_t m_dummy[1] { 0 }; // force 4 byte alignment of data that follows
    // DO NOT CHANGE THE ORDER OF THE ABOVE MEMBERS !
    const uint32_t m_size { 0 };  ///< size of entire section's data
    const uint32_t m_crc32 { 0 }; ///< CRC32 of entire section's data
} PACK_END;


} //end namespace TofComm

#endif //TOFENG_COMMANDTYPES_HPP
