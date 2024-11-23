#include "MspParser.h"
#include "MspParserVersion.h"

uint8_t crc(uint8_t* data, size_t size)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < size; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

MspParser::MspParser()
{
}

MspParser::~MspParser()
{
}

std::string MspParser::getVersion()
{
    return MSP_PARSER_VERSION;
}

bool MspParser::encode(uint8_t* data, size_t& size, MspCommand command, std::vector<int32_t> arguments)
{

    size = 0;

    switch (command)
    {
    case MspCommand::MSP_IDENT: [[fallthrough]];
    case MspCommand::MSP_STATUS: [[fallthrough]];
    case MspCommand::MSP_RAW_IMU: [[fallthrough]];
    case MspCommand::MSP_SERVO: [[fallthrough]];
    case MspCommand::MSP_MOTOR: [[fallthrough]];
    case MspCommand::MSP_RC: [[fallthrough]];
    case MspCommand::MSP_RAW_GPS: [[fallthrough]];
    case MspCommand::MSP_COMP_GPS: [[fallthrough]];
    case MspCommand::MSP_ATTITUDE: [[fallthrough]];
    case MspCommand::MSP_ALTITUDE: [[fallthrough]];
    case MspCommand::MSP_ANALOG: [[fallthrough]];
    case MspCommand::MSP_RC_TUNING: [[fallthrough]];
    case MspCommand::MSP_PID: [[fallthrough]];
    case MspCommand::MSP_BOX: [[fallthrough]];
    case MspCommand::MSP_MISC: [[fallthrough]];
    case MspCommand::MSP_MOTOR_PINS: [[fallthrough]];
    case MspCommand::MSP_BOXNAMES: [[fallthrough]];
    case MspCommand::MSP_PIDNAMES: [[fallthrough]];
    case MspCommand::MSP_WP: [[fallthrough]];
    case MspCommand::MSP_BOXIDS: [[fallthrough]];
    case MspCommand::MSP_SERVO_CONF: [[fallthrough]];
    case MspCommand::MSP_ACC_CALIBRATION: [[fallthrough]];
    case MspCommand::MSP_MAG_CALIBRATION: [[fallthrough]];
    case MspCommand::MSP_RESET_CONF: [[fallthrough]];
    case MspCommand::MSP_BIND: [[fallthrough]];
    case MspCommand::MSP_EEPROM_WRITE:
    {
        data[0] = '$';
        data[1] = 'M';
        data[2] = '<'; // Because it is a request

        data[3] = size;

        data[4] = static_cast<uint8_t>(command);

        data[size + 5] = crc(data + 3, 2 + size);

        size += 6; // Final buffer size
        break;
    }
    case MspCommand::MSP_SET_MOTOR: [[fallthrough]];
    case MspCommand::MSP_SET_RAW_RC:
    {
        data[0] = '$';
        data[1] = 'M';
        data[2] = '<'; // Because it is setting

        size = 32; // 16 arguments * 2 bytes
        data[3] = size;

        data[4] = static_cast<uint8_t>(command);

        for (size_t i = 0; i < 16; i++)
        {
            int16_t value = arguments[i];
            data[5 + i * 2] = value & 0xFF;
            data[6 + i * 2] = (value >> 8) & 0xFF;
        }

        data[size + 5] = crc(data + 3, 2 + size);

        size += 6; // Final buffer size
        break;
    }
    default:
    {
        // Rest of commands will be implemented in the future.
        return false;
    }
    }

    return true;
}

bool MspParser::decode(uint8_t nextByte, MspCommand& command, std::vector<float>& arguments)
{
    // Shift internal buffer
    for (size_t i = 0; i < sizeof(m_internalBuffer) - 1; i++)
    {
        m_internalBuffer[i] = m_internalBuffer[i + 1];
    }

    // Add new byte to internal buffer
    m_internalBuffer[sizeof(m_internalBuffer) - 1] = nextByte;

    // Check if we have a valid attitude response
    if (m_internalBuffer[0] != '$' || m_internalBuffer[1] != 'M' || m_internalBuffer[2] != '>')
    {
        return false;
    }

    int size = m_internalBuffer[3];
    uint8_t crcReceived = m_internalBuffer[5 + size];
    uint8_t crcCalculated = crc(m_internalBuffer + 3, size + 2);
    if (crcReceived != crcCalculated)
    {
        return false;
    }

    switch (static_cast<MspCommand>(m_internalBuffer[4]))
    {
    case MspCommand::MSP_ATTITUDE:
    {
        command = MspCommand::MSP_ATTITUDE;
        float pitch = static_cast<int16_t>(m_internalBuffer[5] | (m_internalBuffer[6] << 8)) / 10.0f;
        float roll = static_cast<int16_t>(m_internalBuffer[7] | (m_internalBuffer[8] << 8)) / 10.0f;
        float yaw = static_cast<int16_t>(m_internalBuffer[9] | (m_internalBuffer[10] << 8)) / 10.0f;
        arguments.push_back(pitch);
        arguments.push_back(roll);
        arguments.push_back(yaw);
        break;
    }
    case MspCommand::MSP_ALTITUDE:
    {
        command = MspCommand::MSP_ALTITUDE;
        float altitude = static_cast<int32_t>(m_internalBuffer[5] | (m_internalBuffer[6] << 8) | (m_internalBuffer[7] << 16) | (m_internalBuffer[8] << 24)) / 100.0f;
        arguments.push_back(altitude);
        break;
    }
    default:
    {
        // Rest of commands will be implemented in the future.
        return false;
    }
    }

    return true;
}