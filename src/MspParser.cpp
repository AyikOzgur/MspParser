#include <cstring>
#include "MspParser.h"
#include "MspParserVersion.h"

std::string MspParser::getVersion()
{
    return MSP_PARSER_VERSION;
}

bool MspParser::encode(uint8_t* data, size_t& size, MspCommand command, std::vector<uint16_t> arguments)
{

    size = 0;
    data[0] = '$';
    data[1] = 'M';
    data[2] = '<';

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
    case MspCommand::MSP_MODE_RANGES: [[fallthrough]];
    case MspCommand::MSP_EEPROM_WRITE:
    {
        data[3] = size;
        data[4] = static_cast<uint8_t>(command);
        break;
    }
    case MspCommand::MSP_SET_MOTOR: [[fallthrough]];
    case MspCommand::MSP_SET_RAW_RC:
    {
        size = 16; // 8 arguments * 2 bytes
        data[3] = size;
        data[4] = static_cast<uint8_t>(command);

        for (size_t i = 0; i < 8; i++)
        {
            uint16_t value = arguments[i];
            data[5 + i * 2] = value & 0xFF;
            data[6 + i * 2] = (value >> 8) & 0xFF;
        }
        break;
    }
    case MspCommand::MSP_SET_RECTANGLE_POS:
    {
        size = 2;
        data[3] = size;

        data[4] = static_cast<uint8_t>(command);
        data[5] = arguments[0];
        data[6] = arguments[1];
        break;
    }
    default:
    {
        // Rest of commands will be implemented in the future.
        return false;
    }
    }

    data[size + 5] = crc(data + 3, 2 + size);
    size += 6; // Final buffer size to write to serial port.

    return true;
}

bool MspParser::decode(uint8_t nextByte, MspCommand& command, std::vector<float>& arguments)
{
    // Shift internal buffer
    for (size_t i = 0; i < sizeof(m_internalBuffer) - 1; i++)
        m_internalBuffer[i] = m_internalBuffer[i + 1];

    // Add new byte to internal buffer
    m_internalBuffer[sizeof(m_internalBuffer) - 1] = nextByte;

    // Check if we have a valid attitude response
    if (m_internalBuffer[0] != '$' || m_internalBuffer[1] != 'M' || m_internalBuffer[2] != '>')
        return false;

    int size = m_internalBuffer[3];
    uint8_t crcReceived = m_internalBuffer[5 + size];
    uint8_t crcCalculated = crc(m_internalBuffer + 3, size + 2);
    if (crcReceived != crcCalculated)
        return false;

    switch (static_cast<MspCommand>(m_internalBuffer[4]))
    {
    case MspCommand::MSP_ATTITUDE:
    {
        command = MspCommand::MSP_ATTITUDE;
        float roll = static_cast<int16_t>(m_internalBuffer[5] | (m_internalBuffer[6] << 8)) / 10.0f;
        float pitch = static_cast<int16_t>(m_internalBuffer[7] | (m_internalBuffer[8] << 8)) / 10.0f;
        float yaw = static_cast<int16_t>(m_internalBuffer[9] | (m_internalBuffer[10] << 8)) ;
        arguments.push_back(roll);
        arguments.push_back(pitch);
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
    case MspCommand::MSP_ANALOG:
    {
        command = MspCommand::MSP_ANALOG;
        float vbat = static_cast<int16_t>(m_internalBuffer[5] ) / 10.0f;
        float powerMeterSum = static_cast<int16_t>(m_internalBuffer[6] | (m_internalBuffer[7] << 8));
        float rssi = static_cast<int16_t>(m_internalBuffer[8] | (m_internalBuffer[9] << 8));
        arguments.push_back(vbat);
        arguments.push_back(powerMeterSum);
        arguments.push_back(rssi);
        break;
    }
    case MspCommand::MSP_RAW_IMU:
    {
        command = MspCommand::MSP_RAW_IMU;
        float accx = static_cast<int16_t>(m_internalBuffer[5] | (m_internalBuffer[6] << 8));
        float accy = static_cast<int16_t>(m_internalBuffer[7] | (m_internalBuffer[8] << 8));
        float accz = static_cast<int16_t>(m_internalBuffer[9] | (m_internalBuffer[10] << 8));
        float gyrx = static_cast<int16_t>(m_internalBuffer[11] | (m_internalBuffer[12] << 8));
        float gyry = static_cast<int16_t>(m_internalBuffer[13] | (m_internalBuffer[14] << 8));
        float gyrz = static_cast<int16_t>(m_internalBuffer[15] | (m_internalBuffer[16] << 8));
        float magx = static_cast<int16_t>(m_internalBuffer[17] | (m_internalBuffer[18] << 8));
        float magy = static_cast<int16_t>(m_internalBuffer[19] | (m_internalBuffer[20] << 8));
        float magz = static_cast<int16_t>(m_internalBuffer[21] | (m_internalBuffer[22] << 8));
        arguments.push_back(accx / 512); // In g unit for mpu6050.
        arguments.push_back(accy / 512); // In g unit for mpu6050.
        arguments.push_back(accz / 512); // In g unit for mpu6050.
        arguments.push_back(gyrx);
        arguments.push_back(gyry);
        arguments.push_back(gyrz);
        arguments.push_back(magx);
        arguments.push_back(magy);
        arguments.push_back(magz);
        break;
    }
    default:
    {
        // Rest of commands will be implemented in the future.
        return false;
    }
    }

    // Clear internal buffer
    memset(m_internalBuffer, 0, sizeof(m_internalBuffer));

    return true;
}

bool MspParser::decodeModes(uint8_t nextByte, MspModes& modes)
{
    // Shift internal buffer
    for (size_t i = 0; i < sizeof(m_internalBufferModes) - 1; i++)
        m_internalBufferModes[i] = m_internalBufferModes[i + 1];

    // Add new byte to internal buffer
    m_internalBufferModes[sizeof(m_internalBufferModes) - 1] = nextByte;

    // Check if we have a valid attitude response
    if (m_internalBufferModes[0] != '$' || m_internalBufferModes[1] != 'M' || m_internalBufferModes[2] != '>')
        return false;

    int size = m_internalBufferModes[3];
    uint8_t crcReceived = m_internalBufferModes[5 + size];
    uint8_t crcCalculated = crc(m_internalBufferModes + 3, size + 2);
    if (crcReceived != crcCalculated)
        return false;

    if (static_cast<MspCommand>(m_internalBufferModes[4]) != MspCommand::MSP_MODE_RANGES)
        return false;

    // Modes are stored in the buffer from byte 5 to byte 5 + size
    // byte [0] = id
    // byte [1] = aux channel (from 0 index)
    // byte [2] = range start (900 + 25 * value)
    // byte [3] = range end (900 + 25 * value)

    // We need this flag because all other unused modes are set to 0 id like arm mode
    bool isArmFound = false;

    for (int i = 5; i < 5 + size; i += 4)
    {
        MSP_MODE_ID id = static_cast<MSP_MODE_ID>(m_internalBufferModes[i]);

        MspMode *mode = &modes.angle;
        for (int j = 0; j < sizeof(modes) / sizeof(MspMode); j++)
        {
            if (id == MSP_MODE_ID::ARM && !isArmFound)
            {
                // Arm mode is the first mode in the buffer
                modes.arm.auxChannel = ++m_internalBufferModes[i + 1];
                modes.arm.rangeStart = m_internalBufferModes[i + 2] * 25 + 900;
                modes.arm.rangeEnd = m_internalBufferModes[i + 3] * 25 + 900;
                isArmFound = true;
            }
            else if (mode->id == id && id != MSP_MODE_ID::ARM)
            {
                mode->auxChannel = ++m_internalBufferModes[i + 1];
                mode->rangeStart = m_internalBufferModes[i + 2] * 25 + 900;
                mode->rangeEnd = m_internalBufferModes[i + 3] * 25 + 900;
                break;
            }
            mode++;
        }
    }

    // Clear internal buffer
    memset(m_internalBufferModes, 0, sizeof(m_internalBufferModes));

    return true;
}

uint8_t MspParser::crc(uint8_t* data, size_t size)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < size; i++)
        crc ^= data[i];
    return crc;
}