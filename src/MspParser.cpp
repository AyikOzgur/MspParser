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

bool MspParser::encode(uint8_t* data, size_t& size, MspCommand command, std::vector<uint32_t> arguments)
{
    // Build MSP_ATTITUDE request
    if (command == MspCommand::MSP_ATTITUDE)
    {
        data[0] = '$';
        data[1] = 'M';
        data[2] = '<'; // Because it is a request

        data[3] = 0; // Because it is a request

        data[4] = static_cast<uint8_t>(command);

        data[5] = crc(data + 3, 2);

        size = 6;

    }
    return false;
}

bool MspParser::decode(uint8_t nextByte, MspCommand& command, std::vector<uint32_t>& arguments)
{
    // Shift internal buffer
    for (size_t i = 0; i < sizeof(m_internalBuffer) - 1; i++)
    {
        m_internalBuffer[i] = m_internalBuffer[i + 1];
    }

    // Add new byte to internal buffer
    m_internalBuffer[sizeof(m_internalBuffer) - 1] = nextByte;

    // Check if we have a valid attitude response
    if (m_internalBuffer[0] == '$' && m_internalBuffer[1] == 'M' && m_internalBuffer[2] == '>')
    {
        // Check the id
        if (m_internalBuffer[4] == static_cast<uint8_t>(MspCommand::MSP_ATTITUDE))
        {

            int size = m_internalBuffer[3];

            uint8_t crcReceived = m_internalBuffer[5 + size];

            uint8_t crcCalculated = crc(m_internalBuffer + 3, size + 2);

            if (crcReceived == crcCalculated)
            {
                float pitch = static_cast<int16_t>(m_internalBuffer[5] | (m_internalBuffer[6] << 8));
                std::cout << "Pitch: " << float(pitch/10) << std::endl;

                return true;
            }
        }
    }

    return false;
}