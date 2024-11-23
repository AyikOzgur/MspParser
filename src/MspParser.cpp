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
        data[2] = '<';

        data[3] = 0; // Because it is a request

        data[4] = static_cast<uint8_t>(command);

        data[5] = 0; // Because it is a request

        data[6] = crc(data + 3, 3);

        size = 7;

    }
    return false;

}

bool MspParser::decode(uint8_t nextByte, MspCommand& command, std::vector<uint32_t>& arguments)
{
    return false;
}