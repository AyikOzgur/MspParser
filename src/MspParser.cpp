#include "MspParser.h"
#include "MspParserVersion.h"

MspParser::~MspParser()
{
}

std::string MspParser::getVersion()
{
    return MSP_PARSER_VERSION;
}

bool MspParser::encode(uint8_t* data, size_t& size, MspCommand command, std::vector<uint32_t> arguments)
{
    return false;
}

bool MspParser::decode(uint8_t nextByte, MspCommand& command, std::vector<uint32_t>& arguments)
{
    return false;
}