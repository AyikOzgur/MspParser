#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <stdint.h>

enum class MspCommand
{
    /**
     * @brief MSP_IDENT response
     * @param VERSION Version of MultiWii
     * @param MULTI_TYPE Type of multicopter (quad, hexa, etc.).
     * @param MSP_VERSION Not used currently.
     * @param CAPABILITY A 32 bit variable to indicate capability of FC board.
     *                   Currently, BIND button is used on first bit, DYNBAL on second, FLAP on third.
     */
    MSP_IDENT = 100,

    /**
     * @brief MSP_STATUS response
     * @param CYCLETIME Unit: microseconds.
     * @param I2C_ERRORS_COUNT Number of I2C errors.
     * @param SENSOR Present bit 0 = ACC, 1 = BARO, 2 = MAG, 3 = GPS, 4 = SONAR.
     * @param FLAG 	A bit variable to indicate which BOX are active, the bit position depends on the BOX which are configured
     * @param GLOBAL_CONF to indicate the current configuration setting. Unit: it depends on ACC sensor and is based on ACC_1G definition
     */
    MSP_STATUS = 101,

};


/**
 * @brief MspParser class
 */
class MspParser
{
public:

    ~MspParser();

    static std::string getVersion();

    bool encode(uint8_t* data, size_t& size, MspCommand command, std::vector<uint32_t> arguments = {});

    bool decode(uint8_t nextByte, MspCommand& command, std::vector<uint32_t>& arguments);

private:

};