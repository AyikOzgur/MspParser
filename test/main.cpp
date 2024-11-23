#include <iostream>
#include <thread>
#include "MspParser.h"
#include "MspParserVersion.h"
#include "SerialPort.h"

#define portNum 7
#define baudrate 115200
#define timeoutMsec 0


int main()
{
    std::cout << "MspParser version: " << MspParser::getVersion() << std::endl;

    MspParser mspParser;

    std::string serialPortName = "\\\\.\\COM" + std::to_string(portNum);

    cr::clib::SerialPort serialPort;
    if (!serialPort.open(serialPortName.c_str(), baudrate, timeoutMsec))
    {
        std::cout << "ERROR: Serial port could not open." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Serial port opened." << std::endl;
    }

    while (true)
    {
        uint8_t bufferWrite[1024]{0};   
        uint8_t bufferRead[1024]{0};

        size_t size = 0;
        mspParser.encode(bufferWrite, size, MspCommand::MSP_ATTITUDE, std::vector<uint32_t>{});

        if (!serialPort.write(bufferWrite, size))
        {
            std::cout << "ERROR: Could not write to serial port." << std::endl;
            return -1;
        }

        int bytes = serialPort.read(bufferRead, sizeof(bufferRead));
        
        for (int i = 0; i < bytes; i++)
        {
            MspCommand command;
            std::vector<uint32_t> arguments;
            mspParser.decode(bufferRead[i], command, arguments);
        }

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}