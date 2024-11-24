#include <iostream>
#include <thread>
#include "MspParser.h"
#include "MspParserVersion.h"
#include "SerialPort.h"

int main()
{
    std::cout << "MspParser version: " << MspParser::getVersion() << std::endl;

    MspParser mspParser;

    std::string serialPortName = "COM7";
    int baudrate = 115200;

    std::cout << "Default serial port: " << serialPortName << std::endl;
    std::cout << "Default baudrate: " << baudrate << std::endl;

    int useDefault;
    std::cout << "Use default serial port settings? (1 - yes, 0 - no): ";
    std::cin >> useDefault;

    if (useDefault == 0)
    {

        std::cout << "Enter serial port name (/dev/ttyUSB0 for linux , COM1 for windows): ";
        std::cin >> serialPortName;

#if defined(_WIN32)
        serialPortName = "\\\\.\\" + serialPortName;
#endif

        std::cout << "Enter baudrate: ";
        std::cin >> baudrate;
    }

    cr::clib::SerialPort serialPort;
    if (!serialPort.open(serialPortName.c_str(), baudrate, 0))
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

        std::cout << "---------------------------------" << std::endl;
        std::cout << "-1 Exit" << std::endl;
        std::cout << static_cast<int>(MspCommand::MSP_ATTITUDE) << "  Get angles." << std::endl;
        std::cout << static_cast<int>(MspCommand::MSP_ALTITUDE) << "  Get altitude" << std::endl;
        std::cout << static_cast<int>(MspCommand::MSP_ANALOG) << "  Get analog" << std::endl;
        std::cout << static_cast<int>(MspCommand::MSP_SET_RAW_RC) << "  Set RC chanlles" << std::endl;
        
        int option;
        std::cout << "Select option: ";
        std::cin >> option;

        if (option == -1)
        {
            return 0;
        }

        MspCommand command = static_cast<MspCommand>(option);

        std::vector<int32_t> arguments;
        if (command == MspCommand::MSP_SET_RAW_RC)
        {
            for (int i = 0; i < 8; i++)
            {
                int value;
                std::cout << "Enter value for channel " << i + 1 << ": ";
                std::cin >> value;
                arguments.push_back(value);
            }
        }

        size_t size = 0;
        if (!mspParser.encode(bufferWrite, size, command, arguments))
        {
            std::cout << "ERROR: Could not encode MSP command." << std::endl;
            continue;
        }

        if (!serialPort.write(bufferWrite, size))
        {
            std::cout << "ERROR: Could not write to serial port." << std::endl;
            continue;
        }

        int bytes = serialPort.read(bufferRead, sizeof(bufferRead));
        std::cout << "Bytes read: " << bytes << std::endl;
        for (int i = 0; i < bytes; i++)
        {
            MspCommand command;
            std::vector<float> arguments;
            if (mspParser.decode(bufferRead[i], command, arguments))
            {
                if (command == MspCommand::MSP_ATTITUDE)
                {
                    std::cout << "ATTITUDE: " << arguments[0] << " " << arguments[1] << " " << arguments[2] << std::endl;
                }
                else if (command == MspCommand::MSP_ALTITUDE)
                {
                    std::cout << "ALTITUDE: " << arguments[0] << std::endl;
                }
                else if (command == MspCommand::MSP_ANALOG)
                {
                    std::cout << "BATTERY: " << arguments[0] << std::endl;
                }
            }
        }

    }

    return 0;
}