#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <stdint.h>

enum class MspCommand
{
    /**
     * @brief Invalid command.
     */
    NONE = -1,

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

    /**
     * @brief MSP_RAW_IMU response
     * @param ACCX Accelerometer X axis
     * @param ACCY Accelerometer Y axis
     * @param ACCZ Accelerometer Z axis
     * @param GYROX Gyroscope X axis
     * @param GYROY Gyroscope Y axis
     * @param GYROZ Gyroscope Z axis
     * @param MAGX Magnetometer X axis
     * @param MAGY Magnetometer Y axis
     * @param MAGZ Magnetometer Z axis
     */
    MSP_RAW_IMU = 102,

    /**
     * @brief MSP_SERVO response
     * @param SERVO1 Servo 1 value
     * @param SERVO2 Servo 2 value
     * @param SERVO3 Servo 3 value
     * @param SERVO4 Servo 4 value
     * @param SERVO5 Servo 5 value
     * @param SERVO6 Servo 6 value
     * @param SERVO7 Servo 7 value
     * @param SERVO8 Servo 8 value
     */
    MSP_SERVO = 103,

    /**
     * @brief MSP_MOTOR response in range 1000 - 2000
     * @param MOTOR1 Motor 1 value
     * @param MOTOR2 Motor 2 value
     * @param MOTOR3 Motor 3 value
     * @param MOTOR4 Motor 4 value
     * @param MOTOR5 Motor 5 value
     * @param MOTOR6 Motor 6 value
     * @param MOTOR7 Motor 7 value
     * @param MOTOR8 Motor 8 value
     */
    MSP_MOTOR = 104,

    /**
     * @brief MSP_SET_MOTOR request in range 1000 - 2000
     * @param MOTOR1 Motor 1 value
     * @param MOTOR2 Motor 2 value
     * @param MOTOR3 Motor 3 value
     * @param MOTOR4 Motor 4 value
     * @param MOTOR5 Motor 5 value
     * @param MOTOR6 Motor 6 value
     * @param MOTOR7 Motor 7 value
     * @param MOTOR8 Motor 8 value
     */
    MSP_SET_MOTOR = 214,

    /**
     * @brief MSP_RC response in range 1000 - 2000
     * @param ROLL Roll value
     * @param PITCH Pitch value
     * @param YAW Yaw value
     * @param THROTTLE Throttle value
     * @param AUX1 AUX1 value
     * @param AUX2 AUX2 value
     * @param AUX3 AUX3 value
     * @param AUX4 AUX4 value
     */
    MSP_RC = 105,

    /**
     * @brief MSP_SET_RAW_RC request in range 1000 - 2000
     * @param ROLL Roll value
     * @param PITCH Pitch value
     * @param YAW Yaw value
     * @param THROTTLE Throttle value
     * @param AUX1 AUX1 value
     * @param AUX2 AUX2 value
     * @param AUX3 AUX3 value
     * @param AUX4 AUX4 value
     */
    MSP_SET_RAW_RC = 200,

    /**
     * @brief MSP_RAW_GPS response
     * @param GPS_FIX GPS Fix
     * @param GPS_NUMSAT Number of satellites
     * @param GPS_LAT Latitude
     * @param GPS_LON Longitude
     * @param GPS_ALT Altitude
     * @param GPS_SPEED Speed
     * @param GPS_GROUNDSPEED Ground speed
     */
    MSP_RAW_GPS = 106,

    /**
     * @brief MSP_COMP_GPS response
     * @param GPS_DISTANCE Distance from home
     * @param GPS_DIRECTION Direction from home
     * @param GPS_UPDATE GPS update
     */
    MSP_COMP_GPS = 107,

    /**
     * @brief MSP_ATTITUDE response in range -1800 - 1800, unit: 0.1 degree
     * @param ANG_X Angle X 
     * @param ANG_Y Angle Y
     * @param HEADING Heading
     */
    MSP_ATTITUDE = 108,

    /**
     * @brief MSP_ALTITUDE response
     * @param EST_ALTITUDE Estimated altitude in cm
     * @param VARIOMETER Variometer in cm/s
     */
    MSP_ALTITUDE = 109,

    /**
     * @brief MSP_ANALOG response
     * @param VBAT Battery voltage unit 0.1V
     * @param POWERMETER Power meter value
     * @param RSSI Received signal strength indicator in range 0 - 1023
     * @param AMPS Current in amperes
     */
    MSP_ANALOG = 110,

    /**
     * @brief MSP_RC_TUNING response
     * @param RC_RATE Rate
     * @param RC_EXPO Expo
     * @param ROLL_RATE Roll rate
     * @param PITCH_RATE Pitch rate
     * @param YAW_RATE Yaw rate
     * @param DYN_THR_PID Dynamic throttle PID
     * @param THR_MID Throttle mid
     * @param THR_EXPO Throttle expo
     */
    MSP_RC_TUNING = 111,

    /**
     * @brief MSP_SET_RC_TUNING request
     * @param RC_RATE Rate
     * @param RC_EXPO Expo
     * @param ROLL_RATE Roll rate
     * @param PITCH_RATE Pitch rate
     * @param YAW_RATE Yaw rate
     * @param DYN_THR_PID Dynamic throttle PID
     * @param THR_MID Throttle mid
     * @param THR_EXPO Throttle expo
     */
    MSP_SET_RC_TUNING = 204,

    /**
     * @brief MSP_PID response
     * @param PID_CONTROLLER PID controller
     * @param PID_PROFILE PID profile
     * @param PID_ROLL Rate Roll PID
     * @param PID_PITCH Rate Pitch PID
     * @param PID_YAW Rate Yaw PID
     * @param PID_ALT Altitude PID
     * @param PID_POS Position PID
     * @param PID_ITEM PID item
     */
    MSP_PID = 112,

    /**
     * @brief MSP_SET_PID request
     * @param PID_CONTROLLER PID controller
     * @param PID_PROFILE PID profile
     * @param PID_ROLL Rate Roll PID
     * @param PID_PITCH Rate Pitch PID
     * @param PID_YAW Rate Yaw PID
     * @param PID_ALT Altitude PID
     * @param PID_POS Position PID
     * @param PID_ITEM PID item
     */
    MSP_SET_PID = 202,

    /**
     * @brief MSP_BOX response
     * @param BOXITEM Box item
     */
    MSP_BOX = 113,

    /**
     * @brief MSP_SET_BOX request
     * @param BOXITEM Box item
     */
    MSP_SET_BOX = 203,

    /**
     * @brief MSP_MISC response
     * @param Not used currently
     */
    MSP_MISC = 114,

    /**
     * @brief MSP_SET_MISC request
     * @param Not used currently
     */
    MSP_SET_MISC = 207,

    /**
     * @brief MSP_MOTOR_PINS response
     * @param MOTOR0 Motor 0 pin
     * @param MOTOR1 Motor 1 pin
     * @param MOTOR2 Motor 2 pin
     * @param MOTOR3 Motor 3 pin
     * @param MOTOR4 Motor 4 pin
     * @param MOTOR5 Motor 5 pin
     * @param MOTOR6 Motor 6 pin
     * @param MOTOR7 Motor 7 pin
     */
    MSP_MOTOR_PINS = 115,

    /**
     * @brief MSP_BOXNAMES response
     * @param BOXNAMES Box names
     */
    MSP_BOXNAMES = 116,

    /**
     * @brief MSP_PIDNAMES response
     * @param PIDNAMES PID names
     */
    MSP_PIDNAMES = 117,

    /**
     * @brief MSP_WP response
     * @param Not used currently
     */
    MSP_WP = 118,

    /**
     * @brief MSP_SET_WP request
     * @param Not used currently
     */
    MSP_SET_WP = 209,

    /**
     * @brief MSP_BOXIDS response
     * @param BOXIDS Box IDs
     */
    MSP_BOXIDS = 119,

    /**
     * @brief MSP_SERVO_CONF response
     * @param SERVO0 Servo 0 configuration
     * @param SERVO1 Servo 1 configuration
     * @param SERVO2 Servo 2 configuration
     * @param SERVO3 Servo 3 configuration
     * @param SERVO4 Servo 4 configuration
     * @param SERVO5 Servo 5 configuration
     * @param SERVO6 Servo 6 configuration
     * @param SERVO7 Servo 7 configuration
     */
    MSP_SERVO_CONF = 120,

    /**
     * @brief MSP_SET_SERVO_CONF request
     * @param SERVO0 Servo 0 configuration
     * @param SERVO1 Servo 1 configuration
     * @param SERVO2 Servo 2 configuration
     * @param SERVO3 Servo 3 configuration
     * @param SERVO4 Servo 4 configuration
     * @param SERVO5 Servo 5 configuration
     * @param SERVO6 Servo 6 configuration
     * @param SERVO7 Servo 7 configuration
     */
    MSP_SET_SERVO_CONF = 212,

    /**
     * @brief MSP_ACC_CALIBRATION request
     * @param No arguments
     */
    MSP_ACC_CALIBRATION = 205,

    /**
     * @brief MSP_MAG_CALIBRATION request
     * @param No arguments
     */
    MSP_MAG_CALIBRATION = 206,

    /**
     * @brief MSP_RESET_CONF request
     * @param No arguments
     */
    MSP_RESET_CONF = 208,

    /**
     * @brief MSP_SELECT_SETTING request
     * @param SELECT Setting to select
     */
    MSP_SELECT_SETTING = 210,

    /**
     * @brief MSP_SET_HEAD request in range -180 - 180
     * @param ANGLE Angle
     */
    MSP_SET_HEAD = 211,

    /**
     * @brief MSP_BIND request
     * @param No arguments
     */
    MSP_BIND = 240,

    /**
     * @brief MSP_EEPROM_WRITE request
     * @param No arguments
     */
    MSP_EEPROM_WRITE = 250,

    /**
     * @brief MSP_MODE_RANGES request
     * @param No arguments
     */
    MSP_MODE_RANGES = 34,

    /**
     * @brief Set the position of rectangle on OSD.
     * @param posX X coordinate in grid.
     * @param posY Y coordinate in grid.
     * @param width rectangle width in grid.
     * @param height rectangle height in grid.
     * @remark THIS IS A CUSTOM COMMAND IT IS ONLY SUPPORTED BY CUSTOM BETAFLIGHT.
     */
    MSP_SET_RECTANGLE_POS = 190
};


/**
 * @brief MspMode ID
 */
enum class MSP_MODE_ID
{
    NONE = -1,
    ARM,
    ANGLE,
    HORIZON,
    ANTI_GRAVITY = 4,
    MAG,
    HEADFREE,
    HEADADJ,
    CAMSTAB,
    PASSTHRU = 12,
    BEEPERON,
    LEDLOW = 15,
    CALIB = 17,
    OSD = 19,
    TELEMETRY,
    SERVO_1 = 23,
    SERVO_2,
    SERVO_3,
    BLACKBOX,
    FAILSAFE,
    AIRMODE,
    D3, // IT IS 3D
    FPV_ANGLE_MIX,
    BLACKBOX_ERASE,
    CAMERA_CONTROL_1,
    CAMERA_CONTROL_2,
    CAMERA_CONTROL_3,
    FLIB_OVER_AFTER_CRASH,
    BOX_PREARM,
    BEEP_GPS_SATELLITE_COUNT,
    VTX_PIT_MODE = 39,
    USER_1,
    USER_2,
    USER_3,
    USER_4,
    PID_AUDIO,
    PARALYZE,
    GPS_RESCUE,
    ACRO_TRAINER,
    DISABLE_VTX_CONTROL,
    LAUNCH_CONTROL,
    MSP_OVERRIDE,
    STICK_COMMANDS_DISABLE,
    BEEPER_MUTE,
    READY,
    LAP_TIMER_RESET
};

struct MspMode
{
    MspMode (MSP_MODE_ID _id) : id(_id) {}
    /// @brief Mode ID
    MSP_MODE_ID id{MSP_MODE_ID::NONE};
    /// @brief Aux channel index
    int auxChannel{-1};
    /// @brief Range start
    int rangeStart{-1};
    /// @brief Range end
    int rangeEnd{-1};
};

struct MspModes
{
    MspMode arm{MSP_MODE_ID::ARM};
    MspMode angle{MSP_MODE_ID::ANGLE};
    MspMode horizon{MSP_MODE_ID::HORIZON};
    MspMode antiGravity{MSP_MODE_ID::ANTI_GRAVITY};
    MspMode mag{MSP_MODE_ID::MAG};
    MspMode headFree{MSP_MODE_ID::HEADFREE};
    MspMode headAdj{MSP_MODE_ID::HEADADJ};
    MspMode camstab{MSP_MODE_ID::CAMSTAB};
    MspMode passthru{MSP_MODE_ID::PASSTHRU};
    MspMode beeperOn{MSP_MODE_ID::BEEPERON};
    MspMode ledLow{MSP_MODE_ID::LEDLOW};
    MspMode calib{MSP_MODE_ID::CALIB};
    MspMode osd{MSP_MODE_ID::OSD};
    MspMode telemetry{MSP_MODE_ID::TELEMETRY};
    MspMode servo1{MSP_MODE_ID::SERVO_1};
    MspMode servo2{MSP_MODE_ID::SERVO_2};
    MspMode servo3{MSP_MODE_ID::SERVO_3};
    MspMode blackbox{MSP_MODE_ID::BLACKBOX};
    MspMode failsafe{MSP_MODE_ID::FAILSAFE};
    MspMode airmode{MSP_MODE_ID::AIRMODE};
    MspMode d3{MSP_MODE_ID::D3};
    MspMode fpvAngleMix{MSP_MODE_ID::FPV_ANGLE_MIX};
    MspMode blackboxErase{MSP_MODE_ID::BLACKBOX_ERASE};
    MspMode cameraControl1{MSP_MODE_ID::CAMERA_CONTROL_1};
    MspMode cameraControl2{MSP_MODE_ID::CAMERA_CONTROL_2};
    MspMode cameraControl3{MSP_MODE_ID::CAMERA_CONTROL_3};
    MspMode flibOverAfterCrash{MSP_MODE_ID::FLIB_OVER_AFTER_CRASH};
    MspMode boxPrearm{MSP_MODE_ID::BOX_PREARM};
    MspMode beepGpsSatelliteCount{MSP_MODE_ID::BEEP_GPS_SATELLITE_COUNT};
    MspMode vtxPitMode{MSP_MODE_ID::VTX_PIT_MODE};
    MspMode user1{MSP_MODE_ID::USER_1};
    MspMode user2{MSP_MODE_ID::USER_2};
    MspMode user3{MSP_MODE_ID::USER_3};
    MspMode user4{MSP_MODE_ID::USER_4};
    MspMode pidAudio{MSP_MODE_ID::PID_AUDIO};
    MspMode paralyze{MSP_MODE_ID::PARALYZE};
    MspMode gpsRescue{MSP_MODE_ID::GPS_RESCUE};
    MspMode acroTrainer{MSP_MODE_ID::ACRO_TRAINER};
    MspMode disableVtxControl{MSP_MODE_ID::DISABLE_VTX_CONTROL};
    MspMode launchControl{MSP_MODE_ID::LAUNCH_CONTROL};
    MspMode mspOverride{MSP_MODE_ID::MSP_OVERRIDE};
    MspMode stickCommandsDisable{MSP_MODE_ID::STICK_COMMANDS_DISABLE};
    MspMode beeperMute{MSP_MODE_ID::BEEPER_MUTE};
    MspMode ready{MSP_MODE_ID::READY};
    MspMode lapTimerReset{MSP_MODE_ID::LAP_TIMER_RESET};
};

/**
 * @brief MspParser class
 */
class MspParser
{
public:

    /**
     * @brief Get the version of library.
     * @return std::string version in format "MAJOR.MINOR.PATCH"
     */
    static std::string getVersion();

    /**
     * @brief Encode msp command.
     * @param data  Destination buffer to store encoded command.
     * @param size Size of encoded command buffer.
     * @param command Msp command to encode.
     * @param arguments Arguments of command.
     * @return True if command is encoded.
     */
    bool encode(uint8_t* data, size_t& size, MspCommand command, std::vector<uint16_t> arguments = {});

    /**
     * @brief Decode msp command response.
     * @param nextByte Next byte from response buffer.
     * @param command Detected msp command.
     * @param arguments Data returned by FC for related command.
     * @return True if command is decoded.
     */
    bool decode(uint8_t nextByte, MspCommand& command, std::vector<float>& arguments);

    /**
     * @brief Decode modes of FC.
     * @param nextByte Next byte from response buffer.
     * @param modes modes of fc.
     * @return True if modes are decoded.
     */
    bool decodeModes(uint8_t nextByte, MspModes& modes);

private:

    /// @brief Internal buffer for decoding response of FC.
    uint8_t m_internalBuffer[32]{0};
    /// @brief Internal buffer for decoding modes response of FC.
    uint8_t m_internalBufferModes[160]{0};

    /**
     * @brief Calculate the crc of buffer.
     * @param data Buffer for crc calculation.
     * @param size Buffer size.
     * @return Calculated crc value.
     */
    uint8_t crc(uint8_t* data, size_t size);
};