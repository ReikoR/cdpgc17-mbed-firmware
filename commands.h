#ifndef CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H
#define CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H

struct SpeedCommand {
    int16_t pipeMotorSpeed;
    int16_t boomMotorSpeed;
};

struct Feedback {
    int32_t pipeMotorPosition;
    int16_t pipeMotorSpeed;
    int32_t boomMotorPosition;
    int16_t boomMotorSpeed;
};

#endif //CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H
