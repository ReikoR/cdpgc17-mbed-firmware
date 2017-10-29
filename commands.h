#ifndef CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H
#define CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H

struct SpeedCommand {
    int32_t pipeMotorSpeed;
    int32_t xMotorSpeed;
};

struct Feedback {
    int32_t pipeMotorPosition;
    int32_t pipeMotorSpeed;
    int32_t xMotorPosition;
    int32_t xMotorSpeed;
};

#endif //CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H
