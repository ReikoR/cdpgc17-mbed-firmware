#ifndef CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H
#define CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H

struct SpeedCommand {
    int32_t yMotorSpeed;
    int32_t xMotorSpeed;
};

struct __attribute__((packed)) Feedback {
    int32_t yMotorPosition;
    int32_t yMotorSpeed;
    int32_t xMotorPosition;
    int32_t xMotorSpeed;
    uint8_t stop;
    uint8_t start;
};

#endif //CDP_GC_2017_MBED_FIRMWARE_COMMANDS_H
