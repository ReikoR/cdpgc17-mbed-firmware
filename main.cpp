#include <mbed-os/targets/TARGET_NXP/TARGET_LPC176X/device/cmsis.h>
#include "mbed.h"
#include "EthernetInterface.h"
#include "commands.h"

#define PORT 8042

EthernetInterface eth;
UDPSocket socket;
Serial pc(USBTX, USBRX);
Serial motorDriverSerial(p9, p10);
Ticker heartbeatTicker;
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

float heartBeatPeriod = 0.02;

bool isLedUpdate = false;
uint32_t ledUpdateCounter = 0;
uint32_t ledUpdateLimit = 25;

bool isFailsafeEnabled = true;
bool isFailsafeActive = true;
uint32_t failsafeCounter = 0;
uint32_t failsafeLimit = 5;

char recvBuffer[64];

uint32_t serialReceiveCounter = 0;
char serialReceiveBuffer[64];
uint32_t serialRxLength = 18;
uint32_t serialTxLength = 18;

void heartbeatTick() {
    ledUpdateCounter++;
    failsafeCounter++;

    if (ledUpdateCounter >= ledUpdateLimit) {
        ledUpdateCounter = 0;
        isLedUpdate = true;
    }

    if (failsafeCounter >= failsafeLimit) {
        failsafeCounter = 0;
        isFailsafeActive = true;
    }

    led3 = 0;
}

void serialWrite(char *sendData, int length) {
    int i = 0;

    while (i < length) {
        if (motorDriverSerial.writeable()) {
            motorDriverSerial.putc(sendData[i]);
            i++;
        }
    }
}

void sendSpeeds(int xMotorSpeed, int yMotorSpeed) {
    int yMotorPosition = yMotorSpeed < 0 ? -100 : 100;
    int xMotorPosition = xMotorSpeed < 0 ? -100 : 100;

    if (yMotorSpeed < 0) {
        yMotorSpeed = -yMotorSpeed;
    }

    if (xMotorSpeed < 0) {
        xMotorSpeed = -xMotorSpeed;
    }

    if (yMotorSpeed > 64000) {
        yMotorSpeed = 64000;
    }

    if (xMotorSpeed > 64000) {
        xMotorSpeed = 64000;
    }

    int qYMotorSpeed = ((yMotorSpeed << 10) / 1000) << 10;
    int qYMotorPosition = yMotorPosition << 20;

    int qXMotorSpeed = ((xMotorSpeed << 10) / 1000) << 10;
    int qXMotorPosition = xMotorPosition << 20;

    char sendBuffer[serialTxLength];

    sendBuffer[0] = '<';

    int * xPositionLocation = (int*)(&sendBuffer[1]);
    int * xSpeedLocation = (int*)(&sendBuffer[5]);

    int * yPositionLocation = (int*)(&sendBuffer[9]);
    int * ySpeedLocation = (int*)(&sendBuffer[13]);

    *yPositionLocation = qYMotorPosition;
    *ySpeedLocation = qYMotorSpeed;

    *xPositionLocation = qXMotorPosition;
    *xSpeedLocation = qXMotorSpeed;

    sendBuffer[17] = '>';

    //pc.printf("serialWrite called %d %*.*s\n", 10, 10, 10, sendBuffer);
    //pc.printf("serialWrite called\n");
    serialWrite(sendBuffer, serialTxLength);
}

void serialRxHandler() {
    // Interrupt does not work with RTOS when using standard functions (getc, putc)
    // https://developer.mbed.org/forum/bugs-suggestions/topic/4217/

    //while (motorDriverSerial.readable()) {
    while (LPC_UART3->LSR & 0x01) {
        //char c = motorDriverSerial.getc();
        char c = LPC_UART3->RBR;

        //LPC_UART0->RBR = '-';
        //LPC_UART0->RBR = (uint8_t)serialReceiveCounter;
        //LPC_UART0->RBR = '-';
        //LPC_UART0->RBR = (uint8_t)c;
        //LPC_UART0->RBR = '\n';

        if (serialReceiveCounter < serialRxLength) {
            if (serialReceiveCounter == 0) {
                if (c == '<') {
                    serialReceiveBuffer[serialReceiveCounter] = c;
                    serialReceiveCounter++;
                } else {
                    serialReceiveCounter = 0;
                }

            } else if (serialReceiveCounter > 0 && serialReceiveCounter < serialRxLength - 1) {
                serialReceiveBuffer[serialReceiveCounter] = c;
                serialReceiveCounter++;

            } else if (serialReceiveCounter == serialRxLength - 1) {
                if (c == '>') {
                    serialReceiveBuffer[serialReceiveCounter] = c;
                    serialReceiveCounter++;

                } else {
                    serialReceiveCounter = 0;
                }

            } else {
                serialReceiveCounter = 0;
            }
        }
    }
}

void onUDPSocketData(void* buffer, int size) {
    //pc.printf("UDP data received\n");
    led2 = !led2;

    failsafeCounter = 0;
    isFailsafeActive = false;

    //pc.printf("sizeof(SpeedCommand) %d\n", sizeof(SpeedCommand));
    //pc.printf("size %d\n", size);

    if (sizeof(SpeedCommand) == size) {
        SpeedCommand * command = static_cast<SpeedCommand*>(buffer);

        int yMotorSpeed = command->yMotorSpeed;
        int xMotorSpeed = command->xMotorSpeed;

        sendSpeeds(xMotorSpeed, yMotorSpeed);
    }
}

// Socket demo
int main() {
    https://github.com/ARMmbed/mbed-os-example-blinky/issues/78

    pc.baud(115200);

    eth.set_network("192.168.4.1", "255.255.255.0", "192.168.4.8");
    eth.connect();

    // Show the network address
    const char *ip = eth.get_ip_address();
    pc.printf("IP address is: %s\n", ip ? ip : "No IP");

    socket.set_blocking(false);
    socket.open(&eth);
    socket.bind(PORT);

    motorDriverSerial.baud(150000);

    motorDriverSerial.attach(serialRxHandler);

    heartbeatTicker.attach(&heartbeatTick, heartBeatPeriod);

    SocketAddress address;

    while (true) {
        if (isLedUpdate) {
            isLedUpdate = false;
            led1 = !led1;
        }

        if (isFailsafeEnabled && isFailsafeActive) {
            isFailsafeActive = false;

            sendSpeeds(0, 0);

            led3 = 1;
        }

        nsapi_size_or_error_t size = socket.recvfrom(&address, recvBuffer, sizeof recvBuffer);

        if (size < 0 && size != NSAPI_ERROR_WOULD_BLOCK) {
            pc.printf("recvfrom failed with error code %d\n", size);
        } else if (size > 0) {
            recvBuffer[size] = '\0';
            //pc.printf("recv %d [%s] from %s:%d\n", size, recvBuffer, address.get_ip_address(), address.get_port());

            onUDPSocketData(recvBuffer, size);
        }

        if (serialReceiveCounter == serialRxLength) {
            serialReceiveCounter = 0;

            int xMotorPosition = ((int) serialReceiveBuffer[1])
                                    | ((int) serialReceiveBuffer[2] << 8)
                                    | ((int) serialReceiveBuffer[3] << 16)
                                    | ((int) serialReceiveBuffer[4] << 24);

            xMotorPosition = ((xMotorPosition >> 10) * 1000) >> 10;

            int xMotorSpeed = ((int) serialReceiveBuffer[5])
                                 | ((int) serialReceiveBuffer[6] << 8)
                                 | ((int) serialReceiveBuffer[7] << 16)
                                 | ((int) serialReceiveBuffer[8] << 24);

            xMotorSpeed = ((xMotorSpeed >> 10) * 1000) >> 10;

            int yMotorPosition = ((int) serialReceiveBuffer[9])
                                 | ((int) serialReceiveBuffer[10] << 8)
                                 | ((int) serialReceiveBuffer[11] << 16)
                                 | ((int) serialReceiveBuffer[12] << 24);

            yMotorPosition = ((yMotorPosition >> 10) * 1000) >> 10;

            int yMotorSpeed = ((int) serialReceiveBuffer[13])
                              | ((int) serialReceiveBuffer[14] << 8)
                              | ((int) serialReceiveBuffer[15] << 16)
                              | ((int) serialReceiveBuffer[16] << 24);

            yMotorSpeed = ((yMotorSpeed >> 10) * 1000) >> 10;

            Feedback feedback;

            feedback.yMotorPosition = yMotorPosition;
            feedback.yMotorSpeed = yMotorSpeed;
            feedback.xMotorPosition = xMotorPosition;
            feedback.xMotorSpeed = xMotorSpeed;

            socket.sendto("192.168.4.8", 8042, &feedback, sizeof feedback);
        }
    }
}