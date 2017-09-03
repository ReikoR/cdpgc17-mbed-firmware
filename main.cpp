#include <mbed-os/targets/TARGET_NXP/TARGET_LPC176X/device/cmsis.h>
#include "mbed.h"
#include "EthernetInterface.h"
#include "commands.h"

#define PORT 8042

EthernetInterface eth;
UDPSocket socket;
Serial pc(USBTX, USBRX);
Serial motorDriverSerial(p9, p10);
Ticker ledTicker;
DigitalOut led1(LED1);
DigitalOut led2(LED2);

bool ledUpdate = false;

char recvBuffer[64];

uint32_t serialReceiveCounter = 0;
char serialReceiveBuffer[64];
uint32_t serialRxLength = 10;

void ledUpdateTick() {
    ledUpdate = true;
}

void serialWrite(char *sendData, int length) {
    int i = 0;

    while (i < length) {
        if (motorDriverSerial.writeable()) {
            motorDriverSerial.putc(sendData[i]);
        }
        i++;
    }
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

    //pc.printf("sizeof(SpeedCommand) %d\n", sizeof(SpeedCommand));
    //pc.printf("size %d\n", size);

    if (sizeof(SpeedCommand) == size) {
        SpeedCommand * command = static_cast<SpeedCommand*>(buffer);

        int pipeMotorSpeed = command->pipeMotorSpeed;
        int boomMotorSpeed = command->boomMotorSpeed;
        int pipeMotorPosition = pipeMotorSpeed < 0 ? -100 : 100;

        //pc.printf("pipe motor speed %d\n", pipeMotorSpeed);
        //pc.printf("boom motor speed %d\n", boomMotorSpeed);

        if (pipeMotorSpeed < 0) {
            pipeMotorSpeed = -pipeMotorSpeed;
        }

        int qSpeed = ((pipeMotorSpeed << 10) / 1000) << 10;
        int qPosition = pipeMotorPosition << 20;

        char sendBuffer[10];

        sendBuffer[0] = '<';

        int * positionLocation = (int*)(&sendBuffer[1]);
        int * speedLocation = (int*)(&sendBuffer[5]);

        *speedLocation = qSpeed;
        *positionLocation = qPosition;

        sendBuffer[9] = '>';

        //pc.printf("serialWrite called %d %*.*s\n", 10, 10, 10, sendBuffer);
        //pc.printf("serialWrite called\n");
        serialWrite(sendBuffer, 10);
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

    ledTicker.attach(&ledUpdateTick, 0.5);

    SocketAddress address;

    while (true) {
        if (ledUpdate) {
            ledUpdate = false;
            led1 = !led1;
        }

        nsapi_size_or_error_t size = socket.recvfrom(&address, recvBuffer, sizeof recvBuffer);

        if (size < 0 && size != NSAPI_ERROR_WOULD_BLOCK) {
            pc.printf("recvfrom failed with error code %d\n", size);
        } else if (size > 0) {
            recvBuffer[size] = '\0';
            //pc.printf("recv %d [%s] from %s:%d\n", size, recvBuffer, address.get_ip_address(), address.get_port());

            onUDPSocketData(recvBuffer, size);
        }

        if (serialReceiveCounter == 10) {
            serialReceiveCounter = 0;

            int pipeMotorPosition = ((int) serialReceiveBuffer[1])
                                    | ((int) serialReceiveBuffer[2] << 8)
                                    | ((int) serialReceiveBuffer[3] << 16)
                                    | ((int) serialReceiveBuffer[4] << 24);

            pipeMotorPosition = ((pipeMotorPosition >> 10) * 1000) >> 10;

            int pipeMotorSpeed = ((int) serialReceiveBuffer[5])
                                 | ((int) serialReceiveBuffer[6] << 8)
                                 | ((int) serialReceiveBuffer[7] << 16)
                                 | ((int) serialReceiveBuffer[8] << 24);

            pipeMotorSpeed = ((pipeMotorSpeed >> 10) * 1000) >> 10;

            Feedback feedback;

            feedback.pipeMotorPosition = pipeMotorPosition;
            feedback.pipeMotorSpeed = (int16_t) pipeMotorSpeed;
            feedback.boomMotorPosition = 0;
            feedback.boomMotorSpeed = 0;

            socket.sendto("192.168.4.8", 8042, &feedback, sizeof feedback);
        }
    }
}