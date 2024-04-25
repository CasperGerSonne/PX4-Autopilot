

/* commander module headers */




/* PX4 headers */



#include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>
#include <matrix/math.hpp>

#include <termios.h>

#include "UartCommander.hpp"

// uart_commander.cpp


UartCommander::UartCommander() {
    // Constructor implementation
}

UartCommander::~UartCommander() {
    // Destructor implementation
}

int UartCommander::Uart_Rxtest1(int expectedn){


    Serial_Port serial_port("/dev/ttyS1", 115200);

    serial_port.start();
    uint8_t* data = new uint8_t[expectedn];


    int bytes_read = 0;
    int messages = 200;
    int correct = 0;
    int count = 0;

    while (count<messages) {

        bytes_read = serial_port._read_port(*data,expectedn,50);

        if (bytes_read > 0) {
            printf("\n%d : ",bytes_read);
            count++;
            for (int i = 0; i < bytes_read; i++) {
                    printf("%c ", static_cast<char>(data[i])); // Print each byte as ASCII character
                }
            if (bytes_read == expectedn){
                correct++;
            }
            bytes_read = 0;


        }
    }
    printf("%d",correct);
	serial_port.stop();


    return EXIT_SUCCESS;
}


int UartCommander::Uart_Rxtest2(int expectedn){

   // Create an instance of the Serial_Port class
    Serial_Port serial_port("/dev/ttyS1", 115200);

        serial_port.start();
        uint8_t* data = new uint8_t[expectedn];
        uint8_t* buffer = new uint8_t[expectedn];

        int bytes_read = 0;
        int totbytes = 0;
        int messages = 50;

        for (int cycle = 0; cycle < messages;cycle++) {

            while (totbytes < expectedn){

                bytes_read = serial_port._read_port(*buffer,expectedn,50);


                for (int i = 0; i<  totbytes + bytes_read;i++){
                    data[i+totbytes] = buffer[i];
                }
                totbytes += bytes_read;

            }

            printf("\n%d : ",totbytes);

            for (int i = 0; i < totbytes; i++) {
                printf("%d \n", static_cast<uint8_t>(data[i]));
            }

            totbytes = 0;
        }
	serial_port.stop();
    return EXIT_SUCCESS;
}
int UartCommander::Uart_Txtest(){

   // Create an instance of the Serial_Port class
    Serial_Port serial_port("/dev/ttyS1", 115200); // Adjust port and baud rate as necessary


        // Open the serial port
        serial_port.start();
        char *msg = new char[6];
        std::strcpy(msg, "hello");




        while (true) {
            // Define a variable to store the received byte


            // Read from the serial port
            serial_port._write_port(msg,6);

            sleep(1);


            // Check if reading was successful




        }
	serial_port.stop();


    return EXIT_SUCCESS;
}
