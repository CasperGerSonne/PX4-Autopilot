// uart_commander.hpp

#ifndef UART_COMMANDER_HPP
#define UART_COMMANDER_HPP

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <cstring>
#include "Serial_port/serial_port.h"

class UartCommander {
public:
    UartCommander(); // Constructor
    ~UartCommander(); // Destructor
    int Uart_Rxtest1(int expected);
    int Uart_Rxtest2(int expected);
    int Uart_Txtest();

};

#endif /* UART_COMMANDER_HPP */
