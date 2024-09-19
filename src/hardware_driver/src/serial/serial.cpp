/**
 * @file serial.cpp
 * @author Nick Steele
 * @brief  Basic serial port control mostly copied from wiringPi (which does not
 * work on Ubuntu)
 * @version 0.1
 * @date 2022-10-18
 *
 */

#include "serial.hpp"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace serial
{
int openSerialPort(const char* deviceName, const int baudRate, int ioFlags)
{
    struct ::termios options;
    int              status, fileDescriptor = ::open(deviceName, ioFlags);

    if (fileDescriptor == -1)
    {
        return -1;
    }

    ::fcntl(fileDescriptor, F_SETFL, O_RDWR);

    // Get and modify current options:

    ::tcgetattr(fileDescriptor, &options);

    ::speed_t iobaud = convertSpeed(baudRate);

    ::cfmakeraw(&options);
    ::cfsetispeed(&options, iobaud);
    ::cfsetospeed(&options, iobaud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 100; // Ten seconds (in hundredths)

    ::tcsetattr(fileDescriptor, TCSANOW, &options);

    ::ioctl(fileDescriptor, TIOCMGET, &status);

    status |= TIOCM_DTR;
    status |= TIOCM_RTS;

    ::ioctl(fileDescriptor, TIOCMSET, &status);

    ::usleep(10000); // Wait 10ms for things to settle and configure

    return fileDescriptor;
}

void serialFlush(const int fileDescriptor)
{
    ::tcdrain(fileDescriptor);
}

void serialEmpty(const int fileDescriptor, int mode)
{
    ::tcflush(fileDescriptor, mode);
}

void serialClose(const int fileDescriptor)
{
    ::close(fileDescriptor);
}

void serialPutchar(const int fileDescriptor, const unsigned char charToSend)
{
    constexpr std::size_t bytesToWrite = 1;
    ::write(fileDescriptor, &charToSend, bytesToWrite);
}

void serialPuts(const int fileDescriptor, const char* string)
{
    ::write(fileDescriptor, string, std::strlen(string));
}

void serialPrintf(const int fileDescriptor, const char* message, ...)
{
    ::va_list argp;
    char      buffer[OUTPUT_BUFFER_LENGTH];

    ::va_start(argp, message);
    ::vsnprintf(buffer, OUTPUT_BUFFER_LENGTH - 1, message, argp);
    ::va_end(argp);

    serialPuts(fileDescriptor, buffer);
}

int serialDataAvail(const int fileDescriptor)
{
    int result;

    if (-1 == ::ioctl(fileDescriptor, FIONREAD, &result))
    {
        return -1;
    }

    return result;
}

int serialGetchar(const int fileDescriptor)
{
    std::uint8_t receivedChar;

    if (-1 == ::read(fileDescriptor, &receivedChar, 1))
    {
        return -1;
    }

    return static_cast<int>(receivedChar) & 0xFF;
}

::speed_t convertSpeed(const std::uint32_t requestedBaudRate)
{
    ::speed_t baudRate;
    switch (requestedBaudRate)
    {
    case 50:
        baudRate = B50;
        break;
    case 75:
        baudRate = B75;
        break;
    case 110:
        baudRate = B110;
        break;
    case 134:
        baudRate = B134;
        break;
    case 150:
        baudRate = B150;
        break;
    case 200:
        baudRate = B200;
        break;
    case 300:
        baudRate = B300;
        break;
    case 600:
        baudRate = B600;
        break;
    case 1200:
        baudRate = B1200;
        break;
    case 1800:
        baudRate = B1800;
        break;
    case 2400:
        baudRate = B2400;
        break;
    case 4800:
        baudRate = B4800;
        break;
    case 9600:
        baudRate = B9600;
        break;
    case 19200:
        baudRate = B19200;
        break;
    case 38400:
        baudRate = B38400;
        break;
    case 57600:
        baudRate = B57600;
        break;
    case 115200:
        baudRate = B115200;
        break;
    case 230400:
        baudRate = B230400;
        break;
    case 460800:
        baudRate = B460800;
        break;
    case 500000:
        baudRate = B500000;
        break;
    case 576000:
        baudRate = B576000;
        break;
    case 921600:
        baudRate = B921600;
        break;
    case 1000000:
        baudRate = B1000000;
        break;
    case 1152000:
        baudRate = B1152000;
        break;
    case 1500000:
        baudRate = B1500000;
        break;
    case 2000000:
        baudRate = B2000000;
        break;
    case 2500000:
        baudRate = B2500000;
        break;
    case 3000000:
        baudRate = B3000000;
        break;
    case 3500000:
        baudRate = B3500000;
        break;
    case 4000000:
        baudRate = B4000000;
        break;

    default:
        std::string errorMessage
            = "Could not find baud rate constant for requested rate ";
        errorMessage = std::to_string(requestedBaudRate);
        throw std::runtime_error(errorMessage);
    }

    return baudRate;
}

}; // namespace serial
