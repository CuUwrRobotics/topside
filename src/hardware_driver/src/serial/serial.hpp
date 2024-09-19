#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

constexpr std::size_t OUTPUT_BUFFER_LENGTH = 1024;

namespace serial
{
::speed_t convertSpeed(const std::uint32_t baudRate);

/**
 * @brief Open a serial port with the given flags.
 *
 * @param deviceName The device file path.
 * @param baudRate   The desired baud rate.
 * @param ioFlags  The flags (defaults given).
 * @return int   The file pointer or -1.
 */
int openSerialPort(const char* deviceName,
                   const int   baudRate,
                   int ioFlags = O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

/**
 * @brief Wait for data to be written out of the rx buffer.
 *
 */
void serialFlush(const int fileDescriptor);

/**
 * @brief Empty the TX/RX buffers.
 *
 * Modes:
 *   - TCIFLUSH:  Discard any data received and not yet read by program.
 *   - TCOFLUSH:  Discard any data written by program but not yet sent by
 * hardware
 *   - TCIOFLUSH: Discard both buffers.
 *
 */
void serialEmpty(const int fileDescriptor, int mode = TCIOFLUSH);

/**
 * @brief Release the serial port
 *********************************************************************************
 */
void serialClose(const int fileDescriptor);

/**
 * @brief Send a single character to the serial port
 *********************************************************************************
 */
void serialPutchar(const int fileDescriptor, const unsigned char charToSend);

/**
 * @brief Write any datatype to the serial port
 *
 */
template<typename T>
void serialWrite(const int fileDescriptor, const T& data)
{
    for (std::size_t idx = 0; idx < sizeof(T); idx++)
    {
        serialPutchar(fileDescriptor, static_cast<std::uint8_t*>(&data)[idx]);
    }
}

/**
 * @brief Send a string to the serial port
 *********************************************************************************
 */
void serialPuts(const int fileDescriptor, const char* string);

/**
 * @brief Printf over Serial
 */
void serialPrintf(const int fileDescriptor, const char* message, ...);

/**
 * @brief Return the number of bytes of data available to be read in the
 * serial port
 */
int serialDataAvail(const int fileDescriptor);

/**
 * @brief Get a single character from the serial device.
 *
 * @note '\0' is considered a valid character
 *
 * @note This function will time-out after 10 seconds (or whatever is
 *       specified in the option c_cc[VTIME]).
 */
int serialGetchar(const int fileDescriptor);

}; // namespace serial
