#ifndef SERIAL_HPP
#define SERIAL_HPP

#ifndef OUTPUT_BUFFER_LENGTH
  #define OUTPUT_BUFFER_LENGTH 1024
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

namespace serial {

speed_t convertSpeed(uint32_t baud);

/**
 * @brief Open a serial port with the given flags.
 *
 * @param device The device file path.
 * @param baud   The desired baud rate.
 * @param flags  The flags (defaults given).
 * @return int   The file pointer or -1.
 */
int openSerialPort(const char *device, const int baud, int flags = O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

/**
 * @brief Wait for data to be written out of the rx buffer.
 *
 */

void serialFlush(const int fd);

/**
 * @brief Empty the TX/RX buffers.
 * 
 * Modes:
 *   - TCIFLUSH:  Discard any data recieved and not yet read by program.
 *   - TCOFLUSH:  Discard any data written by program but not yet sent by hardware
 *   - TCIOFLUSH: Discard both buffers.
 *
 */

void serialEmpty(const int fd, int mode = TCIOFLUSH);

/**
 * @brief Release the serial port
 *********************************************************************************
 */

void serialClose(const int fd);

/**
 * @brief Send a single character to the serial port
 *********************************************************************************
 */

void serialPutchar(const int fd, const unsigned char c);

/**
 * @brief Write any datatype to the serial port
 * 
 */
template <typename T>
void serialWrite(const int fd, const T &data) {
  for (size_t i = 0; i < sizeof(T); i++) {
    serialPutchar(fd, ((uint8_t *)&data)[i]);
  }
}

/**
 * @brief Send a string to the serial port
 *********************************************************************************
 */

void serialPuts(const int fd, const char *s);

/**
 * @brief Printf over Serial
 */

void serialPrintf(const int fd, const char *message, ...);

/**
 * @brief Return the number of bytes of data avalable to be read in the serial port
 */

int serialDataAvail(const int fd);

/**
 * @brief Get a single character from the serial device.
 *
 * @note '\0' is considered a valid character
 *
 * @note This function will time-out after 10 seconds (or whatever is
 *       specified in the option c_cc[VTIME]).
 */

int serialGetchar(const int fd);

};  // namespace serial

#endif  // End of include guard for SERIAL_HPP