/**
 * @file serial.cpp
 * @author Nick Steele
 * @brief  Basic serial port control mostly copied from wiringPi (which does not work on Ubuntu)
 * @version 0.1
 * @date 2022-10-18
 *
 */

#include "serial.hpp"

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
#include <stdexcept>

namespace serial {

int openSerialPort(const char *device, const int baud, int flags) {
  struct termios options;
  int status, fd = open(device, flags);

  if (fd == -1)
    return -1;

  fcntl(fd, F_SETFL, O_RDWR);

  // Get and modify current options:

  tcgetattr(fd, &options);

  speed_t iobaud = convertSpeed(baud);

  cfmakeraw(&options);
  cfsetispeed(&options, iobaud);
  cfsetospeed(&options, iobaud);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 100;  // Ten seconds (in hundreths)

  tcsetattr(fd, TCSANOW, &options);

  ioctl(fd, TIOCMGET, &status);

  status |= TIOCM_DTR;
  status |= TIOCM_RTS;

  ioctl(fd, TIOCMSET, &status);

  usleep(10000);  // Wait 10ms for things to settle and configure

  return fd;
}

void serialFlush(const int fd) {
  tcdrain(fd);
}

void serialEmpty(const int fd, int mode) {
  tcflush(fd, mode);
}


void serialClose(const int fd) {
  close(fd);
}

void serialPutchar(const int fd, const unsigned char c) {
  write(fd, &c, 1);
}

void serialPuts(const int fd, const char *s) {
  write(fd, s, strlen(s));
}

void serialPrintf(const int fd, const char *message, ...) {
  va_list argp;
  char buffer[OUTPUT_BUFFER_LENGTH];

  va_start(argp, message);
  vsnprintf(buffer, OUTPUT_BUFFER_LENGTH - 1, message, argp);
  va_end(argp);

  serialPuts(fd, buffer);
}

int serialDataAvail(const int fd) {
  int result;

  if (ioctl(fd, FIONREAD, &result) == -1)
    return -1;

  return result;
}

int serialGetchar(const int fd) {
  uint8_t x;

  if (read(fd, &x, 1) != 1)
    return -1;

  return ((int)x) & 0xFF;
}

speed_t convertSpeed(uint32_t baud) {
  speed_t baud_ioctl;
  switch (baud) {
    case 50:
      baud_ioctl = B50;
      break;
    case 75:
      baud_ioctl = B75;
      break;
    case 110:
      baud_ioctl = B110;
      break;
    case 134:
      baud_ioctl = B134;
      break;
    case 150:
      baud_ioctl = B150;
      break;
    case 200:
      baud_ioctl = B200;
      break;
    case 300:
      baud_ioctl = B300;
      break;
    case 600:
      baud_ioctl = B600;
      break;
    case 1200:
      baud_ioctl = B1200;
      break;
    case 1800:
      baud_ioctl = B1800;
      break;
    case 2400:
      baud_ioctl = B2400;
      break;
    case 4800:
      baud_ioctl = B4800;
      break;
    case 9600:
      baud_ioctl = B9600;
      break;
    case 19200:
      baud_ioctl = B19200;
      break;
    case 38400:
      baud_ioctl = B38400;
      break;
    case 57600:
      baud_ioctl = B57600;
      break;
    case 115200:
      baud_ioctl = B115200;
      break;
    case 230400:
      baud_ioctl = B230400;
      break;
    case 460800:
      baud_ioctl = B460800;
      break;
    case 500000:
      baud_ioctl = B500000;
      break;
    case 576000:
      baud_ioctl = B576000;
      break;
    case 921600:
      baud_ioctl = B921600;
      break;
    case 1000000:
      baud_ioctl = B1000000;
      break;
    case 1152000:
      baud_ioctl = B1152000;
      break;
    case 1500000:
      baud_ioctl = B1500000;
      break;
    case 2000000:
      baud_ioctl = B2000000;
      break;
    case 2500000:
      baud_ioctl = B2500000;
      break;
    case 3000000:
      baud_ioctl = B3000000;
      break;
    case 3500000:
      baud_ioctl = B3500000;
      break;
    case 4000000:
      baud_ioctl = B4000000;
      break;

    default:
      char buffer[256];
      sprintf(buffer, "Could not find baud constant for rate %d.", baud);
      throw std::runtime_error(buffer);
  }
  return baud_ioctl;
}

};  // namespace serial
