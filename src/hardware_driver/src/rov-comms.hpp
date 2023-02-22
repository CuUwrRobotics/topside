#ifndef ROV_COMMS_CONTROLLER_HPP
#define ROV_COMMS_CONTROLLER_HPP

#include <stdint.h>
#include <ros/ros.h>
#include <stdio.h>

#include "serial/serial.hpp"
#include "hardware-driver-main.hpp"

class RovCommsController
{
  const uint8_t COMMS_HANDSHAKE, CIPO_BUFFER_LENGTH, COPI_BUFFER_LENGTH;

  uint8_t cipo_checksum, copi_checksum;

  uint8_t copi_index, cipo_index;

  uint8_t read_buffer_index;
  bool cipo_checksum_status;

  std::vector<uint8_t> read_buffer;

  int serial_fd;

  void sendChecksum();

public:
  RovCommsController(int ser, uint8_t cipo, uint8_t copi)
      : COMMS_HANDSHAKE((cipo * copi) & 0xFF),
        CIPO_BUFFER_LENGTH(cipo),
        COPI_BUFFER_LENGTH(copi),
        cipo_checksum(0),
        copi_checksum(0),
        copi_index(0),
        cipo_index(0),
        read_buffer_index(0),
        cipo_checksum_status(false),
        read_buffer(cipo),
        serial_fd(ser){};

  /**
   * @brief Waits for the handshake with the ROV to complete.
   *
   * @details This function blocks until the handshake is complete. It is not
   * aware of whether the handshake was successful or not according to the ROV.
   */
  void awaitHandshake();

  /**
   * @brief Sends a block of data to the ROV.
   *
   * If the final block of data is being sent, the checksum is also sent.
   *
   * @param data The data to send.
   */
  void sendBlock(uint8_t data);

  /**
   * @brief Sends an array of data to the ROV.
   *
   * If the final block of data is being sent, the checksum is also sent.
   *
   * @param data The data to send.
   * @param length The length of the data to send.
   */
  void sendBlocks(const uint8_t data[], size_t length);

  /**
   * @brief Sends the data to the ROV.
   *
   * If the final block of data is being sent, the checksum is also sent.
   *
   * @tparam T The type of data to send.
   * @param data The data to send.
   */
  template <typename T>
  void send(const T &data)
  {
    this->sendBlocks((uint8_t *)&data, sizeof(T));
  }

  /**
   * @brief Reads data from the ROV without blocking.
   *
   * @return The number of bytes in the read buffer so far, or -1 if completed.
   */
  int tryReadingData();

  /**
   * @return True if the last CIPO checksum was good, false otherwise.
   */
  inline bool checksumGood() const { return cipo_checksum_status; }
  /**
   * @brief Resets the read buffer index to 0.
   *
   * @details This should be called after the read buffer is processed.
   */
  inline void resetReadBuffer() { read_buffer_index = 0; }

  /**
   * @brief Resets all buffer indexes to 0.
   */
  inline void reset()
  {
    read_buffer_index = 0;
    copi_index = 0;
    cipo_index = 0;
    cipo_checksum = 0;
    copi_checksum = 0;
  }

  /**
   * @brief Pops a byte from the read buffer.
   *
   * Do not call this function while the read buffer is being written to.
   *
   * @return The byte at the read buffer index.
   */
  uint8_t popReadBuffer();

  /**
   * @brief Pops a block of data from the read buffer.
   *
   * Do not call this function while the read buffer is being written to.
   *
   * @param length The length of the data to pop.
   * @return A pointer to the data at the read buffer index.
   */
  uint8_t *popReadBuffer(size_t length);

  /**
   * @brief Pops a block of data from the read buffer.
   *
   * Do not call this function while the read buffer is being written to.
   *
   * @tparam T The type of data to pop.
   * @return The data at the read buffer index.
   */
  template <typename T>
  T popReadBuffer()
  {
    T data = *(T *)&read_buffer[read_buffer_index];
    read_buffer_index += sizeof(T);
    if (read_buffer_index > CIPO_BUFFER_LENGTH)
    {
      ROS_ERROR("Read buffer overflow in RovCommsController::popReadBuffer()");
      read_buffer_index = 0;
      exit(EXIT_FAILURE);
    }
    return data;
  }

}; // class RovCommsController

#endif // End of include guard for ROV_COMMS_CONTROLLER_HPP
