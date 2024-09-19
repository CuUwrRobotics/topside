#pragma once

#include <cstdint>
#include <cstdio>
#include <vector>

#include <ros/ros.h>

#include "hardware-driver-main.hpp"
#include "serial/serial.hpp"

class RovCommsController
{
  private:
    const std::uint8_t COMMS_HANDSHAKE;
    const std::uint8_t CIPO_BUFFER_LENGTH;
    const std::uint8_t COPI_BUFFER_LENGTH;

    std::uint8_t              m_CipoChecksum         = 0;
    std::uint8_t              m_CopiChecksum         = 0;
    std::uint8_t              m_CopiIndex            = 0;
    std::uint8_t              m_CipoIndex            = 0;
    std::uint8_t              m_ReadBufferIndex      = 0;
    bool                      m_CipoChecksumStatus   = false;
    std::vector<std::uint8_t> m_ReadBuffer           = {};
    int                       m_SerialFileDescriptor = -1;

    void sendChecksum();

  public:
    RovCommsController(const int          serialFileDescriptor,
                       const std::uint8_t cipoBufferLength,
                       const std::uint8_t copiBufferLength)
        : COMMS_HANDSHAKE((cipoBufferLength * copiBufferLength) & 0xFF),
          CIPO_BUFFER_LENGTH(cipoBufferLength),
          COPI_BUFFER_LENGTH(copiBufferLength),
          m_CipoChecksum(0),
          m_CopiChecksum(0),
          m_CopiIndex(0),
          m_CipoIndex(0),
          m_ReadBufferIndex(0),
          m_CipoChecksumStatus(false),
          m_ReadBuffer(cipoBufferLength),
          m_SerialFileDescriptor(serialFileDescriptor) {};

    inline int getFileDescriptor() { return m_SerialFileDescriptor; };

    /**
     * @brief Waits for the handshake with the ROV to complete.
     *
     * @details This function blocks until the handshake is complete. It is not
     * aware of whether the handshake was successful or not according to the
     * ROV.
     */
    void awaitHandshake();

    /**
     * @brief Sends a block of data to the ROV.
     *
     * If the final block of data is being sent, the checksum is also sent.
     *
     * @param data The data to send.
     */
    void sendBlock(const std::uint8_t data);

    /**
     * @brief Sends an array of data to the ROV.
     *
     * If the final block of data is being sent, the checksum is also sent.
     *
     * @param data The data to send.
     * @param length The length of the data to send.
     */
    void sendBlocks(const std::uint8_t data[], const std::size_t length);

    /**
     * @brief Sends the data to the ROV.
     *
     * If the final block of data is being sent, the checksum is also sent.
     *
     * @tparam T The type of data to send.
     * @param data The data to send.
     */
    template<typename T>
    void send(const T& data)
    {
        std::uint8_t dataBytes[sizeof(T)];
        std::memcpy(dataBytes, &data, sizeof(T));
        this->sendBlocks(dataBytes, sizeof(T));
    }

    /**
     * @brief Reads data from the ROV without blocking.
     *
     * @return The number of bytes in the read buffer so far, or -1 if
     * completed.
     */
    int tryReadingData();

    /**
     * @return True if the last CIPO checksum was good, false otherwise.
     */
    inline bool checksumGood() const { return m_CipoChecksumStatus; }

    /**
     * @brief Resets the read buffer index to 0.
     *
     * @details This should be called after the read buffer is processed.
     */
    inline void resetReadBuffer() { m_ReadBufferIndex = 0; }

    /**
     * @brief Resets all buffer indexes to 0.
     */
    inline void reset()
    {
        m_ReadBufferIndex = 0;
        m_CopiIndex       = 0;
        m_CipoIndex       = 0;
        m_CipoChecksum    = 0;
        m_CopiChecksum    = 0;

        serial::serialEmpty(m_SerialFileDescriptor, TCIOFLUSH);
    }

    /**
     * @brief Pops a byte from the read buffer.
     *
     * Do not call this function while the read buffer is being written to.
     *
     * @return The byte at the read buffer index.
     */
    std::uint8_t popReadBuffer();

    /**
     * @brief Pops a block of data from the read buffer.
     *
     * Do not call this function while the read buffer is being written to.
     *
     * @param length The length of the data to pop.
     * @return A pointer to the data at the read buffer index.
     */
    std::uint8_t* popReadBuffer(const std::size_t length);

    /**
     * @brief Pops a block of data from the read buffer.
     *
     * Do not call this function while the read buffer is being written to.
     *
     * @tparam T The type of data to pop.
     * @return The data at the read buffer index.
     */
    template<typename T>
    T popReadBuffer()
    {
        T data             = *(T*)&read_buffer[m_ReadBufferIndex];
        m_ReadBufferIndex += sizeof(T);
        if (m_ReadBufferIndex > CIPO_BUFFER_LENGTH)
        {
            ROS_ERROR(
                "Read buffer overflow in RovCommsController::popReadBuffer()");
            m_ReadBufferIndex = 0;
            std::exit(EXIT_FAILURE);
        }
        return data;
    }

}; // class RovCommsController
