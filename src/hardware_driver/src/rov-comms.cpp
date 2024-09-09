#include "rov-comms.hpp"

#include <cstdint>
#include <stdexcept>

void RovCommsController::awaitHandshake()
{
    // Clear the rx/tx buffer
    serial::serialEmpty(m_SerialFileDescriptor);

    ROS_INFO("Attempting Handshake using 0x%02x", COMMS_HANDSHAKE);

    while ((serial::serialDataAvail(m_SerialFileDescriptor) == 0) && ros::ok())
    {
        // Try sending the handshake command
        // NOTE: This is doing double duty, since it's also what clears the
        // arduino's serial buffer if it's waiting for data in COPI mode
        printf(".");
        fflush(stdout);
        serial::serialPutchar(m_SerialFileDescriptor, COMMS_HANDSHAKE);
        serial::serialFlush(m_SerialFileDescriptor);

        // I chose 50ms since the timestamp does a roundtrip within about 20ms.
        ros::Duration(0.1).sleep();
    }
    printf("\n");

    std::uint32_t timestamp = 0;

    for (int i = 0; i < 4 && ros::ok(); i++)
    {
        while ((serial::serialDataAvail(m_SerialFileDescriptor) == 0)
               && ros::ok())
            ; // Wait for more data
        timestamp |= serial::serialGetchar(m_SerialFileDescriptor) << (i * 8);
    }

    if (!ros::ok())
    {
        throw std::runtime_error("ros::ok() false while awaiting handshake.");
    }

    for (int i = 0; i < 4; i++)
    {
        serial::serialPutchar(m_SerialFileDescriptor,
                              (timestamp >> (i * 8)) & 0xFF);
    }
    ROS_INFO("Handshake completed (timestamp %d)", timestamp);
}

void RovCommsController::sendChecksum()
{
    serial::serialWrite(m_SerialFileDescriptor, m_CopiChecksum);
    serial::serialFlush(m_SerialFileDescriptor);

    m_CopiChecksum = 0;
}; // sendChecksum

void RovCommsController::sendBlock(uint8_t data)
{
    m_CopiChecksum += data;
    serial::serialWrite(m_SerialFileDescriptor, data);

    ROS_DEBUG("COPI @%d: 0x%02X", m_CopiIndex, data);

    m_CopiIndex++;

    if (m_CopiIndex >= COPI_BUFFER_LENGTH)
    {
        this->sendChecksum();
        m_CopiIndex = 0;
    }

}; // sendBlock

void RovCommsController::sendBlocks(const std::uint8_t data[],
                                    const std::size_t  length)
{
    for (size_t i = 0; i < length; i++)
    {
        this->sendBlock(data[i]);
    }
}; // sendBlocks

int RovCommsController::tryReadingData()
{
    std::uint8_t data;
    while (serial::serialDataAvail(m_SerialFileDescriptor))
    {
        data = serial::serialGetchar(m_SerialFileDescriptor);

        ROS_DEBUG("CIPO @%d: 0x%02X", m_CipoIndex, data);

        if (m_CipoIndex < CIPO_BUFFER_LENGTH)
        {
            // Store everything but the checksum
            read_buffer[m_CipoIndex]  = data;
            m_CipoChecksum           += data;
        }
        else
        {
            // We've read the entire buffer, verify the checksum
            m_CipoChecksumStatus = (m_CipoChecksum == data);

            if (!m_CipoChecksumStatus)
            {
                ROS_INFO("FAILED: Read checksum 0x%02X (expected 0x%02X)",
                         data,
                         m_CipoChecksum);
            }

            m_CipoIndex    = 0;
            m_CipoChecksum = 0;

            return -1;
        }
        m_CipoIndex++;
    }
    // We haven't read the entire buffer yet
    return m_CipoIndex;
}; // readBlocks

uint8_t* RovCommsController::popReadBuffer(size_t length)
{
    std::uint8_t* data  = &read_buffer[m_ReadBufferIndex];
    m_ReadBufferIndex  += length;
    if (m_ReadBufferIndex > CIPO_BUFFER_LENGTH)
    {
        ROS_ERROR(
            "Read buffer overflow in RovCommsController::popReadBuffer()");
        m_ReadBufferIndex = 0;
        exit(EXIT_FAILURE);
    }
    return data;
}

std::uint8_t RovCommsController::popReadBuffer()
{
    std::uint8_t data = read_buffer[m_ReadBufferIndex++];
    if (m_ReadBufferIndex > CIPO_BUFFER_LENGTH)
    {
        ROS_ERROR(
            "Read buffer overflow in RovCommsController::popReadBuffer()");
        m_ReadBufferIndex = 0;
        exit(EXIT_FAILURE);
    }
    return data;
}
