#include "rov-comms.hpp"

void RovCommsController::awaitHandshake()
{
  // Clear the rx/tx buffer
  serial::serialEmpty(serial_fd);

  ROS_INFO("Attempting Handshake using 0x%02x", COMMS_HANDSHAKE);

  while ((serial::serialDataAvail(serial_fd) == 0) && ros::ok())
  {
    // Try sending the handshake commmand
    // NOTE: This is doing double duty, since it's also what clears the
    // arduino's serial buffer if it's waiting for data in COPI mode
    printf(".");
    fflush(stdout);
    serial::serialPutchar(serial_fd, COMMS_HANDSHAKE);
    serial::serialFlush(serial_fd);

    // I chose 50ms since the timestamp does a roundtrip within about 20ms.
    ros::Duration(0.1).sleep();
  }
  printf("\n");

  uint32_t timestamp = 0;

  for (int i = 0; i < 4 && ros::ok(); i++)
  {
    while ((serial::serialDataAvail(serial_fd) == 0) && ros::ok())
      ; // Wait for more data
    timestamp |= serial::serialGetchar(serial_fd) << (i * 8);
  }

  if (!ros::ok())
  {
    throw std::runtime_error("ros::ok() false while awaiting handshake.");
  }

  for (int i = 0; i < 4; i++)
  {
    serial::serialPutchar(serial_fd, (timestamp >> (i * 8)) & 0xFF);
  }
  ROS_INFO("Handshake completed (timestamp %d)", timestamp);
}

void RovCommsController::sendChecksum()
{
  serial::serialWrite(serial_fd, copi_checksum);
  serial::serialFlush(serial_fd);

  copi_checksum = 0;
}; // sendChecksum

void RovCommsController::sendBlock(uint8_t data)
{
  copi_checksum += data;
  serial::serialWrite(serial_fd, data);

  ROS_DEBUG("COPI @%d: 0x%02X", copi_index, data);

  copi_index++;

  if (copi_index >= COPI_BUFFER_LENGTH)
  {
    this->sendChecksum();
    copi_index = 0;
  }

}; // sendBlock

void RovCommsController::sendBlocks(const uint8_t data[], size_t length)
{
  for (size_t i = 0; i < length; i++)
    this->sendBlock(data[i]);
}; // sendBlocks

int RovCommsController::tryReadingData()
{
  uint8_t data;
  while (serial::serialDataAvail(serial_fd))
  {
    data = serial::serialGetchar(serial_fd);

    ROS_DEBUG("CIPO @%d: 0x%02X", cipo_index, data);

    if (cipo_index < CIPO_BUFFER_LENGTH)
    {
      // Store everything but the checksum
      read_buffer[cipo_index] = data;
      cipo_checksum += data;
    }
    else
    {
      // We've read the entire buffer, verify the checksum
      cipo_checksum_status = (cipo_checksum == data);
      
      if (!cipo_checksum_status)
        ROS_INFO("FAILED: Read checksum 0x%02X (expected 0x%02X)", data, cipo_checksum);

      cipo_index = 0;
      cipo_checksum = 0;

      return -1;
    }
    cipo_index++;
  }
  // We haven't read the entire buffer yet
  return cipo_index;
}; // readBlocks

uint8_t *RovCommsController::popReadBuffer(size_t length)
{
  uint8_t *data = &read_buffer[read_buffer_index];
  read_buffer_index += length;
  if (read_buffer_index > CIPO_BUFFER_LENGTH)
  {
    ROS_ERROR("Read buffer overflow in RovCommsController::popReadBuffer()");
    read_buffer_index = 0;
    exit(EXIT_FAILURE);
  }
  return data;
}

uint8_t RovCommsController::popReadBuffer()
{
  uint8_t data = read_buffer[read_buffer_index++];
  if (read_buffer_index > CIPO_BUFFER_LENGTH)
  {
    ROS_ERROR("Read buffer overflow in RovCommsController::popReadBuffer()");
    read_buffer_index = 0;
    exit(EXIT_FAILURE);
  }
  return data;
}