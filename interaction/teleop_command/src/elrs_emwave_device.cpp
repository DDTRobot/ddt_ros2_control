// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "teleop_command/elrs_emwave_device.hpp"

#include <asm-generic/termbits.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <signal.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>

CRSF::CRSF(std::string interface) { this->interface_ = interface; }

CRSF::~CRSF()
{
  deinit();
  m_init = false;
  wait_flag_.notify_all();
  close(m_fd);
}

void CRSF::begin() { init(); }

void CRSF::threadFunction()
{
  std::unique_lock<std::mutex> lck(thread_mutex_);
  while (m_init) {
    if (
      wait_flag_.wait_until(lck, std::chrono::system_clock::now() + std::chrono::seconds(1)) ==
      std::cv_status::timeout) {
      continue;
    }
    memset(m_uart_buffer, 0, UART_BUFFER);

    int rx_len = read(m_fd, m_uart_buffer, UART_BUFFER);

    if (rx_len >= CRSF_PACKET_SIZE) {
      for (int i = 0; i <= rx_len - CRSF_PACKET_SIZE; i++) {
        if (
          m_uart_buffer[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER &&
          m_uart_buffer[i + 1] == CRSF_FRAME_LENGTH) {
          readPacket(&m_uart_buffer[i], CRSF_PACKET_SIZE);
          continue;
        }
      }

      for (int i = 0; i <= rx_len - CRSF_PACKET_SIZE; i++) {
        if (
          m_uart_buffer[i] == CRSF_CUSTOMER_CMD &&
          m_uart_buffer[i + 1] == CRSF_CUSTOMER_CMD_REV_DATA) {
          readPacket(&m_uart_buffer[i], CRSF_PACKET_SIZE);
          continue;
        }
      }
    }
  }
}

void CRSF::threadFunction1()
{
  while (true) {
    usleep(1000);
    dataTransferBack(&m_transfer_back_data[8], 3, 4);
    usleep(1000);
    dataTransferBack(&m_transfer_back_data[0], 1, 4);
    usleep(1000);
    dataTransferBack(&m_transfer_back_data[4], 2, 4);
  }
}

void CRSF::startThread()
{
  std::thread t([this]() { this->threadFunction(); });
  t.detach();
  std::thread t1([this]() { this->threadFunction1(); });
  t1.detach();
}

void CRSF::signal_handler_IO(int status)
{
  (void)status;
  sigset_t mask;
  sigemptyset(&mask);
  sigaddset(&mask, SIGIO);
  sigprocmask(SIG_BLOCK, &mask, nullptr);

  wait_flag_.notify_all();

  sigprocmask(SIG_UNBLOCK, &mask, nullptr);
}

int CRSF::setSerialOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
  struct termios2 newtio, oldtio;
  if (0 != ioctl(fd, TCGETS2, &oldtio)) {
    close(fd);
    fd = -1;
    perror("ioctl failded");
    return -1;
  }

  memset(&newtio, 0, sizeof(newtio));

  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  switch (nBits) {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }

  switch (nEvent) {
    case 'O':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= INPCK;
      break;
    case 'E':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      newtio.c_iflag |= INPCK;
      break;
    case 'N':
      newtio.c_cflag &= ~PARENB;
      break;
  }

  switch (nSpeed) {
    case 2400:
    case 4800:
    case 9600:
    case 38400:
    case 57600:
    case 115200:
      newtio.c_cflag |= CBAUDEX;
      newtio.c_ispeed = nSpeed;
      newtio.c_ospeed = nSpeed;
      break;
    default:
      newtio.c_cflag |= BOTHER;
      newtio.c_ispeed = nSpeed;
      newtio.c_ospeed = nSpeed;
      break;
  }

  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;

  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 30;
  ioctl(fd, TCIFLUSH, 2);

  if (0 != ioctl(fd, TCSETS2, &newtio)) {
    close(fd);
    fd = -1;
    perror("ioctl failded");
    return -1;
  }

  return 0;
}

bool CRSF::init()
{
  struct sigaction saio;

  m_fd = open(this->interface_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (setSerialOpt(m_fd, 420000, 8, 'N', 1) == -1) {
    return false;
  }

  saio.sa_handler = signal_handler_IO;

  sigemptyset(&saio.sa_mask);

  saio.sa_flags = SA_NODEFER;

  saio.sa_restorer = NULL;

  sigaction(SIGIO, &saio, NULL);

  fcntl(m_fd, F_SETOWN, getpid());

  fcntl(m_fd, F_SETFL, FASYNC);

  m_init = true;

  startThread();

  return true;
}

bool CRSF::deinit()
{
  int result = 0;

  if (m_init) {
    result = close(m_fd);
  }

  if (result == -1) {
    return false;
  }

  return true;
}

void CRSF::readPacket(uint8_t * input_data, uint8_t len)
{
  uint8_t frame_length = 0;
  uint8_t buffer_index = 0;

  if (len < CRSF_PACKET_SIZE) {
    return;
  }

  if (input_data[0] == CRSF_CUSTOMER_CMD) {
    if (input_data[1] == CRSF_CUSTOMER_CMD_REV_DATA) {
      m_modeReceivedCallback(&input_data[2]);
      return;
    }
  }

  if (buffer_index == 0) {
    if (input_data[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
      m_inBuffer[buffer_index++] = input_data[0];
      frame_length = input_data[1];
      m_inBuffer[buffer_index++] = input_data[1];
    } else {
      buffer_index = 0;
    }
  }

  if (buffer_index > 1 && buffer_index < frame_length + 1) {
    for (uint8_t i = buffer_index; i < frame_length + 1; i++) {
      m_inBuffer[buffer_index++] = input_data[i];
    }
  }

  if (buffer_index == frame_length + 1) {
    m_inBuffer[buffer_index++] = input_data[25];
    uint8_t crc = 0;

    if (m_inBuffer[1] < 25) {
      crc = crsf_crc8(&m_inBuffer[2], m_inBuffer[1] - 1);
    }

    m_inBuffer[24] = crc;

    if (frame_length == CRSF_FRAME_LENGTH && m_inBuffer[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
      if (crc == m_inBuffer[25]) {
        memcpy(m_crsfData, m_inBuffer, CRSF_PACKET_SIZE);
        updateChannels();
      }
    }
  }
}

uint16_t CRSF::getChannel(uint8_t channel) const { return m_channels[channel - 1]; }

void CRSF::updateChannels()
{
  if (m_crsfData[1] == 24) {
    m_channels[0] = ((m_crsfData[3] | m_crsfData[4] << 8) & 0x07FF);
    m_channels[1] = ((m_crsfData[4] >> 3 | m_crsfData[5] << 5) & 0x07FF);
    m_channels[2] = ((m_crsfData[5] >> 6 | m_crsfData[6] << 2 | m_crsfData[7] << 10) & 0x07FF);
    m_channels[3] = ((m_crsfData[7] >> 1 | m_crsfData[8] << 7) & 0x07FF);
    m_channels[4] = ((m_crsfData[8] >> 4 | m_crsfData[9] << 4) & 0x07FF);
    m_channels[5] = ((m_crsfData[9] >> 7 | m_crsfData[10] << 1 | m_crsfData[11] << 9) & 0x07FF);
    m_channels[6] = ((m_crsfData[11] >> 2 | m_crsfData[12] << 6) & 0x07FF);
    m_channels[7] = ((m_crsfData[12] >> 5 | m_crsfData[13] << 3) & 0x07FF);
    m_channels[8] = ((m_crsfData[14] | m_crsfData[15] << 8) & 0x07FF);
    m_channels[9] = ((m_crsfData[15] >> 3 | m_crsfData[16] << 5) & 0x07FF);
    m_channels[10] = ((m_crsfData[16] >> 6 | m_crsfData[17] << 2 | m_crsfData[18] << 10) & 0x07FF);
    m_channels[11] = ((m_crsfData[18] >> 1 | m_crsfData[19] << 7) & 0x07FF);
    m_channels[12] = ((m_crsfData[19] >> 4 | m_crsfData[20] << 4) & 0x07FF);
    m_channels[13] = ((m_crsfData[20] >> 7 | m_crsfData[21] << 1 | m_crsfData[22] << 9) & 0x07FF);
    m_channels[14] = ((m_crsfData[22] >> 2 | m_crsfData[23] << 6) & 0x07FF);
    m_channels[15] = ((m_crsfData[23] >> 5 | m_crsfData[24] << 3) & 0x07FF);
  }

  m_dataReceivedCallback(m_channels);
}

uint8_t CRSF::crsf_crc8(const uint8_t * ptr, uint8_t len) const
{
  static const uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = crsf_crc8tab[crc ^ *ptr++];
  }
  return crc;
}

void CRSF::onDataReceived(std::function<void(const uint16_t *)> callback)
{
  m_dataReceivedCallback = callback;
}

void CRSF::onModeReceived(std::function<void(const uint8_t *)> callback)
{
  m_modeReceivedCallback = callback;
}

uint16_t CRSF::uartProtocol_CalCheckCrc(const uint8_t * buffer, uint8_t size)
{
  uint16_t crc = 0xFFFF;

  if (NULL != buffer && size > 0) {
    while (size--) {
      crc = (crc >> 8) | (crc << 8);
      crc ^= *buffer++;
      crc ^= ((unsigned char)crc) >> 4;
      crc ^= crc << 12;
      crc ^= (crc & 0xFF) << 5;
    }
  }
  return crc;
}

int CRSF::uart_write(uint8_t * payload, uint8_t len) { return write(m_fd, payload, len); }

/**********************************************************
  Head  Type    status_id          payload（nbyte)   CRC
  0x31   0x04    1byte           4bytes       2bytes
**********************************************************/
int CRSF::dataTransferBack(uint8_t * payload, uint8_t id, uint8_t len)
{
  uint8_t status_data[CRSF_MAX_PAYLOAD_LENGTH + 6] = {0};
  uint16_t crc_cal = 0;

  if (payload == NULL) {
    printf("set_robot_status null pointer");
  }

  if (len > CRSF_MAX_PAYLOAD_LENGTH) {
    printf("set_robot_status len overflow!!!\n");
    return -1;
  }

  status_data[0] = CRSF_CUSTOMER_CMD;
  status_data[1] = CRSF_CUSTOMER_CMD_SET_STATUS;
  status_data[2] = id;
  memcpy(&status_data[3], payload, len);
  crc_cal = uartProtocol_CalCheckCrc(&status_data[2], len + 1);
  status_data[len + 3] = crc_cal >> 8 & 0xff;
  status_data[len + 4] = crc_cal & 0xff;
  return uart_write(status_data, len + 5);
}

void CRSF::setTransferBackData(uint8_t * data, uint8_t offset, uint8_t len)
{
  if (len + offset > RETURN_INFO_MAX_LENGTH) {
    printf("setTransferBackData len overflow!!!\n");
    return;
  } else {
    memcpy(&m_transfer_back_data[offset], data, len);
  }
}

void CRSF::upgradeTransferBackData(void)
{
  for (int i = CRSF_ROBOT_STATUS_ID_MIN; i <= CRSF_ROBOT_STATUS_ID_MAX; i++) {
    dataTransferBack(&m_transfer_back_data[(i - 1) * 4], i, 4);
  }
}

bool CRSF::isConnected() { return true; }

bool CRSF::isUartConnected() { return true; }
