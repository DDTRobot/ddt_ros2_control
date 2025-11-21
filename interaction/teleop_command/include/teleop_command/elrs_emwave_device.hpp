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

#ifndef TELEOP_COMMAND__ELRS_EMWAVE_DEVICE_HPP_
#define TELEOP_COMMAND__ELRS_EMWAVE_DEVICE_HPP_

#include <stdint.h>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */
#define DEBUG_CRSF (false)

// Basic setup
#define UART_BUFFER 512
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_CONNECTION_TIMEOUT 1000
#define RETURN_INFO_MAX_LENGTH 32

// Device address & type,
// The address means the destination of the data packet, so for decoder, the destination is the FC.
#define CRSF_TYPE_SETTINGS_WRITE 0x2D
#define CRSF_ADDRESS_MODULE 0xEE             // Crossfire transmitter
#define CRSF_ADDRESS_RADIO 0xEA              // Radio Transmitter
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8  // Flight Controler
#define CRSF_CUSTOMER_CMD (0X31)

// Define channel input limite
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 992
#define CRSF_CHANNEL_MAX 1810

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US 1100  // 700 ms + 400 ms for potential ad-hoc request
// At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz
#define CRSF_TIME_BETWEEN_FRAMES_US 6667
#define CRSF_PACKET_TIMEOUT_US 100000
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX 60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE 26
#define CRSF_FRAME_LENGTH 24  // length of type + payload + crc
#define CRSF_ROBOT_STATUS_ID_MIN (0x01)
#define CRSF_ROBOT_STATUS_ID_MAX (0x08)
#define CRSF_MAX_PAYLOAD_LENGTH (0x04)

#define CALIB_AXIS_MID (992)
#define CALIB_AXIS_MAX (1800)
#define CALIB_BTN_MID (800)

typedef enum {
  CRSF_CUSTOMER_CMD_WIFI_MODE = 0x01,
  CRSF_CUSTOMER_CMD_BINGING_MODE = 0x02,
  CRSF_CUSTOMER_CMD_REV_DATA = 0x03,
  CRSF_CUSTOMER_CMD_SET_STATUS = 0x04,
  CRSF_CUSTOMER_CMD_RECONNECT = 0x05,
  CRSF_CUSTOMER_CMD_DISCONNECT = 0x06,
  CRSF_CUSTOMER_CMD_IS_BINGDING = 0x07,
  CRSF_CUSTOMER_CMD_UART_LINKED = 0x08,
} crsf_customer_cmd;

class CRSF
{
private:
  uint8_t m_inBuffer[CRSF_PACKET_SIZE];
  uint8_t m_crsfData[CRSF_PACKET_SIZE];
  uint16_t m_channels[CRSF_MAX_CHANNEL];
  uint8_t m_uart_buffer[UART_BUFFER];
  uint8_t m_transfer_back_data[RETURN_INFO_MAX_LENGTH];
  int m_fd;
  int m_init;
  std::function<void(const uint16_t *)> m_dataReceivedCallback;
  void (*m_disconnectedCallback)();
  std::function<void(const uint8_t *)> m_modeReceivedCallback;
  inline static std::condition_variable wait_flag_;
  inline static std::mutex thread_mutex_;
  void updateChannels();
  uint8_t crsf_crc8(const uint8_t * ptr, uint8_t len) const;
  int setSerialOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
  uint16_t uartProtocol_CalCheckCrc(const uint8_t * buffer, uint8_t size);
  int dataTransferBack(uint8_t * payload, uint8_t id, uint8_t len);
  int uart_write(uint8_t * payload, uint8_t len);
  void upgradeTransferBackData(void);
  static void signal_handler_IO(int status);
  void threadFunction();
  void threadFunction1();
  void startThread();
  bool deinit();
  bool init();

public:
  explicit CRSF(std::string interface = "/dev/ttyTHS1");
  ~CRSF();
  void readPacket(uint8_t * inData, uint8_t len);
  void begin();
  uint16_t getChannel(uint8_t channel) const;
  bool isConnected();
  void onDataReceived(std::function<void(const uint16_t[])> callback);
  void onModeReceived(std::function<void(const uint8_t[])> callback);
  bool isUartConnected();
  void setTransferBackData(uint8_t * data, uint8_t offset, uint8_t len);
  std::string interface_;
};

#endif  // TELEOP_COMMAND__ELRS_EMWAVE_DEVICE_HPP_
