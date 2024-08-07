#pragma once
//FROM: https://github.com/uzh-rpg
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <spdlog/spdlog.h>
#include <serial/serial.h>

//8-bit flag
const uint8_t START = 0xFA;
const uint8_t END = 0xFB;

const uint8_t NEEDACK = 0b00000001;
const uint8_t NOACK = 0b00000010;
const uint8_t ACK = 0b00000100;
const uint8_t BROADCAST = 0xFF;

struct SwarmPacket
{
  uint8_t header = START;
  uint8_t flag = 0;
  uint8_t id = 0;
  uint8_t recv_id = 0;//if recv_id != self_id, drop && if recv_id == 0xFF, recv
  uint8_t msg_id;//message id for ack
  uint8_t msg_type;//message type
  uint8_t data[128];//data
  uint8_t end = END;
  uint16_t checksum = 0;     // crc16
} __attribute__((packed));


class SerialPort {
 public:
  explicit SerialPort(std::string port, const int baudrate);
  ~SerialPort();

  bool start();
  bool stop();
  bool isOpen() const;

  int send(const SwarmPacket& packet);
  bool addReceiveCallback(std::function<void(const SwarmPacket& packet)> function);
  
  static constexpr int BUFFER_SIZE = 1024;

 private:
  std::shared_ptr<serial::Serial > serial_;

  void serialThread();

  std::function<void(const SwarmPacket& packet)> receive_callback_;
  bool should_exit_{false};

  std::thread serial_thread_;

  static constexpr size_t BS = BUFFER_SIZE;

  //Double buffer for receiving data
  uint8_t receive_buffer_[BS];
  size_t received_length_{0u};

  std::chrono::high_resolution_clock::time_point last_received_time_;

  std::mutex rc_mtx_;
  std::mutex buffer_mtx;

  std::shared_ptr<spdlog::logger> logger_;
};
