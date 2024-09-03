#pragma once
//FROM: https://github.com/uzh-rpg
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <spdlog/spdlog.h>
#include <serial/serial.h>

//8-bit flag
const uint8_t START = 0xFA;
const uint8_t END = 0xFB;

const uint8_t MSG_DRONE = 0x01;
const uint8_t MSG_GCS = 0x02;

struct SwarmPacket
{
  uint8_t header = START;
  uint8_t id = 0;//time span id
  uint8_t dontdrop = 0;
  uint8_t msg_type;//message type
  uint8_t data[128];//data
  uint8_t end = END;
  uint16_t checksum = 0;     // crc16
} __attribute__((packed));


class SerialPort {
 public:
  explicit SerialPort(std::string port, const int baudrate, int hz);
  ~SerialPort();

  bool start();
  bool stop();
  bool isOpen() const;

  void send(const SwarmPacket& packet);
  bool addReceiveCallback(std::function<void(const SwarmPacket& packet)> function);
  
  static constexpr int BUFFER_SIZE = 1024;

 private:
  //serial port
  std::shared_ptr<serial::Serial > serial_;

  //send queue
  int hz;
  std::queue<SwarmPacket> send_queue;//queue size = rate (Hz)

  //send && recv loop
  std::thread receive_thread_;
  void receive_t();

  std::thread send_thread_;
  void send_t();

  //exit loop
  bool should_exit_{false};

  //receive
  std::function<void(const SwarmPacket& packet)> receive_callback_;



  //buffer
  static constexpr size_t BS = BUFFER_SIZE;
  uint8_t receive_buffer_[BS];
  size_t received_length_{0u};

  //for ca
  std::chrono::high_resolution_clock::time_point last_received_time_;
  std::condition_variable cv;
  std::mutex cv_mtx_;
  bool gcs_arrive = false;

  //mutex
  std::mutex rc_mtx_;
  std::mutex cb_mtx_;
  std::mutex send_mtx_;

  //logger
  std::shared_ptr<spdlog::logger> logger_;
};
