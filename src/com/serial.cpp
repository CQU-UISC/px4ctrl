#include "com/serial.hpp"
#include "com/crc.hpp"
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <thread>

// Constructor
SerialPort::SerialPort(std::string port, const int baudrate, int hz) : hz(hz) {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      "logs/serial.log", true);
  logger_ = std::make_shared<spdlog::logger>(
      "serial", spdlog::sinks_init_list{console_sink, file_sink});

  std::vector<serial::PortInfo> devices_found = serial::list_ports();
  std::vector<serial::PortInfo>::iterator iter = devices_found.begin();
  bool found = false;
  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;
    if (device.port == port) {
      found = true;
      break;
    }
    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
  if (!found) {
    logger_->error("Failed to create SerialPort. Port {} not found.", port);
    exit(1);
  }

  serial_ = std::make_shared<serial::Serial>(port, baudrate);
  // multiple sinks
  logger_->set_level(spdlog::level::info);
  logger_->info("SerialPort created with port: {} and baudrate: {}", port,
                baudrate);
}

// Destructor
SerialPort::~SerialPort() {
  should_exit_ = true;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  stop();
  logger_->info("SerialPort destroyed.");
}

// Start the serial port communication
bool SerialPort::start() {
  if (serial_->isOpen()) {
    should_exit_ = false;
    receive_thread_ = std::thread(&SerialPort::receive_t, this);
    send_thread_ = std::thread(&SerialPort::send_t, this);
    logger_->info("SerialPort communication started.");
    return true;
  }
  logger_->error(
      "Failed to start SerialPort communication. Serial port is not open.");
  return false;
}

// Stop the serial port communication
bool SerialPort::stop() {
  if (serial_->isOpen()) {
    serial_->close();
    logger_->info("SerialPort communication stopped.");
    return true;
  }
  logger_->warn("SerialPort communication was not running.");
  return false;
}

void SerialPort::send(const SwarmPacket &packet) {
  if (!serial_->isOpen()) {
    logger_->error("Failed to send data. Serial port is not open.");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    send_queue.push(packet);
  }
  return;
}

void SerialPort::send_t() {
  logger_->info("Send thread started.");
  while (!should_exit_) {
    SwarmPacket packet;
    // handle queue
    if (!send_queue.empty()) {
      {
        std::lock_guard<std::mutex> lock(send_mtx_);
        while (send_queue.size() > hz) {
          if (send_queue.front().dontdrop) {
            break;
          }
          send_queue.pop();
        }
        packet = send_queue.front();
        send_queue.pop();
      }

      if(packet.id!=0){
          std::unique_lock<std::mutex> lock(cv_mtx_);
          cv.wait(lock,[&](){return gcs_arrive;});
          gcs_arrive = false;
          lock.unlock();
      }

      // Add CRC and CRLF
      uint8_t buffer_[BS];
      memcpy(buffer_, &packet, sizeof(packet));
      int length = sizeof(packet);
      uint16_t crc = crc16::Get_CRC16_Check_Sum((unsigned char *)&packet,
                                                length - 2, 0xFFFF);
      *reinterpret_cast<uint16_t *>(buffer_ + length - 2) = crc;
      buffer_[length] = '\r';
      buffer_[length + 1] = '\n';
      length += 2;


      int time_span_width = 1000 / hz; // ms
      int sleep_for = time_span_width/10 * packet.id;
      // only send when time's  (1000ms/10 => 100ms/10 => 10ms) <=== id * 10ms
      // 10HZ
      // Send data
      // 
      std::chrono::high_resolution_clock::time_point next_t;
      if(packet.id!=0){
        next_t = last_received_time_ +  std::chrono::milliseconds(sleep_for);
      }else{
        next_t = std::chrono::high_resolution_clock::now()+std::chrono::milliseconds(time_span_width);
      }
      

      logger_->info("Sending packet with id: {}, at time: {}", packet.id,
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        next_t.time_since_epoch())
                        .count());
      std::this_thread::sleep_until(next_t);
      size_t bytes_written =
          serial_->write(reinterpret_cast<const uint8_t *>(buffer_), length);
      logger_->debug("Sent {} bytes to serial port.", bytes_written);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(50)); // 50 us
  }
  logger_->info("Send thread stoped.");
}

// Check if the serial port is open
bool SerialPort::isOpen() const { return serial_->isOpen(); }

// Add a callback function for receiving data
bool SerialPort::addReceiveCallback(
    std::function<void(const SwarmPacket &packet)> function) {
  std::lock_guard<std::mutex> lock(cb_mtx_);
  receive_callback_ = std::move(function);
  logger_->info("Receive callback added.");
  return true;
}

// Serial port reading thread function
void SerialPort::receive_t() {
  logger_->info("Receive thread started.");
  while (!should_exit_) {
    // read to buffer
    size_t bytes_read = serial_->read(
        reinterpret_cast<uint8_t *>(receive_buffer_ + received_length_),
        BS - received_length_);
    if (bytes_read > 0) {
      received_length_ += bytes_read;
      {
        logger_->info("Received {} bytes from serial port.", bytes_read);
        std::lock_guard<std::mutex> lock(rc_mtx_);
        last_received_time_ = std::chrono::high_resolution_clock::now();
      }
    }

    // find HEADER
    size_t header_pos = 0;
    for (size_t i = 0; i < received_length_; i++) {
      if (receive_buffer_[i] == START) {
        header_pos = i;
        break;
      }
    }
    if (header_pos > 0) {
      logger_->warn("Dropping {} bytes before START byte.", header_pos);
      received_length_ -= header_pos;
      memmove(receive_buffer_, receive_buffer_ + header_pos, received_length_);
    }

    // verify packet length
    if (received_length_ < sizeof(SwarmPacket)) {
      continue;
    }

    // find END
    size_t end_pos = 0;
    for (size_t i = 0; i < received_length_; i++) {
      if (receive_buffer_[i] == END) {
        end_pos = i;
        break;
      }
    }

    if (end_pos == 0) {
      // no END found&&received_length_ > sizeof(SwarmPacket)
      logger_->warn("Dropping {} bytes because can't parse.", received_length_);
      received_length_ = 0;
    }

    // parse packet
    if (end_pos > 0) {
      SwarmPacket packet;
      memcpy(&packet, receive_buffer_, sizeof(SwarmPacket)); // copy packet
      // check CRC
      uint16_t crc = crc16::Get_CRC16_Check_Sum((unsigned char *)&packet,
                                                sizeof(packet) - 2, 0xFFFF);
      if (crc == packet.checksum) {
        logger_->info("Received packet with id: {}", packet.id);
        if (receive_callback_ != nullptr) {
          std::lock_guard<std::mutex> lock(cb_mtx_);
          receive_callback_(packet);
        }

        if(packet.id==0){
          std::lock_guard<std::mutex> lock(cv_mtx_);
          gcs_arrive = true;
          cv.notify_all();
        }
       
      } else {
        logger_->error("CRC check failed. Dropping packet with id: {}",
                       packet.id);
      }
      received_length_ -= sizeof(SwarmPacket);
      memmove(receive_buffer_, receive_buffer_ + sizeof(SwarmPacket),
              received_length_);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(50)); // 50 us
  }
  logger_->info("Receive thread stoped.");
}
