#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <thread>
#include "crc.hpp"
#include "serial.hpp"

// Constructor
SerialPort::SerialPort(std::string port, const int baudrate) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/serial.log", true);
    logger_ = std::make_shared<spdlog::logger>("serial", spdlog::sinks_init_list{console_sink, file_sink});
    //
    std::vector<serial::PortInfo> devices_found = serial::list_ports();
	std::vector<serial::PortInfo>::iterator iter = devices_found.begin();
	bool found = false;
    while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;
        if(device.port == port){
            found = true;
            break;
        }
		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
    }
    if(!found){
        logger_->error("Failed to create SerialPort. Port {} not found.", port);
        exit(1);
    }

    serial_ = std::make_shared<serial::Serial>(port, baudrate);
    //multiple sinks
    logger_->set_level(spdlog::level::info);
    logger_->info("SerialPort created with port: {} and baudrate: {}", port, baudrate);
}

// Destructor
SerialPort::~SerialPort() {
    stop();
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    logger_->info("SerialPort destroyed.");
}

// Start the serial port communication
bool SerialPort::start() {
    if (serial_->isOpen()) {
        should_exit_ = false;
        serial_thread_ = std::thread(&SerialPort::serialThread, this);
        logger_->info("SerialPort communication started.");
        return true;
    }
    logger_->error("Failed to start SerialPort communication. Serial port is not open.");
    return false;
}

// Stop the serial port communication
bool SerialPort::stop() {
    if (serial_->isOpen()) {
        should_exit_ = true;
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
        serial_->close();
        logger_->info("SerialPort communication stopped.");
        return true;
    }
    logger_->warn("SerialPort communication was not running.");
    return false;
}

// Send data to the serial port using CSMA-CA
int SerialPort::send(const SwarmPacket &packet) {
    if (!serial_->isOpen()) {
    logger_->error("Failed to send data. Serial port is not open.");
    return -1;
    }

    // Add CRC and CRLF
    uint8_t buffer_[BS];
    memcpy(buffer_, &packet, sizeof(packet));
    int length = sizeof(packet);
    uint16_t  crc = crc16::Get_CRC16_Check_Sum((unsigned char*)&packet,length-2,0xFFFF);
    *reinterpret_cast<uint16_t*>(buffer_+length-2) =  crc;
    buffer_[length] = '\r';
    buffer_[length+1] = '\n';
    length += 2;

    // Implementing CSMA-CA
    const int max_retries = 5;
    int retry_count = 0;

    while (retry_count < max_retries) {
        // Step 1: Clear to Send (CTS)
        {
            std::unique_lock<std::mutex> lock(rc_mtx_);
            long last_recv_time_past = std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::high_resolution_clock::now()-last_received_time_)).count();
            if (last_recv_time_past > 10) {
                // No data being received, channel is clear
                break;
            }
        }
        // Step 2: Wait for a random backoff time
        std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 5 + 10));
        retry_count++;
    }

    if (retry_count == max_retries) {
        logger_->error("Failed to send data. Channel busy after {} retries.", max_retries);
        return -1;
    }


    // Step 3: Random delay 5~10ms
    std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 5 + 5));

    // only send when time's  (1000ms/10 => 100ms/10 => 10ms) <=== id * 10ms
    // 10HZ
    // Send data
    auto t = std::chrono::high_resolution_clock::now();
    long time_span = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
    int cur = (time_span%1000)/100;//0~9
    int next = 10-cur + packet.id;
    auto next_t = t + std::chrono::milliseconds(next*100);
    if( next != 10){
        std::this_thread::sleep_until(next_t);
    }
    logger_->info("Sending packet with id: {}, at time: {}", packet.id, std::chrono::duration_cast<std::chrono::milliseconds>(next_t.time_since_epoch()).count());
    size_t bytes_written = serial_->write(reinterpret_cast<const uint8_t*>(buffer_), length);
    logger_->debug("Sent {} bytes to serial port.", bytes_written);
    return static_cast<int>(bytes_written);
}

// Check if the serial port is open
bool SerialPort::isOpen() const {
    return serial_->isOpen();
}

// Add a callback function for receiving data
bool SerialPort::addReceiveCallback(std::function<void(const SwarmPacket& packet)> function) {
    std::lock_guard<std::mutex> lock(buffer_mtx);
    receive_callback_ = std::move(function);
    logger_->info("Receive callback added.");
    return true;
}

// Serial port reading thread function
void SerialPort::serialThread() {
    logger_->info("Serial thread started.");
    while (!should_exit_) {
        //read to buffer
        size_t bytes_read = serial_->read(reinterpret_cast<uint8_t*>(receive_buffer_+received_length_), BS - received_length_);
        if (bytes_read > 0) {
            received_length_ += bytes_read;
            {
                logger_->info("Received {} bytes from serial port.", bytes_read);
                std::lock_guard<std::mutex> lock(rc_mtx_);
                last_received_time_ = std::chrono::high_resolution_clock::now();
            }
        }
        
        //find HEADER
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

        //verify packet length
        if(received_length_ <  sizeof(SwarmPacket)){
            continue;
        }

        //find END
        size_t end_pos = 0;
        for (size_t i = 0; i < received_length_; i++) {
            if (receive_buffer_[i] == END) {
                end_pos = i;
                break;
            }
        }

        if (end_pos == 0) {
            //no END found&&received_length_ > sizeof(SwarmPacket)
            logger_->warn("Dropping {} bytes because can't parse.", received_length_);
            received_length_ = 0;
        }

        //parse packet
        if (end_pos > 0) {
            SwarmPacket packet;
            memcpy(&packet, receive_buffer_, sizeof(SwarmPacket));//copy packet
            //check CRC
            uint16_t crc = crc16::Get_CRC16_Check_Sum((unsigned char*)&packet,sizeof(packet)-2,0xFFFF);
            if (crc == packet.checksum) {
                logger_->info("Received packet with id: {}", packet.id);
                {
                    std::lock_guard<std::mutex> lock(buffer_mtx);
                    receive_callback_(packet);
                }
            } else {
                logger_->error("CRC check failed. Dropping packet with id: {}", packet.id);
            }
            received_length_ -= sizeof(SwarmPacket);
            memmove(receive_buffer_, receive_buffer_ + sizeof(SwarmPacket), received_length_);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}
