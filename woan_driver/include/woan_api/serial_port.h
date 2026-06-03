#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <array>
#include <mutex>
#include <memory>
#include <cstdint> // [修改1] 必须包含，否则 uint8_t 报错
#include <thread>  // 用于 sleep (如果需要)

class SerialPort {
public:
    using SharedPtr = std::shared_ptr<SerialPort>;

    // 构造函数
    // [注意] 在调用此构造函数时，baudrate 请直接传整数 921600
    // 不要传 Linux 的宏 B921600，否则 Windows 下编译失败
    SerialPort(std::string port_name, unsigned int baudrate, int timeout_ms = 1)
        : io_service_(), port_(io_service_), timeout_ms_(timeout_ms) {
        try {
            port_.open(port_name);
            port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            port_.set_option(boost::asio::serial_port_base::character_size(8));
            port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        } catch (boost::system::system_error& e) {
            std::cerr << "Error opening serial port (" << port_name << "): " << e.what() << std::endl;
        }
    }

    ~SerialPort() {
        if (port_.is_open()) {
            port_.cancel(); // 确保取消所有异步操作
            port_.close();
        }
    }

    // Check if the serial port is open and valid
    bool is_valid() const {
        return port_.is_open();
    }

size_t send(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!port_.is_open()) {
        // std::cerr << "串口未连接" << std::endl;
        return 0;
    }
    try {
        return boost::asio::write(port_, boost::asio::buffer(data, len));
    } catch (boost::system::system_error& e) {
        std::cerr << "Write error: " << e.what() << std::endl;
        return 0;
    }
}


    // 2 参数版本的接收（带超时的基础读取）
    size_t recv(uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t bytes_read = 0;
        boost::system::error_code ec = boost::asio::error::would_block;
        
        // 重置 io_service (旧版 Boost 写法，新版建议 io_context.restart())
        io_service_.reset();

        boost::asio::deadline_timer timer(io_service_);
        
        // 异步读取
        port_.async_read_some(boost::asio::buffer(data, len), 
            [&](const boost::system::error_code& error, size_t bytes) {
                ec = error;
                bytes_read = bytes;
            });

        // 异步超时计时器
        timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms_));
        timer.async_wait([&](const boost::system::error_code& error) {
            if (!error) { 
                port_.cancel(); // 超时则取消读取
            }
        });

        // 阻塞直到完成或超时
        while (io_service_.run_one()) {
            if (ec != boost::asio::error::would_block) {
                timer.cancel(); // 读取成功，取消计时器
            }
        }

        return bytes_read;
    }

    // ✅ [修改2] 增强版：循环读取直到匹配帧头和长度
    // 解决了 Windows 串口数据分片导致的“数据不全”问题
    void recv(uint8_t* data, uint8_t head, size_t len) {
        // 总超时控制 (简单计数器防止死循环)
        int retry_count = 0; 
        const int max_retries = 10; // 最多尝试读取 10 次分片

        while (retry_count < max_retries) {
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                
                // 1. 先尝试在队列中找帧头
                while (!recv_queue.empty() && recv_queue.front() != head) {
                    recv_queue.pop(); // 丢弃无效数据直到找到帧头
                }

                // 2. 如果找到了帧头，且数据长度足够
                if (!recv_queue.empty() && recv_queue.front() == head && recv_queue.size() >= len) {
                    for (size_t i = 0; i < len; i++) {
                        data[i] = recv_queue.front();
                        recv_queue.pop();
                    }
                    return; // 成功获取完整一帧，退出
                }
            } // 解锁，以便下面的 recv 可以写入

            // 3. 队列数据不够，从硬件读取更多数据
            uint8_t temp_buf[256];
            // 注意：这里复用上面的带超时的 recv
            size_t n = this->recv(temp_buf, sizeof(temp_buf)); 

            if (n > 0) {
                // 读取到了新数据，填入队列
                std::lock_guard<std::mutex> lock(queue_mutex_);
                for (size_t i = 0; i < n; i++) {
                    recv_queue.push(temp_buf[i]);
                }
            } else {
                // 如果这次没读到数据（超时了），增加重试计数
                retry_count++;
            }
        }
        
        // 如果退出循环，说明超时且未读到完整帧，data 数据可能无效
        // 实际使用建议抛出异常或返回 bool
    }

    // 获取内部 io_service (如果是旧代码依赖)
    boost::asio::io_service& get_io_service() { return io_service_; }

private:
    boost::asio::io_service io_service_;
    boost::asio::serial_port port_;
    int timeout_ms_;
    
    std::mutex mutex_;
    std::mutex queue_mutex_;
    std::queue<uint8_t> recv_queue;
};

#endif