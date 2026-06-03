#pragma once

#include <memory>      // std::shared_ptr
#include <vector>      // std::vector
#include <string>      // std::string, std::stoi, std::to_string
#include <cstdio>      // snprintf
#include <thread>      // std::this_thread::sleep_for
#include <chrono>      // std::chrono::milliseconds
#include <cstring>
#include "serial_port.h"  // SerialPort 类（需要确认实际路径）

#ifdef _WIN32
#include <basetsd.h>
typedef SSIZE_T ssize_t;
#endif

class SlcanProtocolBase
{
public:
    using Ptr = std::shared_ptr<SlcanProtocolBase>;
    virtual ~SlcanProtocolBase() = default;

    virtual void init() = 0;
    virtual bool send(uint16_t frame_id, const std::vector<uint8_t>& data) = 0;
    virtual bool recv(uint16_t& frame_id, std::vector<uint8_t>& data) = 0;
};

class CanableSlcan : public SlcanProtocolBase
{
public:
    explicit CanableSlcan(std::shared_ptr<SerialPort> serial)
        : serial_(std::move(serial)) {}

    void init() override
    {
        // ASCII SLCAN 初始化序列
        serial_->send((uint8_t*)"C\r", 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        serial_->send((uint8_t*)"S8\r", 3); // 1M
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        serial_->send((uint8_t*)"O\r", 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool send(uint16_t frame_id, const std::vector<uint8_t>& data) override
    {
        std::string cmd = "t" + int_to_hex(frame_id, 3) +
                          std::to_string(data.size()) +
                          bytes_to_hex_string(data) + "\r";
        return serial_->send((uint8_t*)cmd.c_str(), cmd.size()) == (ssize_t)cmd.size();
    }

    bool recv(uint16_t& frame_id, std::vector<uint8_t>& data) override
    {
        uint8_t buf[128];
        ssize_t len = serial_->recv(buf, sizeof(buf));
        if (len <= 0) return false;

        rx_buffer_.insert(rx_buffer_.end(), buf, buf + len);

        // 查找一整行
        auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), '\r');
        if (it == rx_buffer_.end())
            return false;

        std::string line(rx_buffer_.begin(), it);
        rx_buffer_.erase(rx_buffer_.begin(), it + 1);
        // std::cout << "Received canable line: " << line << std::endl;
        return parse_frame(line, frame_id, data);
    }

private:
    std::shared_ptr<SerialPort> serial_;
    std::vector<uint8_t> rx_buffer_;

    bool parse_frame(const std::string& line,
                     uint16_t& frame_id,
                     std::vector<uint8_t>& data)
    {
        // tIIILDATA (t + 3位ID + 1位长度 + 2*L个十六进制字符)
        if (line.size() < 5 || line[0] != 't')
            return false;

        // 解析 ID (3位十六进制)
        uint8_t id_h = hex2nibble(line[1]);
        uint8_t id_m = hex2nibble(line[2]);
        uint8_t id_l = hex2nibble(line[3]);
        if (id_h == 0xFF || id_m == 0xFF || id_l == 0xFF)
            return false;

        frame_id = (id_h << 8) | (id_m << 4) | id_l;

        // 解析 DLC
        uint8_t dlc = line[4] - '0';
        if (dlc > 8) return false;

        // 验证总长度
        if (line.size() != 5 + dlc * 2)
            return false;

        // 解析数据
        data.clear();
        data.reserve(dlc);

        for (size_t i = 0; i < dlc; ++i)
        {
            uint8_t hi = hex2nibble(line[5 + i*2]);
            uint8_t lo = hex2nibble(line[6 + i*2]);
            if (hi == 0xFF || lo == 0xFF)
                return false;
            data.push_back((hi << 4) | lo);
        }

        return true;
    }

    static uint8_t hex2nibble(char c)
    {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return 0xFF; // 非法字符
    }

    static std::string int_to_hex(int value, int width)
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%0*X", width, value);
        return std::string(buf);
    }

    static std::string bytes_to_hex_string(const std::vector<uint8_t>& data)
    {
        std::string s;
        char buf[3];
        for (auto b : data)
        {
            snprintf(buf, sizeof(buf), "%02X", b);
            s += buf;
        }
        return s;
    }
};

class DamiaoSlcan : public SlcanProtocolBase
{
public:
    DamiaoSlcan(std::shared_ptr<SerialPort> serial) : serial_(serial) {}

    void init() override
    {
        // 达妙不需要额外初始化
    }

    bool send(uint16_t frame_id, const std::vector<uint8_t>& data) override
    {
        // 1. 实例化结构体，利用其默认构造函数初始化 (自动填充帧头 55 AA, CMD 03 等)
        can_send_frame tx_frame; 
        
        // 2. 填充 ID 和 数据
        // 注意：结构体内部的 set 函数可以复用，或者手动赋值
        tx_frame.canId = frame_id;
        
        size_t len = data.size() > 8 ? 8 : data.size();
        std::memcpy(tx_frame.data, data.data(), len);

        // 3. 发送整个结构体
        // sizeof(tx_frame) 应该是 30 字节左右，加上结尾可能并没有填满32字节
        // 达妙协议通常要求定长32字节或者根据FrameLen决定。
        // 如果协议严格要求32字节，需要补齐。这里假设发送结构体大小即可，
        // 或者保留原有的 32 字节 buffer 逻辑，用 memcpy 覆盖。
        
        return serial_->send((uint8_t*)&tx_frame, sizeof(tx_frame)) == sizeof(tx_frame);
    }

    bool recv(uint16_t& frame_id, std::vector<uint8_t>& data) override
    {
        // 1. 直接读取到结构体中 (或者先读buffer再转)
        // 达妙反馈帧固定 16 字节
        uint8_t buf[16] = {0};
        ssize_t len = serial_->recv(buf, 16);
        if (len != 16) return false;

        // 强转为结构体指针方便访问
        CAN_Receive_Frame* rx_frame = (CAN_Receive_Frame*)buf;

        // 2. 校验帧头帧尾
        if (rx_frame->FrameHeader != 0xAA || rx_frame->frameEnd != 0x55) return false;

        // 3. 校验 CMD
        if (rx_frame->CMD != 0x11 && rx_frame->CMD != 0xEE) return false;

        // 4. 提取 ID
        frame_id = (uint16_t)rx_frame->canId;

        // 5. 提取数据
        data.assign(rx_frame->canData, rx_frame->canData + 8);

        return true;
    }

private:
    std::shared_ptr<SerialPort> serial_;

#pragma pack(push, 1)
    typedef struct can_send_frame
    {
        uint8_t FrameHeader[2] = {0x55, 0xAA}; // 默认值生效
        uint8_t FrameLen = 0x1e; 
        uint8_t CMD = 0x03; 
        uint32_t sendTimes = 1; 
        uint32_t timeInterval = 10; 
        uint8_t IDType = 0; 
        uint32_t canId = 0x01; 
        uint8_t frameType = 0; 
        uint8_t len = 0x08; 
        uint8_t idAcc = 0;
        uint8_t dataAcc = 0;
        uint8_t data[8] = {0};
        uint8_t crc = 0; 
    } can_send_frame;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct
    {
        uint8_t FrameHeader;    
        uint8_t CMD;   
        //     0x00: 心跳
        //     0x01: receive fail 0x11: receive success
        //     0x02: send fail 0x12: send success
        //     0x03: set baudrate fail 0x13: set baudrate success
        //     0xEE: communication error 此时格式段为错误码
        //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载         
        uint8_t canDataLen: 6;  
        uint8_t canIde: 1;      
        uint8_t canRtr: 1;      
        uint32_t canId;         
        uint8_t canData[8];     
        uint8_t frameEnd;       
    } CAN_Receive_Frame;
#pragma pack(pop)    
};