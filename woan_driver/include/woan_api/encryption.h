#ifndef ENCRYPTION_H
#define ENCRYPTION_H

#include "woan_api/woan_platform.h"

#include <vector>
#include <string>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <cstring>
#include <cerrno>

#if defined(_WIN32)
#include <io.h>
#include <process.h>  // for _getpid()
#else
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>   // for getpid(), write(), close(), unlink()
#endif

namespace woan_api {

/**
 * @class TrajectoryEncryption
 * @brief 轨迹数据加密工具类，使用AES-256-CBC加密
 */
class TrajectoryEncryption {
public:
    TrajectoryEncryption();
    ~TrajectoryEncryption();

    /**
     * @brief 加密数据
     * @param plaintext 明文数据
     * @param ciphertext 输出密文数据
     * @return true if successful, false otherwise
     */
    bool encrypt(const std::vector<uint8_t>& plaintext, std::vector<uint8_t>& ciphertext);

    /**
     * @brief 解密数据
     * @param ciphertext 密文数据
     * @param plaintext 输出明文数据
     * @return true if successful, false otherwise
     */
    bool decrypt(const std::vector<uint8_t>& ciphertext, std::vector<uint8_t>& plaintext);

private:
    // AES-256密钥（32字节）
    static constexpr uint8_t key_[32] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
        0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
    };

    // 初始化向量（16字节）
    static constexpr uint8_t iv_[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
    };
};

/**
 * @brief 解密URDF文件（如果是加密的）
 * @param urdf_path URDF文件路径（可能是加密的 .encrypted 文件）
 * @param throw_on_error 如果为true，解密失败时抛出异常；如果为false，失败时返回原路径
 * @return 解密后的URDF文件路径（如果是加密文件，返回临时文件路径；否则返回原路径）
 * @throws std::runtime_error 当 throw_on_error=true 且解密失败时
 */
inline std::string decryptUrdfFile(const std::string& urdf_path, bool throw_on_error = false) {
    // 检查是否为加密文件（.encrypted 后缀）
    if (urdf_path.length() < 10 || urdf_path.substr(urdf_path.length() - 10) != ".encrypted") {
        // 不是加密文件，直接返回原路径
        return urdf_path;
    }
    
    // 读取加密文件
    std::ifstream encrypted_file(urdf_path, std::ios::binary);
    if (!encrypted_file.is_open()) {
        std::string error_msg = "Failed to open encrypted URDF file: " + urdf_path;
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;  // 返回原路径，让调用者处理错误
    }
    
    // 读取密文长度（4字节）
    uint32_t ciphertext_len = 0;
    if (!encrypted_file.read(reinterpret_cast<char*>(&ciphertext_len), sizeof(uint32_t))) {
        encrypted_file.close();
        std::string error_msg = "Failed to read ciphertext length from: " + urdf_path;
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }
    
    if (ciphertext_len == 0 || ciphertext_len > 10 * 1024 * 1024) {  // 限制最大10MB
        encrypted_file.close();
        std::string error_msg = "Invalid ciphertext length: " + std::to_string(ciphertext_len);
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }
    
    // 读取密文数据
    std::vector<uint8_t> ciphertext(ciphertext_len);
    if (!encrypted_file.read(reinterpret_cast<char*>(ciphertext.data()), ciphertext_len) ||
        encrypted_file.gcount() != static_cast<std::streamsize>(ciphertext_len)) {
        encrypted_file.close();
        std::string error_msg = "Failed to read ciphertext data from: " + urdf_path;
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }
    encrypted_file.close();
    
    // 解密数据
    TrajectoryEncryption encryption;
    std::vector<uint8_t> plaintext;
    if (!encryption.decrypt(ciphertext, plaintext)) {
        std::string error_msg = "Failed to decrypt URDF file: " + urdf_path;
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }
    
    // 使用内存文件系统（/dev/shm）创建临时文件，提高安全性
    // 优先使用 /dev/shm（内存文件系统，仅 Linux），如果不存在则使用系统临时目录
    std::filesystem::path temp_base_dir;
#if defined(_WIN32)
    // Windows: 使用系统临时目录
    temp_base_dir = std::filesystem::temp_directory_path();
#else
    // Linux/POSIX: 优先使用 /dev/shm（内存文件系统）
    if (std::filesystem::exists("/dev/shm")) {
        temp_base_dir = "/dev/shm";
    } else {
        temp_base_dir = std::filesystem::temp_directory_path();
    }
#endif

#if defined(_WIN32)
    // Windows: 使用 C++ 标准库创建临时文件
    std::string temp_filename = "decrypted_urdf_" + std::to_string(_getpid()) + "_" +
                                std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".urdf";
    std::filesystem::path temp_path = temp_base_dir / temp_filename;

    // 写入解密后的内容
    std::ofstream temp_file(temp_path, std::ios::binary);
    if (!temp_file.is_open()) {
        std::string error_msg = "Failed to create temporary URDF file: " + temp_path.string();
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }

    temp_file.write(reinterpret_cast<const char*>(plaintext.data()), plaintext.size());
    temp_file.close();

    if (!temp_file.good()) {
        std::filesystem::remove(temp_path); // 删除文件
        std::string error_msg = "Failed to write decrypted URDF content";
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }

    // Windows: 设置文件权限（仅当前用户可访问）
    // 注意：Windows 文件权限模型与 POSIX 不同，这里使用基本的属性设置
    std::filesystem::permissions(temp_path,
                                 std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
                                 std::filesystem::perm_options::replace);

    std::string temp_path_str = temp_path.string();
#else
    // Linux/POSIX: 使用 mkstemp 创建安全的临时文件（自动设置权限为 0600）
    std::string temp_template = (temp_base_dir / ("decrypted_urdf_" + std::to_string(getpid()) + "_XXXXXX.urdf")).string();
    std::vector<char> temp_template_vec(temp_template.begin(), temp_template.end());
    temp_template_vec.push_back('\0');

    int fd = mkstemps(temp_template_vec.data(), 5); // 5 = ".urdf" 的长度
    if (fd < 0) {
        std::string error_msg = "Failed to create secure temporary URDF file: " + std::string(strerror(errno));
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }

    // 写入解密后的内容
    ssize_t written = write(fd, plaintext.data(), plaintext.size());
    if (written != static_cast<ssize_t>(plaintext.size())) {
        close(fd);
        unlink(temp_template_vec.data()); // 删除文件
        std::string error_msg = "Failed to write decrypted URDF content";
        if (throw_on_error) {
            throw std::runtime_error(error_msg);
        }
        std::cerr << "[X] " << error_msg << std::endl;
        return urdf_path;
    }

    // 确保文件权限为 0600（只有当前用户可读写）
    fchmod(fd, 0600);

    close(fd);

    std::string temp_path_str(temp_template_vec.data());
#endif

    // 不打印文件路径，减少信息泄露
    // std::cout << "✓ Decrypted URDF saved to secure temporary file" << std::endl;
    return temp_path_str;
}

/**
 * @brief 直接解密URDF内容到内存字符串
 * @param urdf_path URDF文件路径（可能是加密的 .encrypted 文件）
 * @return 解密后的字符串内容，如果失败则返回空字符串
 */
inline std::string decryptUrdfToMemory(const std::string& urdf_path) {
    // 检查是否为加密文件（.encrypted 后缀）
    if (urdf_path.length() < 10 || urdf_path.substr(urdf_path.length() - 10) != ".encrypted") {
        return ""; // 不是加密文件
    }
    
    // 读取加密文件
    std::ifstream encrypted_file(urdf_path, std::ios::binary);
    if (!encrypted_file.is_open()) return "";
    
    // 读取密文长度（4字节）
    uint32_t ciphertext_len = 0;
    if (!encrypted_file.read(reinterpret_cast<char*>(&ciphertext_len), sizeof(uint32_t))) return "";
    
    if (ciphertext_len == 0 || ciphertext_len > 10 * 1024 * 1024) return "";
    
    // 读取密文数据
    std::vector<uint8_t> ciphertext(ciphertext_len);
    if (!encrypted_file.read(reinterpret_cast<char*>(ciphertext.data()), ciphertext_len)) return "";
    encrypted_file.close();
    
    // 解密数据
    TrajectoryEncryption encryption;
    std::vector<uint8_t> plaintext;
    if (!encryption.decrypt(ciphertext, plaintext)) return "";
    
    return std::string(plaintext.begin(), plaintext.end());
}

}  // namespace woan_api

#endif  // ENCRYPTION_H
