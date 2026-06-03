#!/bin/bash
# Version: 1.0
# Date: $(date +%Y-%m-%d)
# Author: Your Name
# Description: Install libwoan_api.so for target system

set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check sudo privileges
if [ "$(id -u)" != "0" ]; then
    echo -e "${RED}✗ 错误: 此脚本必须以 sudo 权限运行${NC}"
    echo -e "${YELLOW}用法示例: sudo $0${NC}"
    exit 1
fi

# Cleanup old library
sudo rm -f /usr/local/lib/libwoan_api.so* 2>/dev/null || true

if [ $(uname -m) = "x86_64" ]; then
    if [ -f "./linux_x86_64_v1.0.0/libwoan_api.so" ];then
        cd linux_x86_64_v1.0.0
        sudo cp ./libwoan_api.so /usr/local/lib/
        
        # 检查源文件和目标文件是否为同一文件
        if ! cmp -s ./libwoan_api.so ../libwoan_api.so 2>/dev/null; then
            sudo cp ./libwoan_api.so ..
        fi
        cd ..
    else
        echo -e "${RED}✗ 错误: 在当前目录中找不到 x86_64 libwoan_api.so!${NC}"
        exit 1
    fi
else
    echo -e "${RED}✗ 错误: 当前架构 $(uname -m) 暂不支持!${NC}"
    echo -e "${YELLOW}提示: 目前仅支持 x86_64 架构${NC}"
    exit 1
fi 

# Configure library path
if ! grep -q "/usr/local/lib" /etc/ld.so.conf; then
    echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf > /dev/null
fi

# Update library cache
sudo /sbin/ldconfig

# Completion message

echo -e "${GREEN}✓ libwoan_api.so 安装成功!${NC}"

