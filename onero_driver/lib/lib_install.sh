#!/bin/bash
# Version: 1.0
# Date: $(date +%Y-%m-%d)
# Author: OneRobotics
# Description: Install libonero_api.so for target system

set -e

# Switch to the directory where this script lives, so all subsequent
# relative paths (./linux_*_v*/libonero_api.so, ./libonero_api.so) resolve
# against the script's location instead of the caller's cwd.
cd "$(dirname "$(readlink -f "$0")")"

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
sudo rm -f /usr/local/lib/libonero_api.so* 2>/dev/null || true

# Detect target architecture (override with ONERO_TARGET_ARCH=aarch64 or =x86_64)
RAW_ARCH="${ONERO_TARGET_ARCH:-$(uname -m)}"
case "$RAW_ARCH" in
    x86_64|amd64|AMD64)
        PLATFORM_TAG="linux_x86_64_v1.0.0"
        EXPECTED_MACHINE="X86-64"
        ;;
    aarch64|arm64)
        PLATFORM_TAG="linux_aarch64_v1.0.0"
        EXPECTED_MACHINE="AArch64"
        ;;
    *)
        echo -e "${RED}✗ 错误: 当前架构 $RAW_ARCH 暂不支持${NC}"
        echo -e "${YELLOW}  支持: x86_64, aarch64${NC}"
        exit 1
        ;;
esac

check_so_arch() {
    local so_file="$1"
    local machine
    machine="$(LC_ALL=C readelf -h "$so_file" 2>/dev/null | awk -F: '/Machine:/ {gsub(/^[ \t]+/, "", $2); print $2; exit}')"
    if [[ "$machine" != *"$EXPECTED_MACHINE"* ]]; then
        echo -e "${RED}✗ 错误: libonero_api.so 架构不匹配${NC}"
        echo -e "${YELLOW}  当前环境: $RAW_ARCH, 期望: $EXPECTED_MACHINE, 实际: ${machine:-unknown}${NC}"
        return 1
    fi
}

SO_SOURCE="./${PLATFORM_TAG}/libonero_api.so"
if [ ! -f "$SO_SOURCE" ]; then
    if [ -f "./libonero_api.so" ]; then
        SO_SOURCE="./libonero_api.so"
    else
        mapfile -t SO_CANDIDATES < <(find . -maxdepth 2 -type f -path "./linux_*_v*/libonero_api.so" | sort)
        if [ "${#SO_CANDIDATES[@]}" -eq 1 ]; then
            SO_SOURCE="${SO_CANDIDATES[0]}"
        else
            echo -e "${RED}✗ 错误: 在 ./${PLATFORM_TAG}/ 下找不到 libonero_api.so${NC}"
            echo -e "${YELLOW}  请确认当前发布包已包含此架构的预编译运行库${NC}"
            exit 1
        fi
    fi
fi

if ! check_so_arch "$SO_SOURCE"; then
    exit 1
fi

sudo cp "$SO_SOURCE" /usr/local/lib/

# 检查源文件和目标文件是否为同一文件
if ! cmp -s "$SO_SOURCE" ./libonero_api.so 2>/dev/null; then
    sudo cp "$SO_SOURCE" ./libonero_api.so
fi

# Configure library path
if ! grep -q "/usr/local/lib" /etc/ld.so.conf; then
    echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf > /dev/null
fi

# Update library cache
sudo /sbin/ldconfig

# Completion message

echo -e "${GREEN}✓ libonero_api.so 安装成功!${NC}"
