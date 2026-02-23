#!/bin/bash
set -e

echo "========================================"
echo "    OpenClaw 環境自動安裝與測試腳本     "
echo "========================================"

# 1. 檢查並安裝 Docker (若未安裝)
echo -e "\n[階段一] 檢查 Docker 與 Docker Compose..."
if ! command -v docker &> /dev/null; then
    echo "未檢測到 Docker，開始安裝 Docker..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    echo "Docker 安裝完成。"
else
    echo "Docker 已安裝。"
fi

# 1.5 檢查並配置 NVIDIA Container Toolkit
echo -e "\n[階段一.五] 檢查 NVIDIA Container Toolkit..."
if ! dpkg -l | grep -q nvidia-container-toolkit; then
    echo "檢測到未安裝 nvidia-container-toolkit，開始安裝..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
      sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    echo "NVIDIA Toolkit 安裝與設定完成。"
else
    echo "已安裝 NVIDIA Container Toolkit，跳過安裝。"
fi

# 2. 建立 Python 虛擬環境 (Host端)
echo -e "\n[階段二] 準備 Host 端 Python 虛擬環境..."
cd /home/idaka/openclaw
sudo apt-get install -y python3-venv python3-pip
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "虛擬環境 venv 建立完成。"
else
    echo "虛擬環境 venv 已存在。"
fi
source venv/bin/activate

# 3. Docker 映像檔拉取與啟動 (背景模式以利測試)
echo -e "\n[階段三] 下載並啟動 OpenClaw 容器..."
echo "正在拉取最新的 ARM64 Docker 映像檔..."
# 確保 docker compose 命令存在
if command -v docker-compose &> /dev/null; then
    sudo docker-compose pull
    echo "正在背景啟動容器..."
    sudo docker-compose up -d
else
    sudo docker compose pull
    echo "正在背景啟動容器..."
    sudo docker compose up -d
fi

echo "等待 10 秒讓容器與基礎服務初始化..."
sleep 10

# 4. 基礎狀態檢查與測試
echo -e "\n[階段四] 系統狀態與測試報告..."
echo "--- 容器運行狀態 ---"
sudo docker ps | grep openclaw || echo "容器似乎未正常運行，請檢查日誌"

echo "--- 記憶體使用狀況 ---"
free -h

echo -e "\n============================================="
echo "🎉 所有自動化設定與測試啟動腳本執行完畢！"
echo "👉 若要查看 OpenClaw 詳細日誌，請輸入: sudo docker compose logs -f"
echo "👉 若要停止系統，請輸入: sudo docker compose down"
echo "============================================="
