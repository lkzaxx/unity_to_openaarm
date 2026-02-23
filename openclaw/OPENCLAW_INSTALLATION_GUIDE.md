# OpenClaw 於 Jetson 開發環境安裝與設定完整指南

本指南詳細記錄了如何在 NVIDIA Jetson 平台上，從零開始建置、配置並安裝 OpenClaw 系統的標準作業流程 (SOP)。

> **官方文件**：https://docs.openclaw.ai/install/docker
> **官方 GitHub**：https://github.com/openclaw/openclaw

---

## 階段一：系統底層與基礎環境優化

### 1. 擴充 Swap 記憶體空間 (必做)
由於 Jetson 設備在運行無頭瀏覽器與 Node.js 時耗用大量記憶體，必須透過 NVMe SSD 擴充至少 16GB 的 Swap 空間。
```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
sudo sysctl vm.swappiness=30
echo 'vm.swappiness=30' | sudo tee -a /etc/sysctl.conf
```

### 2. 安裝 Docker Engine
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```

### 3. 安裝 NVIDIA Container Toolkit (GPU 穿透)
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 階段二：下載 OpenClaw 與建立環境

### 1. 安裝 Node.js ≥ 22 (OpenClaw 官方要求)
```bash
curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt-get install -y nodejs
node -v  # 確認版本 >= 22
```

### 2. 從官方 GitHub 拉取 OpenClaw 原始碼
```bash
cd ~
git clone https://github.com/openclaw/openclaw.git openclaw_src
cd ~/openclaw_src
```

### 3. 撰寫環境變數檔 (`.env`)
> 在 `~/openclaw_src` 目錄下建立 `.env` 檔案
- **GEMINI_API_KEY**：填寫 Gemini 3.1 Pro 的 API 金鑰。
- **BROWSER_CONCURRENCY**：強制設定為 `1`，限制無頭瀏覽器併發，保護記憶體。
- **ROS_DOMAIN_ID**：填寫您的 ROS2 網域 ID（例如：`ROS_DOMAIN_ID=0`）。

---

## 階段三：使用官方 Docker Setup 腳本建置並啟動

### 1. 執行官方 Docker Setup 腳本 (推薦方式)
官方提供的 `docker-setup.sh` 會自動執行以下動作：
- 建置 Gateway Docker 映像檔 (本地 build，非從 Hub Pull)
- 執行 Onboarding Wizard（引導設定 AI Provider、Workspace 等）
- 產生 Gateway Token 並寫入 `.env`
- 透過 Docker Compose 啟動 Gateway
```bash
cd ~/openclaw_src
./docker-setup.sh
```

### 2. 手動方式 (若 docker-setup.sh 遇到問題)
```bash
cd ~/openclaw_src
docker build -t openclaw:local -f Dockerfile .
docker compose run --rm openclaw-cli onboard
docker compose up -d openclaw-gateway
```

### 3. 存取 Control UI
- 在瀏覽器開啟 `http://<JETSON_IP>:18789/`
- 在 Settings 中貼上 Gateway Token

---

## 階段四：通訊軟體頻道設定

### 1. 設定 Telegram Bot
```bash
cd ~/openclaw_src
docker compose run --rm openclaw-cli channels add --channel telegram --token "<YOUR_TELEGRAM_BOT_TOKEN>"
```

### 2. 設定 Discord Bot
```bash
cd ~/openclaw_src
docker compose run --rm openclaw-cli channels add --channel discord --token "<YOUR_DISCORD_BOT_TOKEN>"
```

### 3. Health Check 驗證
```bash
docker compose exec openclaw-gateway node dist/index.js health --token "$OPENCLAW_GATEWAY_TOKEN"
```

---

## 階段五：進階整合 - ROS2 與 VR

### 1. ROS2 跨容器通訊
- 若需在 Docker 容器內啟用 ROS2 通訊，需額外掛載 ROS2 設定
- 使用 `OPENCLAW_EXTRA_MOUNTS` 環境變數掛載所需目錄

### 2. 雙目視覺與 Unity VR
- 將 IMX219-83 雙目相機的影像流經由 GStreamer 或 ROS2 影像節點讀取
- 透過 `ros_tcp_endpoint` 橋接送至 Unity 端

---

> [!NOTE]
> 請依序執行上述階段。任何異常狀態應優先查閱 Docker 日誌 (`docker compose logs -f`) 與系統日誌 (`dmesg`) 進行排障。
> 官方 Troubleshooting：https://docs.openclaw.ai/install/docker#troubleshooting
