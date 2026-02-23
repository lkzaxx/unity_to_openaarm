# OpenClaw 實作進度檢核表 (Checkpoint)

> **📝 參考基準文獻**：請隨時對照 [`OPENCLAW_INSTALLATION_GUIDE.md`](./OPENCLAW_INSTALLATION_GUIDE.md)。
> **📚 官方 Docker 文件**：https://docs.openclaw.ai/install/docker
> **� 官方 GitHub**：https://github.com/openclaw/openclaw
> **�💡 使用方式**：完成一項後將 `[ ]` 改為 `[x]`。

---

## 階段一：系統底層與基礎環境優化

- [x] **擴充 Swap 記憶體空間 (必做)**
  - [x] 劃分並掛載至少 16GB NVMe SSD 作為 Swap。
  - [x] 調整 `swappiness`。
  - [x] 設定 `/etc/fstab` 開機自動掛載。
- [x] **安裝 Docker Engine**
  - [x] 使用 `get.docker.com` 腳本安裝 Docker v29.2.1。
  - [x] 將使用者加入 docker 群組。
- [x] **安裝與配置 NVIDIA Container Toolkit**
  - [x] 安裝 nvidia-container-toolkit v1.16.2。
  - [x] 配置 Docker daemon 預設 runtime 為 nvidia。
  - [x] 重新啟動 Docker 服務。

---

## 階段二：下載 OpenClaw 與建立環境

- [x] **安裝 Node.js ≥ 22 (官方要求)**
  - [x] 使用 NodeSource 安裝 Node.js v22.22.0。
  - [x] 驗證 `node -v` = v22.22.0，`npm -v` = 10.9.4。
- [x] **從官方 GitHub 拉取 OpenClaw 原始碼**
  - [x] 執行 `git clone https://github.com/openclaw/openclaw.git ~/openclaw_src`。
  - [x] 倉庫大小：211.74 MiB，6500 個檔案。
- [x] **撰寫環境變數檔 (`.env`)**
  - [x] 由 `docker-setup.sh` 自動產生於 `~/openclaw_src/.env`。

---

## 階段三：使用官方 Docker Setup 建置並啟動

- [x] **Docker 映像檔本地建置**
  - [x] `docker build -t openclaw:local` 成功 (FROM node:22-bookworm)。
  - [x] 包含 pnpm install、pnpm build、pnpm ui:build。
- [x] **執行 `./docker-setup.sh`**
  - [x] 自動建置映像檔。
  - [x] 產生 Gateway Token。
  - [x] Docker Compose 啟動 Gateway。
- [x] **容器運行狀態確認**
  - [x] 容器名稱：`openclaw_src-openclaw-gateway-1`
  - [x] 映像檔：`openclaw:local`
  - [x] 端口映射：`0.0.0.0:18789-18790->18789-18790/tcp`
  - [x] 狀態：**Running**

### ⚠️ 待處理事項
- [x] ~~容器日誌顯示 `Missing config`~~ → 已加 `--allow-unconfigured` + `gateway.mode=local` 解決。
- [x] ~~Swap 空間只有 3.7GB~~ → 已建立 16GB swapfile，總計 19GB。
- [x] 存取 Control UI (`http://192.168.0.15:18789/`) → HTTP 200 ✅
  - Gateway Token: `cc293ac022a00890020a21d99ddbbbbf3331ae4abfe8ea8da92c0675d99aaeca`
  - ℹ️ 瀏覽器需用 `localhost` 或 SSH 隧道才能使用 Control UI
- [x] 配置 AI Provider
  - ✅ OpenAI GPT-5.2-Codex (primary) — OPENAI_API_KEY 已設定 (避開 Claude 餘額不足)
  - ✅ Claude Opus 4.6 (fallback) — ANTHROPIC_API_KEY 已設定

---

## 階段四：通訊軟體頻道設定與測試

- [ ] **設定 Telegram Bot**
  - [ ] `docker compose run --rm openclaw-cli channels add --channel telegram --token "<TOKEN>"`
- [ ] **設定 Discord Bot**
  - [ ] `docker compose run --rm openclaw-cli channels add --channel discord --token "<TOKEN>"`
- [ ] **基礎測試**
  - [ ] 發送測試訊息確認 Bot 回應正常。

---

## 階段五：進階整合 - ROS2 與 VR

- [ ] **ROS2 跨容器通訊**
  - [ ] 掛載 ROS2 設定至 Docker 容器。
  - [ ] 測試 Topic 發佈/訂閱。
- [ ] **雙目視覺與 Unity VR 橋接**
  - [ ] IMX219-83 影像讀取。
  - [ ] ros_tcp_endpoint 部署。
  - [ ] Unity 端雙向通訊測試。

---

## 📋 重要資訊速查
| 項目 | 值 |
|---|---|
| OpenClaw 原始碼路徑 | `~/openclaw_src` |
| Docker Compose 檔案 | `~/openclaw_src/docker-compose.yml` |
| 環境變數檔 | `~/openclaw_src/.env` |
| Gateway 端口 | `18789` |
| Gateway Token | `cc293ac022a00890020a21d99ddbbbbf3331ae4abfe8ea8da92c0675d99aaeca` |
| Node.js 版本 | v22.22.0 |
| Docker 版本 | v29.2.1 |
| NVIDIA Toolkit | v1.16.2 |

*備註：遇到報錯優先查閱 `docker compose -f ~/openclaw_src/docker-compose.yml logs -f openclaw-gateway`。*
