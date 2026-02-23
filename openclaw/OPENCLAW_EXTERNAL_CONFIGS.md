# OpenClaw 外部設定與依賴紀錄
本文件記錄了為了讓 OpenClaw 在 Jetson 平台上順利運作、突破容器限制以及提升安全性，而在 OpenClaw 容器外部（Jetson 宿主機系統層級）所做的所有設定與建立的檔案。

## 1. 系統資源最佳化

### Swap 記憶體擴充
由於編譯與運行大型容器需要較多記憶體，我們在 NVMe SSD 上建立了 16GB 的 Swapfile。
* **路徑**: `/swapfile`
* **容量**: 16GB (總計 19.7GB)
* **設定檔調整**: 
  * `/etc/fstab` 寫入了開機自動掛載設定。
  * `/etc/sysctl.conf` 將 `vm.swappiness` 調整為 `30`，優化記憶體交換頻率。

---

## 2. 網路與安全性 (HTTPS & 反向代理)

### Nginx 反向代理設定
為了解決 Control UI 只能在 localhost 使用的問題，且必須在安全連線 (Secure Context) 下才能正常處理跨域請求與 WebSocket，我們安裝了 Nginx 並配置為反向代理。
* **套件**: `nginx`, `openssl`
* **設定檔位置**: `/etc/nginx/sites-available/openclaw-ssl` 
  * 已建立軟連結至 `/etc/nginx/sites-enabled/`

**主要配置特點**:
1. 提供 HTTPS 加密連線：代理 `443` port 到 OpenClaw Gateway 的 `18789` port。
2. 支援 WebSocket Upgrade：確保 Web UI 上的終端機即時通訊不斷線。
3. **靜態資源捷徑路由**：為了讓 Web UI 能夠正常載入圖片（不受 Gateway Auth 阻擋），特別將 `/__openclaw__/canvas/` 路徑直接對應到本機資料夾，跳過代理驗證。

### 自簽 SSL 憑證
* **金鑰檔**: `/etc/nginx/ssl/openclaw.key`
* **憑證檔**: `/etc/nginx/ssl/openclaw.crt`

---

## 3. SSH 免密碼登入 (突破容器權限)

為了讓運行在 Docker 容器內的 OpenClaw (Node.js 環境) 能夠執行需要系統底層權限的 bash 腳本（例如開關 CAN 介面、執行 ROS 節點），我們建立了容器連回宿主機的專屬 SSH 通道。

### 設定與金鑰檔
* **建立的資料夾**: `/home/idaka/.openclaw/ssh_config/`
* **金鑰對**:
  * 私鑰: `id_rsa`
  * 公鑰: `id_rsa.pub`
* **SSH Config**: `config` (指定目標 IP 為 192.168.0.15，帳號 `idaka`，並停用 HostKeyChecking)。

### 宿主機授權
為允許 OpenClaw 登入，已將上述公鑰內容寫入宿主機的授權清單中：
* **檔案路徑**: `/home/idaka/.ssh/authorized_keys`

**容器掛載方式**: 在 `docker-compose.yml` 中，將 `/home/idaka/.openclaw/ssh_config` 掛載到容器的 `/home/node/.ssh:ro`。

---

## 4. 目錄與檔案共享設定

為了讓 OpenClaw 能夠看見並修改專案程式碼，我們在 `docker-compose.yml` 中做出了以下掛載設定（Volumes）：

| 宿主機 (Host) 實際路徑 | 容器內對應路徑 | 權限 | 目的 |
|---|---|---|---|
| `~/openarm_can` | `/home/node/openarm_can` | 可讀寫 (`rw`) | 修改與分析 CAN 控制相關 Python 腳本 |
| `~/ros2_ws` | `/home/node/ros2_ws` | 可讀寫 (`rw`) | 修改 ROS2 節點、Shell 腳本 |
| `~/openclaw` | `/home/node/openclaw` | 可讀寫 (`rw`) | 管理 OpenClaw 設定檔 |
| `~/Docs` | `/home/node/Docs` | 唯讀 (`ro`) | 讀取專案相關文件 |
| `~/CAN...资料...` | `/home/node/CANFD_Docs` | 唯讀 (`ro`) | 讀取硬體驅動參考文件（避免中文括弧導致路徑解析錯誤而重新命名掛載點） |

---

## 5. 資源分享目錄 (Canvas)

為了解決 Web UI 無法顯示本機產生的截圖與分析圖片，我們建立了一個專門的靜態資源目錄。

* **本機路徑**: `/home/idaka/.openclaw/canvas`
* **運作機制**: 
  1. OpenClaw 透過 SSH 本機指令將相機截圖存入此目錄。
  2. Nginx 直接將網址 `/__openclaw__/canvas/圖片名.jpg` 導向本機的這個資料夾。
  3. 控制面板即可正確顯示 Markdown 格式的圖片 `![名稱](__openclaw__/canvas/圖片名.jpg)`。

---

## 6. OpenClaw 底層設定檔修改

這些設定檔雖然屬於 OpenClaw，但已被我們手動最佳化，未來如需變更 AI 模型或環境變數，皆可從這裡查找：

1. **環境變數設定檔**: `/home/idaka/openclaw_src/.env`
   - 包含寫入的 `OPENAI_API_KEY` 與 `ANTHROPIC_API_KEY`。
2. **核心系統設定**: `/home/idaka/.openclaw/openclaw.json`
   - 已配置為繞過 Claude API 餘額不足的限制：
     - **主要模型**: `openai/gpt-5.2-codex`
     - **備用模型**: `anthropic/claude-opus-4-6`
