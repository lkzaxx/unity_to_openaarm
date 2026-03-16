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

---

## 7. 已知衝突與注意事項 (硬體控制端)

### CAN Bus 佇列溢位與手臂震盪問題
因為 OpenClaw 背景服務（如 Node.js, Docker, Nginx 等）增加了作業系統的常駐負載，這會導致 Linux 系統的進程切換稍微變頻繁。
當 OpenArm 執行高頻 (500Hz) 的 MIT 馬達控制時：
* Linux CAN 介面預設的發送佇列長度 (`qlen` / `txqueuelen`) 只有 **10**。
* 在稍微忙碌的系統下，瞬間送出 7 顆馬達的指令會導致佇列瞬間溢滿，**造成 Linux 核心直接丟棄 (Drop) 後續的馬達控制封包**。
* **症狀**：手臂因為連續漏接指令而產生嚴重的抽搐與劇烈震盪，且無法抵達目標位置。

**解決方案**：
若觀察到異常震盪，請務必確認您的 `/home/idaka/ros2_ws/scripts/cansetup.sh` 中已加入加大佇列的設定：
```bash
sudo ip link set can1 txqueuelen 1000
sudo ip link set can2 txqueuelen 1000
```
每次重開機後，都必須執行套用新的 `txqueuelen`，方能確保高頻控制不掉包。


---

## 8. 記憶系統 Embedding 設定

### 問題背景
OpenClaw 的記憶搜索使用 embedding（向量化）來做語義匹配。原本使用 OpenAI 的 embedding API，但因 OpenAI 額度用完（429 insufficient_quota），導致記憶搜索完全失敗。

### 解決方案：改用 local embedding
改用本地 embedding，由 Jetson Orin Nano Super 的 GPU 直接運算，免費且無額度限制。

### 設定方式
在  中加入：


### 模型資訊
| 項目 | 值 |
|---|---|
| 模型名稱 | EmbeddingGemma 300M (Google) |
| 檔案 |  |
| 大小 | 約 300MB |
| 來源 |  (HuggingFace) |
| 引擎 | node-llama-cpp |
| 模型快取路徑 (容器內) |  |
| 首次使用 | 自動下載，之後使用快取 |

### 可用的 Embedding Provider
| Provider | 環境變數 | 說明 |
|---|---|---|
| usage: openai [-h] [-v] [-b API_BASE] [-k API_KEY] [-p PROXY [PROXY ...]]
              [-o ORGANIZATION] [-t {openai,azure}]
              [--api-version API_VERSION] [--azure-endpoint AZURE_ENDPOINT]
              [--azure-ad-token AZURE_AD_TOKEN] [-V]
              {api,tools,migrate,grit} ...

positional arguments:
  {api,tools,migrate,grit}
    api                 Direct API calls
    tools               Client side tools for convenience

options:
  -h, --help            show this help message and exit
  -v, --verbose         Set verbosity.
  -b API_BASE, --api-base API_BASE
                        What API base url to use.
  -k API_KEY, --api-key API_KEY
                        What API key to use.
  -p PROXY [PROXY ...], --proxy PROXY [PROXY ...]
                        What proxy to use.
  -o ORGANIZATION, --organization ORGANIZATION
                        Which organization to run as (will use your default
                        organization if not specified)
  -t {openai,azure}, --api-type {openai,azure}
                        The backend API to call, must be `openai` or `azure`
  --api-version API_VERSION
                        The Azure API version, e.g.
                        'https://learn.microsoft.com/en-us/azure/ai-
                        services/openai/reference#rest-api-versioning'
  --azure-endpoint AZURE_ENDPOINT
                        The Azure endpoint, e.g.
                        'https://endpoint.openai.azure.com'
  --azure-ad-token AZURE_AD_TOKEN
                        A token from Azure Active Directory,
                        https://www.microsoft.com/en-
                        us/security/business/identity-access/microsoft-entra-
                        id
  -V, --version         show program's version number and exit |  | 原本使用，額度用完 |
|  |  | Anthropic 旗下，品質最佳 |
|  |  | Google，有免費額度 |
|  | 無需 | 需安裝 Ollama |
|  | 無需 | 本地 GPU 運算，目前採用 |

### 注意事項
- 模型快取在容器內，容器重建後需重新下載（約 300MB）
- 如需持久化，可將  掛載到宿主機


---

## 8. Memory Embedding Settings

### Background
OpenClaw memory search uses embedding for semantic matching.
Originally used OpenAI embedding API, but quota exhausted (429 insufficient_quota),
causing memory search to fail completely.

### Solution: Local Embedding
Use local embedding on Jetson Orin Nano Super GPU. Free, no quota limits.

### Configuration
In openclaw.json, add under agents.defaults:
  memorySearch.provider = "local"

### Model Info
- Model: EmbeddingGemma 300M by Google
- File: embeddinggemma-300m-qat-Q8_0.gguf, about 300MB
- Source: HuggingFace ggml-org/embeddinggemma-300m-qat-q8_0-GGUF
- Engine: node-llama-cpp
- Cache path in container: /home/node/.node-llama-cpp/models/
- First use: auto-downloads, then cached

### Available Embedding Providers
- openai: needs OPENAI_API_KEY, was used, quota exhausted
- voyage: needs VOYAGE_API_KEY, Anthropic subsidiary, best quality
- gemini: needs GEMINI_API_KEY, Google, has free tier
- ollama: no key needed, requires Ollama installed
- local: no key needed, local GPU compute, currently adopted

### Notes
- Model cache is inside container; rebuilt container requires re-download about 300MB
- To persist, mount /home/node/.node-llama-cpp to host
