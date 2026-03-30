# OpenClaw Jetson 部署設定

OpenClaw 在 NVIDIA Jetson Orin Nano Super 上的 Docker 部署與遠端存取設定。

## 環境

| 項目 | 值 |
|------|---|
| 主機 | Jetson Orin Nano Super |
| OS | Ubuntu 22.04.5 LTS (JetPack R36.4.4) |
| Docker Image | openclaw:local |
| OpenClaw 版本 | v2026.3.8 |
| 設定檔 | /home/idaka/.openclaw/openclaw.json |

## 架構

```
PC 瀏覽器
    |
    | https://192.168.0.15
    v
nginx (port 443, HTTPS, 對外 LAN)
    |
    |-- /__openclaw__/canvas/*  --> 直接從磁碟讀檔 (不經 gateway，無需 auth)
    |-- /*                      --> proxy_pass http://127.0.0.1:18789
    v
openclaw-gateway (bind: loopback, network_mode: host)
```

- nginx 監聽 443 port，提供 HTTPS 與反向代理
- Gateway 綁定 loopback，僅接受 localhost 連線
- nginx 反代到 localhost:18789，gateway 視為本地請求，不需要 auth token
- Canvas 圖片由 nginx 直接從 `/home/idaka/.openclaw/canvas/` 提供，完全繞過 gateway 認證

## 存取方式

PC 瀏覽器直接開啟：

```
https://192.168.0.15/chat
```

不需要 SSH tunnel，nginx 反向代理已處理所有認證問題。

## 設定檔說明

### .env

```env
OPENCLAW_GATEWAY_BIND=loopback
# 不設定 OPENCLAW_GATEWAY_TOKEN (gateway 會自動產生，但 loopback 模式下不影響)
```

### docker-compose.override.yml

```yaml
services:
  openclaw-gateway:
    network_mode: "host"    # 容器直接使用 Jetson 網路，讓 bind:loopback 生效
    volumes:
      - /home/idaka/openclaw_src:/app
      - /home/idaka/.ssh:/home/node/.ssh
      - /home/idaka/ros2_ws:/home/idaka/ros2_ws
      - /home/idaka/openarm_can:/home/idaka/openarm_can
      - /home/idaka/realsense:/home/idaka/realsense
      - /home/idaka/CANFD_Docs:/home/idaka/CANFD_Docs:ro
      - /home/idaka/Docs:/home/idaka/Docs:ro

  openclaw-cli:
    network_mode: "host"
    volumes:
      - /home/idaka/openclaw_src:/app
      - /home/idaka/.ssh:/home/node/.ssh
      - /home/idaka/ros2_ws:/home/idaka/ros2_ws
      - /home/idaka/openarm_can:/home/idaka/openarm_can
      - /home/idaka/realsense:/home/idaka/realsense
      - /home/idaka/CANFD_Docs:/home/idaka/CANFD_Docs:ro
      - /home/idaka/Docs:/home/idaka/Docs:ro
```

### openclaw.json (gateway 區段)

```json
{
  "gateway": {
    "mode": "local",
    "controlUi": {
      "dangerouslyAllowHostHeaderOriginFallback": true
    }
  }
}
```

不設定 bind、port、auth，使用預設值 (loopback、無需認證)。
gateway 啟動時會自動產生 auth token 寫入設定檔，這是正常行為，不影響 localhost 存取。

### nginx 設定 (/etc/nginx/sites-enabled/default)

```nginx
server {
    listen 443 ssl;
    server_name 192.168.0.15;

    ssl_certificate     /etc/nginx/ssl/openclaw.crt;
    ssl_certificate_key /etc/nginx/ssl/openclaw.key;

    # Canvas 圖片直接由 nginx 提供，繞過 gateway auth
    location /__openclaw__/canvas/ {
        alias /home/idaka/.openclaw/canvas/;
        autoindex off;
        expires 1h;
    }

    # 其餘請求反代到 gateway
    location / {
        proxy_pass http://127.0.0.1:18789;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection $connection_upgrade;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto https;
        proxy_read_timeout 86400;
        proxy_send_timeout 86400;
    }
}
```

## Webchat 圖片渲染修補

OpenClaw webchat (control-ui) 的 markdown renderer 預設只接受 `data:image/...;base64,...` 格式的圖片，
不渲染 `http://` 或 `https://` URL 圖片。這導致 agent 產生的 canvas 圖片 (以 URL 引用) 在對話窗中不顯示。

### 修改內容

檔案：`dist/control-ui/assets/index-wxM3V0HM.js`
備份：`dist/control-ui/assets/index-wxM3V0HM.js.bak`

圖片 URL 驗證正則 `Vy`：

```
改前：Vy=/^data:image\/[a-z0-9.+-]+;base64,/i
改後：Vy=/^(data:image\/[a-z0-9.+-]+;base64,|https?:\/\/)/i
```

修改後同時接受 base64 內嵌圖片和 https URL 圖片。

### 注意事項

- 此修改直接改 dist 打包檔，openclaw 版本更新後會被覆蓋，需重新套用
- 修改後瀏覽器需清除快取才能生效
- 原始檔已備份為 `.bak`，可隨時還原

## 為什麼這樣設定

OpenClaw v2026.2.19 起強制 LAN 綁定必須有 auth token (自動產生)。
Gateway 改為 bind: loopback + nginx 反代的原因：

- Gateway 綁定 localhost，nginx 反代到 localhost:18789，gateway 視為本地請求不需 auth
- Docker 使用 network_mode: host，讓 gateway 直接綁定 Jetson 的 localhost
- nginx 對外提供 HTTPS (port 443)，支援 WebSocket 升級
- nginx 直接提供 canvas 圖片檔案，不經 gateway auth

## 容器管理

```bash
# 啟動
cd ~/openclaw_src
sudo docker compose up -d

# 停止
sudo docker compose down

# 查看狀態
sudo docker ps

# 查看 gateway log
sudo docker logs openclaw_src-openclaw-gateway-1

# 驗證 gateway 運作
curl http://127.0.0.1:18789/healthz

# nginx 管理
sudo systemctl status nginx
sudo nginx -t            # 測試設定
sudo systemctl reload nginx
```

## 備份與還原

如需還原為 LAN 直連模式 (圖片會有問題)：

```bash
cp ~/.openclaw/openclaw.json.bak.lan-auth ~/.openclaw/openclaw.json
# 修改 .env: OPENCLAW_GATEWAY_BIND=lan
# 修改 override: 移除 network_mode: "host"
sudo docker compose down && sudo docker compose up -d
```
