# OpenClaw Discord Bot 設定指南

> 機器: Jetson Orin Nano Super (192.168.0.15)
> Bot: @jetson_openclaw
> 設定日期: 2026-03-21

---

## 設定步驟

### 1. Discord Developer Portal

1. 登入 https://discord.com/developers/applications
2. New Application -> 取名
3. Bot 頁面 -> Reset Token -> 複製 Token
4. Bot 頁面 -> Privileged Gateway Intents -> 全部開啟:
   - Presence Intent
   - Server Members Intent
   - Message Content Intent (必須)
5. OAuth2 -> URL Generator:
   - Scopes: bot + applications.commands
   - Bot Permissions: 需包含 VIEW_CHANNEL + SEND_MESSAGES + READ_MESSAGE_HISTORY
6. 複製邀請 URL -> 選 Server -> 授權

### 2. Jetson 上設定

```bash
# 設定 Discord token
openclaw config set channels.discord.token '"YOUR_BOT_TOKEN"' --json

# 啟用 Discord channel
openclaw config set channels.discord.enabled true --json

# 開放群組頻道權限
openclaw config set channels.discord.groupPolicy '"open"' --json

# 重啟 gateway
openclaw gateway restart
```

### 3. 首次使用 - Pairing

Bot 首次收到訊息會要求 pairing:

在 Jetson 上批准:
```bash
openclaw pairing approve discord PAIRING_CODE
```

---

## 踩坑記錄

### 問題 1: Unknown channel: discord

現象: openclaw channels add --channel discord 報錯
原因: Discord 設定不是用 channels add 而是用 config set
解法: openclaw config set channels.discord.token/enabled/groupPolicy

### 問題 2: Bot 登入但 disconnected

現象: log 顯示 logged in 但 status 顯示 disconnected
解法: openclaw gateway install 安裝 systemd service

### 問題 3: Bot 收不到 @mention

原因: 邀請時缺少 VIEW_CHANNEL 權限
解法: 權限整數加上 1024 重新邀請

### 問題 4: 頻道沒反應只有 DM 有

原因: 預設 groupPolicy 是 allowlist
解法: openclaw config set channels.discord.groupPolicy '"open"' --json

### 問題 5: EACCES permission denied mkdir /home/node

原因: workspace 路徑是 /home/node/.openclaw/workspace 但不存在
解法:
```bash
sudo mkdir -p /home/node
sudo chown idaka:idaka /home/node
ln -sf /home/idaka/.openclaw /home/node/.openclaw
```

### 問題 6: Bot 記憶被洗掉

原因: 建了 /home/node/.openclaw/ 後 OpenClaw 自動初始化空 workspace
解法:
```bash
rm -rf /home/node/.openclaw
ln -sf /home/idaka/.openclaw /home/node/.openclaw
openclaw gateway restart
```

---

## 檢查指令

```bash
openclaw channels status --probe    # channel 狀態
openclaw channels logs              # 查 log
openclaw gateway restart            # 重啟
ps aux | grep openclaw-gateway      # 檢查進程
```

## 重要路徑

```
/home/idaka/.openclaw/              <- 真正的 OpenClaw 資料
/home/node/.openclaw                <- symlink -> /home/idaka/.openclaw
/home/idaka/.openclaw/workspace/    <- skills, 記憶, 工具
/home/idaka/.openclaw/agents/       <- agent 設定
/home/idaka/.openclaw/openclaw.json <- 主設定檔
```
