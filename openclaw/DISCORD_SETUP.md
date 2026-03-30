# OpenClaw Discord Bot 設定指南

> 機器: Jetson Orin Nano Super (192.168.0.15)
> Bot: @jetson_openclaw
> 設定日期: 2026-03-21

---

## 設定步驟

### 1. Discord Developer Portal

1. 登入 https://discord.com/developers/applications
2. New Application → 取名
3. Bot 頁面 → Reset Token → 複製 Token
4. Bot 頁面 → Privileged Gateway Intents → 全部開啟:
   - ✅ Presence Intent
   - ✅ Server Members Intent
   - ✅ Message Content Intent (必須)
5. OAuth2 → URL Generator:
   - Scopes: bot + applications.commands
   - Bot Permissions: 需包含 VIEW_CHANNEL + SEND_MESSAGES + READ_MESSAGE_HISTORY
6. 複製邀請 URL → 在瀏覽器開 → 選 Server → 授權

### 2. Jetson 上設定



### 3. 首次使用 — Pairing

Bot 首次收到訊息會要求 pairing:


在 Jetson 上批准:


---

## 踩坑記錄

### 問題 1: 

**現象:**  報錯
**原因:** OpenClaw 的 Discord 設定不是用 ，而是用 
**解法:**


### 問題 2: Bot 登入成功但 

**現象:** log 顯示  但 status 顯示 
**原因:** Gateway 不斷被 SIGTERM 重啟
**解法:** 用  安裝 systemd service，確保穩定運行


### 問題 3: Bot 收不到 @mention 訊息

**現象:** 在頻道 @bot 沒反應
**原因:** 邀請 Bot 時缺少 VIEW_CHANNEL 權限
**解法:** 重新邀請 Bot，權限整數加上 1024 (VIEW_CHANNEL)

### 問題 4: 頻道 @沒反應，只有 DM 有

**現象:** 私訊 bot 有回覆，但群組頻道 @bot 沒反應
**原因:** 預設 groupPolicy 是 allowlist (需要逐一授權頻道)
**解法:**


### 問題 5: 

**現象:** Bot 登入成功但處理訊息時 crash
**原因:** OpenClaw config 裡 workspace 路徑是 ，但  不存在
**解法:**

這讓  指向真正的 workspace，保留所有記憶和 skills。

### 問題 6: Bot 記憶被洗掉（重新初始化）

**現象:** Bot 回覆 I just came online — fresh out of the box, no memories
**原因:** 建了  目錄後，OpenClaw 在裡面自動初始化了新的空 workspace，覆蓋了 symlink
**解法:**


---

## 目前設定



## 檢查指令



## 重要路徑


