# OpenClaw CLI 安裝與 Claude Max 認證配置指南

## 環境資訊
- 伺服器：192.168.0.15 (Jetson)
- 用戶：idaka
- OpenClaw 源碼：`/home/idaka/openclaw_src`
- 配置目錄：`/home/idaka/.openclaw/`

---

## 1. 安裝 CLI

### 1.1 安裝 pnpm（如果沒有）
```bash
sudo npm install -g pnpm
```

### 1.2 安裝依賴並 Build
```bash
cd /home/idaka/openclaw_src
pnpm install
npm run build
```

### 1.3 連結 CLI
```bash
sudo npm link
```

### 1.4 確認安裝成功
```bash
openclaw --version
# 應顯示：2026.2.23
```

---

## 2. 生成 Claude Setup Token

在**任何有安裝 Claude Code CLI 的機器**上執行：
```bash
claude setup-token
```

會產生一個 1 年有效的 OAuth token：
```
Your OAuth token (valid for 1 year):
sk-ant-oat01-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx...
```

> **注意**：此 token 是給 Claude Max/Pro 訂閱用戶使用，無需額外付費 API 費用。

---

## 3. 執行配置向導

```bash
openclaw onboard
```

### 3.1 安全確認
```
◇  I understand this is powerful and inherently risky. Continue?
│  Yes
```

### 3.2 Config handling
```
◆  Config handling
│  ○ Use existing values
│  ● Update values        ← 選這個（更新配置）
│  ○ Reset
```

### 3.3 Gateway 設定
```
◆  What do you want to set up?
│  ● Local gateway (this machine)   ← 選這個
│  ○ Remote gateway (info-only)
```

### 3.4 Gateway port
```
◆  Gateway port
│  18789                            ← 保持預設，直接 Enter
```

### 3.5 Gateway bind
```
◆  Gateway bind
│  ○ Loopback (127.0.0.1)
│  ● LAN (0.0.0.0)                  ← 選這個（允許外部連接）
│  ○ Tailnet (Tailscale IP)
│  ○ Auto (Loopback → LAN)
│  ○ Custom IP
```

### 3.6 Tailscale exposure
```
◆  Tailscale exposure
│  ● Off (No Tailscale exposure)    ← 選這個（區域網路使用）
│  ○ Serve
│  ○ Funnel
```

### 3.7 Gateway token
```
◆  Gateway token (blank to generate)
│  cc293ac022a00890020a21d99ddbbbbf3331ae4abfe8ea8da92c0675d99aaeca
```
> 使用現有的 token，或留空自動生成新的

### 3.8 選擇 AI 後端
```
◆  Model/auth provider
│  Anthropic
```

### 3.9 選擇認證方式
```
◆  Anthropic auth method
│  ● Anthropic token (paste setup-token)   ← Claude Max 用戶選這個
│  ○ Anthropic token (Claude Code CLI)
│  ○ Anthropic API key
```

### 3.10 貼上 Setup Token
```
◇  Paste Anthropic setup-token
│  sk-ant-oat01-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx...
```

### 3.11 選擇預設模型
```
◇  Default model
│  anthropic/claude-opus-4-5
```

---

## 4. 重啟 Gateway

配置完成後，重啟 Docker 容器套用新配置：

```bash
cd /home/idaka/openclaw_src
sudo docker compose restart openclaw-gateway
```

確認啟動狀態：
```bash
sudo docker logs openclaw_src-openclaw-gateway-1 --tail 10
```

應看到：
```
[gateway] agent model: anthropic/claude-opus-4-5
[gateway] listening on ws://0.0.0.0:18789 (PID 7)
```

---

## 5. 測試連接

在瀏覽器訪問：
```
https://192.168.0.15/
```

在聊天中測試：
```
/model opus
```
然後發送訊息確認 Claude 正常回應。

---

## 6. 認證方式對比

| 認證方式 | 適用場景 | 優點 | 缺點 |
|---------|---------|------|------|
| **setup-token** | Claude Max/Pro 訂閱用戶 | 無需額外付費，複用訂閱額度 | 需要 Claude Code CLI 生成 token |
| Claude Code CLI | 已配置 Claude Code 的用戶 | 自動讀取凭證 | 可能找不到憑證文件 |
| API Key | API 按量付費用戶 | 最直接 | 需要獨立付費 |

---

## 7. 常見問題

### Q: 出現 "credit balance is too low" 錯誤
A: setup-token 無法直接調用 Anthropic API，需要通過 OpenClaw 的代理機制。確保：
1. 使用 `openclaw onboard` 正確配置
2. 重啟 Docker gateway

### Q: 出現 "OAuth authentication is currently not supported" 錯誤
A: 這表示直接用 curl 測試 API 會失敗，但透過 OpenClaw gateway 使用是正常的。

### Q: 如何更新 token（token 過期時）
A: 重新執行：
```bash
claude setup-token      # 生成新 token
openclaw onboard        # 選擇 Update values，貼上新 token
sudo docker compose restart openclaw-gateway
```

---

## 8. SSH 連接指令

```bash
ssh idaka@192.168.0.15
# 密碼: idaka987
# sudo 密碼: idaka987
```

---

## 9. 相關文件位置

| 文件 | 路徑 |
|------|------|
| 主配置 | `/home/idaka/.openclaw/openclaw.json` |
| Docker Compose | `/home/idaka/openclaw_src/docker-compose.yml` |
| 環境變數 | `/home/idaka/openclaw_src/.env` |
| 認證配置 | `/home/idaka/.openclaw/agents/main/agent/auth.json` |
| 記憶文件 | `/home/idaka/.openclaw/workspace/memory/` |
