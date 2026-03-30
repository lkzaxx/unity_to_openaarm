# idaka-System-5080 系統資訊

> 最後更新：2026-03-17
> SSH：`ssh idaka_5080@192.168.0.245` (pw: Qwerty~12345)

### SSH 連線注意事項

Windows 上使用 PuTTY `plink` 連線時，首次連線需處理 host key：

```bash
# 首次連線會因 host key 未快取而失敗，需用 -hostkey 指定 fingerprint
plink -ssh -hostkey "SHA256:YNhhvr/LxVpgn89TIT2NTlTkQebYTYef0PuR6eslE+I" idaka_5080@192.168.0.245 -pw "Qwerty~12345" "hostname"
```

- `echo y | plink ...` 無法正確通過 host key 確認（會 timeout）
- `plink -batch` 遇到未知 host key 會直接拒絕
- **正確做法**：先用 `-batch` 取得 fingerprint，再用 `-hostkey` 參數指定
- 或直接用 PuTTY GUI 連線一次，host key 即會存入 Windows Registry，之後 plink 就不再詢問

---

## 電腦規格

| 項目 | 規格 |
|------|------|
| 主機名稱 | idaka-System-5080 |
| 主機板 | ASUS PRIME B760M-A WIFI D4 |
| CPU | Intel Core i7-14700 (20 核 28 執行緒, 最高 5.4 GHz) |
| RAM | 64 GB DDR4 |
| GPU | NVIDIA GeForce RTX 5080 (16 GB VRAM) |
| 儲存 | NVMe SSD 931.5 GB (已用 35G / 可用 834G) |
| OS | Ubuntu 24.04.4 LTS (Noble Numbat) |
| Kernel | 6.17.0-19-generic (x86_64) |

### GPU 詳細

- NVIDIA Driver: 580.126.09
- CUDA Version: 13.0 (驅動支援版本, nvcc 未安裝)
- nvidia-smi 正常運作

### CPU 快取

- L1d: 768 KiB (20 instances)
- L1i: 1 MiB (20 instances)
- L2: 28 MiB (11 instances)
- L3: 33 MiB (1 instance)

---

## 網路資訊

### 網路介面

| 介面 | 狀態 | IP 位址 | MAC |
|------|------|---------|-----|
| lo | UP | 127.0.0.1/8 | - |
| enp5s0 (有線) | DOWN | 無 | 30:c5:99:b5:46:ff |
| wlp0s20f3 (Wi-Fi) | UP | **192.168.0.245/24** | 64:4a:7d:59:05:42 |
| tailscale0 (VPN) | UP | 100.102.81.110/32 | - |

### 路由

- 預設閘道：`192.168.0.1` (via wlp0s20f3)
- 區域網路：`192.168.0.0/24`

### DNS

- nameserver: 127.0.0.53 (systemd-resolved)
- search domain: tail518e73.ts.net

### Tailscale

- IPv4: 100.102.81.110
- IPv6: fd7a:115c:a1e0::9501:51ab

---

## 已安裝軟體

| 軟體 | 版本 | 備註 |
|------|------|------|
| Python | 3.12.3 | 系統內建 |
| Miniconda3 | conda 26.1.1 | 路徑: ~/miniconda3 |
| NVIDIA Driver | 580.126.09 | |
| CUDA | 13.0 | 驅動支援, nvcc 未安裝 |
| Tailscale | 已安裝 | VPN 已連線 |
| VS Code Server | 已安裝 | ~/.vscode-server |
| NVIDIA Omniverse | 已安裝 | ~/.nvidia-omniverse |
| Git | **未安裝** | |
| Docker | **未安裝** | |

### Conda 環境

| 環境名稱 | 路徑 |
|----------|------|
| base | ~/miniconda3 |
| isaaclab | ~/miniconda3/envs/isaaclab |

### NVIDIA Omniverse / Isaac

- Kit apps: `~/Documents/Kit/apps/`
- Kit shared: `~/Documents/Kit/shared/`
- Omniverse logs/cache: `~/.nvidia-omniverse/`

---

## 檔案結構

### 家目錄 `/home/idaka_5080/`

```
/home/idaka_5080/
├── Desktop/                  (4.0K)
├── Documents/                (52K)
│   └── Kit/
│       ├── apps/
│       └── shared/
├── Downloads/                (4.0K)
├── miniconda3/               (18G)  ← Miniconda 安裝
├── Miniconda3-latest-Linux-x86_64.sh  ← 安裝檔 (155MB)
├── Music/                    (4.0K)
├── Pictures/                 (4.0K)
├── Public/                   (4.0K)
├── Templates/                (4.0K)
├── Videos/                   (4.0K)
├── snap/                     (145M)
├── .nvidia-omniverse/        ← Omniverse 設定
├── .vscode-server/           ← VS Code Remote
├── .conda/
├── .anaconda/
├── .copilot/
├── .dotnet/
└── .ssh/
```

### 根目錄掛載

| 掛載點 | 裝置 | 大小 | 已用 | 可用 | 使用率 |
|--------|------|------|------|------|--------|
| / | /dev/nvme0n1p2 | 915G | 35G | 834G | 5% |
| /boot/efi | /dev/nvme0n1p1 | 1.1G | 6.2M | 1.1G | 1% |

### 磁碟分割

```
nvme0n1       931.5G  disk
├─nvme0n1p1     1G    part  /boot/efi
└─nvme0n1p2   930.5G  part  /
```

---

## 備註

- 這台電腦主要用途可能為 NVIDIA Isaac Sim / Isaac Lab 開發
- 有線網路 (enp5s0) 目前未連接，使用 Wi-Fi 連線
- Tailscale VPN 已啟用
- Swap: 8 GB (目前未使用)
- 建議安裝 git 和 docker 以便開發使用
