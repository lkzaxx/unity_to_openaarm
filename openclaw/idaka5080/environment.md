# idaka_5080 環境說明

> 維護者：Wayne Lin (waynelin-idaka@gmail.com)
> 用途：OpenArm VR Robot - YOLO 訓練 / Isaac Sim / eHand Pose Estimation
> 最後更新：2026-03-21

---

## 系統資訊

| 項目 | 規格 |
|------|------|
| 主機名稱 | idaka-System-5080 |
| OS | Ubuntu 24.04.4 LTS |
| CPU | Intel Core i7-14700 (20 核 28 執行緒) |
| RAM | 64 GB DDR4 |
| GPU | NVIDIA GeForce RTX 5080 (16 GB VRAM) |
| NVIDIA Driver | 580.126.09 |
| CUDA | 13.0 (驅動支援) |
| 儲存 | NVMe SSD 931.5 GB |
| Docker | 29.3.0 |

---

## 我的工作目錄

```
~/openarm_yolo_training/     <-- YOLO 訓練資料、模型、腳本（Wayne 的，請勿修改）
```

---

## Conda 環境

| 環境名 | Python | PyTorch | 用途 |
|--------|--------|---------|------|
| base | - | - | 預設 |
| yolo | 3.11 | 2.10.0+cu128 | YOLO 訓練、SAM、OpenCV、pyrender |
| isaaclab | 3.11 | - | Isaac Sim 5.1 (預裝) |

### yolo 環境主要套件

| 套件 | 版本 |
|------|------|
| ultralytics | 8.4.23 |
| torch | 2.10.0+cu128 |
| opencv-python | 4.13.0 |
| pybullet | 3.2.7 |
| pyrender | 0.1.45 |
| trimesh | 4.11.4 |
| mediapipe | 0.10.14 |

### isaaclab 環境

| 套件 | 版本 |
|------|------|
| isaacsim | 5.1.0 |

---

## Docker 容器

| 容器名 | 映像 | 用途 | 掛載目錄 |
|--------|------|------|---------|
| foundationpose | wenbowen123/foundationpose | 6DoF Pose Estimation | ~/openarm_yolo_training/ only |

---

## apt 安裝的套件

| 套件 | 版本 | 安裝日期 |
|------|------|---------|
| nvidia-jetpack | 6.2.1+b38 | 2026-03-18 |
| cuda-toolkit-12-6 | 12.6.11 | 2026-03-18 |
| tensorrt | 10.3.0 | 2026-03-18 |

---

## 注意事項

- Docker 容器只掛載 ~/openarm_yolo_training/，不掛載 /home 或其他目錄
- 未修改系統級設定
- 有問題請聯繫 Wayne
