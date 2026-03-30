# YOLO 模型訓練環境建置指南

> 目標機器：idaka-System-5080 (192.168.0.245)
> GPU：NVIDIA RTX 5080 16GB / CUDA 13.0 / Driver 580.126.09
> 最後更新：2026-03-18

---

## 0. SSH 免密碼登入設定

### Windows SSH Config

檔案位置：`C:\Users\lkzax\.ssh\config`

```
Host IDAKA_5080
    HostName 192.168.0.245
    User idaka_5080
    Port 22
    IdentityFile ~/.ssh/id_ed25519
```

### 設定流程（已完成 2026-03-18）

```powershell
# 1. 產生 SSH Key（無 passphrase）
ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_ed25519
# → Overwrite? y
# → passphrase? 直接按 Enter（空白）

# 2. 把 public key 傳到 5080（用 plink + 密碼，僅需一次）
plink -ssh -pw "Qwerty~12345" -hostkey "SHA256:YNhhvr/LxVpgn89TIT2NTlTkQebYTYef0PuR6eslE+I" ^
  idaka_5080@192.168.0.245 ^
  "mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo 'PUBLIC_KEY_CONTENT' > ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"

# 3. （選用）啟用 Windows SSH Agent 服務
#    需在管理員 PowerShell 中執行
Set-Service ssh-agent -StartupType Automatic
Start-Service ssh-agent
ssh-add $env:USERPROFILE\.ssh\id_ed25519

# 4. 測試
ssh IDAKA_5080 "hostname && whoami"
# → idaka-System-5080 / idaka_5080
```

### 注意事項

- SSH key 如果有設 passphrase，非互動環境（如 Claude Code）無法輸入 → 建議不設 passphrase 或啟用 ssh-agent
- `ssh-copy-id` 在 Windows Git Bash 可用，但需要能互動輸入密碼
- PuTTY `plink` 可帶 `-pw` 參數，適合首次傳 key 時使用
- host key fingerprint: `SHA256:YNhhvr/LxVpgn89TIT2NTlTkQebYTYef0PuR6eslE+I`

### 連線方式

```bash
# 免密碼登入
ssh IDAKA_5080

# SCP 上傳檔案
scp local_file.txt IDAKA_5080:~/path/

# SCP 下載檔案
scp IDAKA_5080:~/path/remote_file.txt ./
```

---

## 工作目錄結構

所有程式碼、資料集、模型統一放在 `~/openarm_yolo_training/`：

```
/home/idaka_5080/yolo_training/
├── datasets/    ← 訓練/驗證資料集
├── models/      ← 預訓練模型 (.pt)
├── scripts/     ← 訓練/推論程式碼
├── runs/        ← 訓練輸出結果 (weights, logs)
└── exports/     ← 匯出模型 (ONNX, TensorRT)
```

---

## 1. 前置安裝

SSH 進入機器後，先安裝缺少的基本工具：

```bash
sudo apt update && sudo apt install -y git curl unzip
```

## 2. 安裝 CUDA Toolkit

驅動已裝好 (580.126.09)，只需安裝 CUDA Toolkit（不要重裝驅動）：

```bash
# 安裝 CUDA Toolkit（僅 toolkit，不含驅動）
sudo apt install -y nvidia-cuda-toolkit
```

驗證：

```bash
nvcc --version
```

## 3. 建立 Conda 環境

```bash
# 啟用 conda（如果 shell 還沒初始化）
~/miniconda3/bin/conda init bash
source ~/.bashrc

# 建立 YOLO 專用環境
conda create -n yolo python=3.11 -y
conda activate yolo
```

> 使用 Python 3.11 以確保與 PyTorch / Ultralytics 的相容性。

## 4. 安裝 PyTorch + Ultralytics

```bash
# 安裝 PyTorch（CUDA 12.8 — 向下相容 CUDA 13.0 驅動）
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128

# 驗證 GPU 可用
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"

# 安裝 Ultralytics (YOLOv8/v11)
pip install ultralytics
```

## 5. 驗證環境

```bash
yolo checks
```

預期輸出應顯示：
- Ultralytics 版本
- Python 3.11
- torch (CUDA)
- GPU: NVIDIA GeForce RTX 5080

---

## 6. 準備資料集

### 資料集結構（YOLO 格式）

```
~/openarm_yolo_training/datasets/
└── my_dataset/
    ├── images/
    │   ├── train/
    │   │   ├── img001.jpg
    │   │   └── ...
    │   └── val/
    │       ├── img101.jpg
    │       └── ...
    ├── labels/
    │   ├── train/
    │   │   ├── img001.txt    ← 每行: class x_center y_center width height
    │   │   └── ...
    │   └── val/
    │       ├── img101.txt
    │       └── ...
    └── data.yaml
```

### data.yaml 範例

```yaml
path: /home/idaka_5080/yolo_training/datasets/my_dataset
train: images/train
val: images/val

names:
  0: class_a
  1: class_b
  2: class_c
```

### 上傳資料集到機器

```bash
# 從本機（Windows）用 scp 上傳
scp -r ./my_dataset idaka_5080@192.168.0.245:~/openarm_yolo_training/datasets/
```

---

## 7. 訓練模型

### 基本訓練（命令列）

```bash
conda activate yolo

# YOLOv8n（Nano，快速測試用）
yolo detect train model=yolov8n.pt data=~/openarm_yolo_training/datasets/my_dataset/data.yaml epochs=100 imgsz=640

# YOLOv8s（Small，平衡速度與精度）
yolo detect train model=yolov8s.pt data=~/openarm_yolo_training/datasets/my_dataset/data.yaml epochs=100 imgsz=640

# YOLOv8m（Medium，精度更高）
yolo detect train model=yolov8m.pt data=~/openarm_yolo_training/datasets/my_dataset/data.yaml epochs=200 imgsz=640

# YOLOv11（最新版）
yolo detect train model=yolo11n.pt data=~/openarm_yolo_training/datasets/my_dataset/data.yaml epochs=100 imgsz=640
```

### RTX 5080 建議參數

| 模型 | batch size | imgsz | 預估 VRAM 使用 |
|------|-----------|-------|---------------|
| yolov8n | 64 | 640 | ~4 GB |
| yolov8s | 32 | 640 | ~6 GB |
| yolov8m | 16 | 640 | ~10 GB |
| yolov8l | 8 | 640 | ~14 GB |
| yolov8x | 4-8 | 640 | ~16 GB |

```bash
# 範例：用 YOLOv8m 搭配建議 batch size
yolo detect train \
  model=yolov8m.pt \
  data=~/openarm_yolo_training/datasets/my_dataset/data.yaml \
  epochs=200 \
  imgsz=640 \
  batch=16 \
  workers=8 \
  device=0 \
  project=~/openarm_yolo_training/runs \
  name=my_experiment
```

### Python 腳本方式

```python
from ultralytics import YOLO

model = YOLO("yolov8m.pt")

results = model.train(
    data="/home/idaka_5080/yolo_training/datasets/my_dataset/data.yaml",
    epochs=200,
    imgsz=640,
    batch=16,
    workers=8,
    device=0,
    project="/home/idaka_5080/yolo_training/runs",
    name="my_experiment",
)
```

---

## 8. 監控訓練

### 即時 GPU 監控

```bash
# 每 1 秒刷新
watch -n 1 nvidia-smi
```

### TensorBoard

```bash
pip install tensorboard
tensorboard --logdir ~/openarm_yolo_training/runs --host 0.0.0.0 --port 6006
```

從 Windows 瀏覽器訪問：`http://192.168.0.245:6006`

---

## 9. 評估與推論

```bash
# 驗證模型
yolo detect val model=~/openarm_yolo_training/runs/my_experiment/weights/best.pt data=~/openarm_yolo_training/datasets/my_dataset/data.yaml

# 單張圖片推論
yolo detect predict model=~/openarm_yolo_training/runs/my_experiment/weights/best.pt source=./test_image.jpg

# 即時攝影機推論
yolo detect predict model=~/openarm_yolo_training/runs/my_experiment/weights/best.pt source=0 show=True
```

---

## 10. 匯出模型

```bash
# 匯出為 ONNX（通用部署）
yolo export model=~/openarm_yolo_training/runs/my_experiment/weights/best.pt format=onnx

# 匯出為 TensorRT（NVIDIA 最佳化推論）
yolo export model=~/openarm_yolo_training/runs/my_experiment/weights/best.pt format=engine

# 匯出為 NCNN（手機/嵌入式）
yolo export model=~/openarm_yolo_training/runs/my_experiment/weights/best.pt format=ncnn
```

---

## 快速開始（完整一鍵流程）

```bash
# SSH 進入
ssh idaka_5080@192.168.0.245

# 前置安裝（僅第一次）
sudo apt update && sudo apt install -y git nvidia-cuda-toolkit
~/miniconda3/bin/conda init bash && source ~/.bashrc
conda create -n yolo python=3.11 -y
conda activate yolo
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics

# 驗證
yolo checks

# 開始訓練（替換 data.yaml 路徑）
yolo detect train model=yolov8m.pt data=./data.yaml epochs=200 imgsz=640 batch=16
```

---

## 備註

- RTX 5080 的 16GB VRAM 可以流暢訓練到 YOLOv8l，YOLOv8x 需要降低 batch size
- 如需更大模型，可用 `batch=-1` 讓 Ultralytics 自動偵測最大 batch size
- 訓練結果存放在 `~/openarm_yolo_training/runs/` 目錄下
- 如要部署到 Jetson Orin Nano，建議匯出為 TensorRT engine 格式

---

## 11. 罐子計數專案：SAM + 合成資料 + ByteTrack

> 目標：用極少量真實照片，透過合成資料訓練 YOLOv11，搭配 ByteTrack 實現自動罐子計數。

### 整體架構

```
拍攝罐子照片 → SAM 去背切割 → 合成資料腳本（貼圖到隨機背景）→ YOLOv11n 訓練 → ByteTrack 追蹤 + 虛擬計數線
```

### 實際執行流程（腳本位置：`scripts/can_training/`）

```
Windows (本機)                         5080 (遠端 RTX 5080)
───────────                           ──────────────────────
                                      00_setup.sh (僅第一次)
                                      conda create + pip install
                                            │
pic/can/*.jpg ──── scp 上傳 ──────→  ~/openarm_yolo_training/datasets/can_source/
scripts/can_training/ ── scp ─────→  ~/openarm_yolo_training/scripts/can_training/
                                            │
                                            ▼
                                      01_sam_cutout.py
                                      (YOLO 找到 "bottle" bbox → SAM2 精確分割)
                                      (備用: 01b_sam_manual.py 手動點擊模式)
                                            │
                                            ▼
                                      ~/openarm_yolo_training/datasets/can_cutouts/
                                      cutout_000.png ~ cutout_006.png (RGBA 去背圖)
                                            │
                                            ▼
                                      02_gen_synthetic.py
                                      (隨機貼到真實背景 + 自動生成 YOLO 標註)
                                      (800 train + 100 val = 900 張)
                                      (SCALE 0.20~0.65, ROTATION ±60°)
                                            │
                                            ▼
                                      ~/openarm_yolo_training/datasets/can_dataset_v3/
                                      ├── images/{train,val}/syn_XXXX.jpg
                                      ├── labels/{train,val}/syn_XXXX.txt
                                      └── data.yaml
                                            │
                                            ▼
                                      03_train.py
                                      (YOLOv11n, epochs=200, batch=64, ~4 分鐘)
                                            │
                                            ▼
                                      ~/openarm_yolo_training/runs/can_v3/weights/best.pt
                                            │
                                            ▼
                                      04_validate.py
                                      (mAP 評估 + 用原始照片測試推論)
                                            │
scp 下載 ←────────────────────────── best.pt (訓練好的模型)
```

### 逐步執行指令

#### Step 0: 環境建置（5080 上，僅第一次）

```bash
ssh IDAKA_5080
bash ~/openarm_yolo_training/scripts/can_training/00_setup.sh
```

#### Step 1: 上傳圖片 + 腳本（Windows 上）

```bash
# 建立遠端目錄
ssh IDAKA_5080 "mkdir -p ~/openarm_yolo_training/{datasets/can_source,scripts}"

# 上傳來源圖片（排除 yolo_ 開頭）
scp idaka5080/pic/can/cam_*.jpg IDAKA_5080:~/openarm_yolo_training/datasets/can_source/
scp idaka5080/pic/can/check_view_*.png IDAKA_5080:~/openarm_yolo_training/datasets/can_source/
scp idaka5080/pic/can/realsense_*.png IDAKA_5080:~/openarm_yolo_training/datasets/can_source/

# 上傳訓練腳本
scp -r idaka5080/scripts/can_training/ IDAKA_5080:~/openarm_yolo_training/scripts/
```

#### Step 2: SSH 進 5080 依序執行

```bash
ssh IDAKA_5080
conda activate yolo
cd ~/openarm_yolo_training/scripts/can_training

# A) SAM 分割去背（自動模式）
python 01_sam_cutout.py
# → 如果自動偵測失敗，改用手動模式:
# python 01b_sam_manual.py

# B) 檢查去背結果
ls ~/openarm_yolo_training/datasets/can_cutouts/
# → 應有 cutout_000.png ~ cutout_006.png

# C) 生成合成訓練資料 (500 張)
python 02_gen_synthetic.py

# D) 訓練 YOLOv11n (~5-10 分鐘)
python 03_train.py

# E) 驗證模型
python 04_validate.py
```

#### Step 3: 下載模型（Windows 上）

```bash
scp IDAKA_5080:~/openarm_yolo_training/runs/can_v3/weights/best.pt ./can_best.pt
```

### 故障排除

| 問題 | 解法 |
|------|------|
| SAM 沒偵測到罐子 | 用 `01b_sam_manual.py` 手動點擊罐子中心 |
| CUDA out of memory | 降低 `03_train.py` 中 `BATCH_SIZE` (64→32) |
| mAP < 0.5 | 檢查 cutouts 品質 / 增加合成數量到 1000 / 加真實背景圖 |
| 真實圖片偵測不到 | 補拍更多角度 / 用少量真實標註圖 fine-tune |
| SSH 連線失敗 | 確認同網段 192.168.0.x / 檢查 5080 是否開機 |

---

### 訓練迭代記錄（2026-03-18）

#### v1：程式化背景（失敗）

```
合成參數: 450 train, SCALE 0.08~0.30, ROTATION ±30°, MAX_OBJECTS 8, 程式化背景
訓練參數: yolo11n.pt, epochs=150, batch=64, imgsz=640
mAP50: 0.995 (合成驗證集)
```

**問題：**
- 真實圖片每張偵測 13~24 個 "can" → 大量背景誤報（箱子、地板、架子都被框）
- 罐子被分成上下兩半各標一次（蓋子 + 瓶身）
- 原因：程式化背景（純色/漸層/噪點）跟真實環境差異太大，模型沒學到「什麼不是罐子」

#### v2：真實背景 + 減少物件數（部分改善）

```
合成參數: 450 train, SCALE 0.15~0.40, ROTATION ±20°, MAX_OBJECTS 3, 80% 真實背景
背景來源: 用 OpenCV inpainting 將原始 7 張照片中的罐子塗掉
訓練參數: yolo11n.pt, epochs=150, batch=64, imgsz=640
```

**改善：** 背景誤報完全消除
**殘留問題：**
- `realsense_135610`（斜罐）完全沒偵測到 → 旋轉角度不夠
- `realsense_135835` 等圖罐子被分成上下兩框（0.82 + 0.74）→ 合成時罐子太小
- 原因：SCALE_MAX=0.40 太小，真實圖中罐子佔畫面比例更大；ROTATION ±20° 不涵蓋斜放

#### v3：加大縮放 + 旋轉範圍（成功 ✓）

```
合成參數: 800 train + 100 val, SCALE 0.20~0.65, ROTATION ±60°, MAX_OBJECTS 3, 80% 真實背景
訓練參數: yolo11n.pt, epochs=200 (early stop at 72), batch=64, imgsz=640, patience=40
最終模型: ~/openarm_yolo_training/runs/can_v3/weights/best.pt (5.5MB)
```

**結果：7 張真實照片全部正確偵測**

| 圖片 | v1 | v2 | v3 |
|------|----|----|-----|
| cam (webcam) | 19 誤報 | 1, can 0.92 | **1, can 0.95** |
| realsense_135437 | 19 誤報 | 2 (分框) | **1, can 正確** |
| realsense_135610 (斜) | 20 誤報 | **0 漏偵測** | **1, can 0.93** |
| realsense_135644 | 17 誤報 | 2 (分框) | **1, can 正確** |
| realsense_135719 (斜) | 17 誤報 | 1, can 0.51 | **1, can 0.94** |
| realsense_135835 | 13 誤報 | 3 (分框) | **1, can 0.95** |
| check_view (廣角) | 24 誤報 | 1, can 正確 | 2 (小瑕疵) |
| 背景誤報 | 滿滿 | 0 | **0** |

#### 調參經驗總結

| 參數 | 作用 | 踩坑經驗 |
|------|------|---------|
| **SCALE_MAX** | 合成圖中物件最大佔比 | 設太小 (0.30~0.40) → 模型只學到小物件，大物件被分成多框。**設 0.60~0.65 最佳** |
| **ROTATION_RANGE** | 合成時隨機旋轉角度 | 設太小 (±20°) → 斜放/躺倒的物件偵測不到。**設 ±60° 涵蓋各種姿態** |
| **MAX_OBJECTS** | 每張圖放幾個物件 | 設太多 (6~8) → 模型覺得到處都是目標，誤報多。**設 1~3 較佳** |
| **背景圖來源** | 合成圖的底圖 | 純程式化背景 → 大量誤報。**必須用真實環境照片（用 inpainting 塗掉目標物）** |
| **NUM_TRAIN** | 訓練圖片數量 | 500 張可以跑，**800~1000 張更穩定** |
| epochs | 訓練回合數 | 設 200 + patience=40，實際 early stop 在 72 epoch。**不用刻意調高** |
| batch | 批次大小 | RTX 5080 跑 yolo11n batch=64 只用 ~9GB VRAM，**不需要降** |

---

### 開發藍圖（5 步驟）

#### Step 1：準備素材

拍攝罐子照片，建議：
- 每種罐子至少 3-5 張（正面、側面、俯視、45度、躺倒）
- 光線均勻、背景乾淨（方便 SAM 分割）
- 使用 RealSense RGB 或一般相機皆可

現有素材（`pic/can/` 目錄，7 張可用）：
- `cam_20260318_101230.jpg` — Webcam 俯視
- `check_view_1773647898.png` — RealSense 概覽
- `realsense_20260318_1354xx~1358xx.png` x5 — RealSense 俯視，同一罐子略微不同角度
- `yolo_20260318_135835.png` — 預訓練 YOLO 偵測結果（偵測為 "bottle" 0.35）**← 排除不用**

**現有圖片評估：可以開始，但建議補拍**
- 優點：有 7 張不同角度的來源圖
- 不足：全部都是俯視角、同一背景、同一種罐子、無躺倒姿態
- 建議：補拍正面平視、側面、躺倒各 2 張（約 10 分鐘）

#### Step 2：SAM 自動分割去背

安裝 SAM 2（Meta 最新版）：

```bash
conda activate yolo
pip install segment-anything-2
```

或使用 Grounded-SAM（文字提示自動偵測 + 分割）：

```bash
pip install grounded-sam-2
```

SAM 去背腳本範例：

```python
import numpy as np
from PIL import Image
from sam2.sam2_image_predictor import SAM2ImagePredictor

# 載入模型
predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2-hiera-large")

# 載入圖片
image = Image.open("can_photo.jpg")
predictor.set_image(np.array(image))

# 用點擊提示（罐子中心座標）
masks, scores, _ = predictor.predict(
    point_coords=np.array([[320, 240]]),  # 罐子中心 (x, y)
    point_labels=np.array([1]),            # 1 = 前景
)

# 取最高分的 mask
best_mask = masks[scores.argmax()]

# 儲存去背結果（RGBA PNG）
image_array = np.array(image)
rgba = np.concatenate([image_array, (best_mask * 255).astype(np.uint8)[..., None]], axis=-1)
Image.fromarray(rgba).save("can_cutout.png")
```

#### Step 3：合成資料生成

將切出的罐子隨機貼到不同背景上，自動生成 YOLO 標註：

```python
import os
import random
from PIL import Image, ImageEnhance, ImageFilter

def generate_synthetic_dataset(
    cutout_paths,       # 去背罐子圖片列表 (RGBA PNG)
    bg_dir,             # 背景圖片目錄
    output_dir,         # 輸出目錄
    num_images=500,     # 生成圖片數量
    max_objects=8,      # 每張最多放幾個罐子
    img_size=640,       # 輸出圖片大小
):
    os.makedirs(f"{output_dir}/images/train", exist_ok=True)
    os.makedirs(f"{output_dir}/images/val", exist_ok=True)
    os.makedirs(f"{output_dir}/labels/train", exist_ok=True)
    os.makedirs(f"{output_dir}/labels/val", exist_ok=True)

    cutouts = [Image.open(p).convert("RGBA") for p in cutout_paths]
    backgrounds = [os.path.join(bg_dir, f) for f in os.listdir(bg_dir)
                   if f.lower().endswith(('.jpg', '.png'))]

    for i in range(num_images):
        # 隨機選背景
        bg = Image.open(random.choice(backgrounds)).convert("RGB")
        bg = bg.resize((img_size, img_size))

        labels = []
        num_obj = random.randint(1, max_objects)

        for _ in range(num_obj):
            cutout = random.choice(cutouts).copy()

            # 隨機變換
            scale = random.uniform(0.08, 0.3)
            new_w = int(cutout.width * scale)
            new_h = int(cutout.height * scale)
            cutout = cutout.resize((new_w, new_h))

            # 隨機旋轉
            angle = random.uniform(-30, 30)
            cutout = cutout.rotate(angle, expand=True, resample=Image.BICUBIC)

            # 隨機亮度/對比度
            enhancer = ImageEnhance.Brightness(cutout)
            cutout = enhancer.enhance(random.uniform(0.7, 1.3))

            # 隨機位置
            max_x = img_size - cutout.width
            max_y = img_size - cutout.height
            if max_x <= 0 or max_y <= 0:
                continue
            x = random.randint(0, max_x)
            y = random.randint(0, max_y)

            # 貼上
            bg.paste(cutout, (x, y), cutout)

            # 生成 YOLO 標註 (class x_center y_center width height) 歸一化
            cx = (x + cutout.width / 2) / img_size
            cy = (y + cutout.height / 2) / img_size
            w = cutout.width / img_size
            h = cutout.height / img_size
            labels.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

        # 分割 train/val (90/10)
        split = "val" if i % 10 == 0 else "train"
        bg.save(f"{output_dir}/images/{split}/syn_{i:04d}.jpg")
        with open(f"{output_dir}/labels/{split}/syn_{i:04d}.txt", "w") as f:
            f.write("\n".join(labels))

    # 生成 data.yaml
    with open(f"{output_dir}/data.yaml", "w") as f:
        f.write(f"path: {os.path.abspath(output_dir)}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n\n")
        f.write("names:\n  0: can\n")

    print(f"Done! Generated {num_images} images in {output_dir}")
```

**背景圖片來源建議：**
- 拍攝實際產線/輸送帶表面照片（最佳）
- 或下載通用工業背景圖（如純色桌面、金屬表面等）

#### Step 4：訓練 YOLOv11n

```bash
conda activate yolo

# 用合成資料訓練
yolo detect train \
  model=yolo11n.pt \
  data=~/openarm_yolo_training/datasets/can_dataset/data.yaml \
  epochs=100 \
  imgsz=640 \
  batch=64 \
  workers=8 \
  device=0 \
  project=~/openarm_yolo_training/runs \
  name=can_detect_v1

# （可選）用少量真實標註圖 fine-tune
yolo detect train \
  model=~/openarm_yolo_training/runs/can_detect_v1/weights/best.pt \
  data=~/openarm_yolo_training/datasets/can_real/data.yaml \
  epochs=50 \
  imgsz=640 \
  batch=64 \
  device=0 \
  project=~/openarm_yolo_training/runs \
  name=can_detect_v1_finetune
```

#### Step 5：ByteTrack 追蹤 + 虛擬計數線

Ultralytics 已內建 ByteTrack 支援：

```python
import cv2
from ultralytics import YOLO
from ultralytics.solutions import ObjectCounter

# 載入訓練好的模型
model = YOLO("~/openarm_yolo_training/runs/can_detect_v1/weights/best.pt")

# 方法 A：手動追蹤 + 計數線
cap = cv2.VideoCapture(0)  # 或影片路徑
count = 0
counted_ids = set()
LINE_Y = 300  # 虛擬計數線 Y 座標

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 追蹤（自動使用 ByteTrack）
    results = model.track(frame, persist=True, tracker="bytetrack.yaml")

    if results[0].boxes.id is not None:
        boxes = results[0].boxes.xyxy.cpu().numpy()
        ids = results[0].boxes.id.cpu().numpy().astype(int)

        for box, obj_id in zip(boxes, ids):
            cy = (box[1] + box[3]) / 2  # 物體中心 Y
            # 物體越過計數線且尚未計數
            if cy > LINE_Y and obj_id not in counted_ids:
                count += 1
                counted_ids.add(obj_id)

    # 畫計數線
    cv2.line(frame, (0, LINE_Y), (frame.shape[1], LINE_Y), (0, 0, 255), 2)
    cv2.putText(frame, f"Count: {count}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

    annotated = results[0].plot()
    cv2.imshow("Can Counter", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"Total cans counted: {count}")
```

```python
# 方法 B：使用 Ultralytics 內建 ObjectCounter（更簡潔）
from ultralytics import solutions

counter = solutions.ObjectCounter(
    model="~/openarm_yolo_training/runs/can_detect_v1/weights/best.pt",
    region=[(0, 300), (640, 300)],  # 計數線座標
    show=True,
)
counter.count(source="video.mp4")
```

### 依賴套件總表

```bash
conda activate yolo
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics
pip install segment-anything-2    # SAM 2
pip install opencv-python-headless
pip install Pillow
pip install lapx                  # ByteTrack 需要
```

### 部署選項

| 目標平台 | 匯出格式 | 指令 |
|----------|---------|------|
| PC (RTX 5080) | PyTorch (.pt) | 直接使用 |
| Jetson Orin Nano | TensorRT (.engine) | `yolo export format=engine device=0` |
| 通用邊緣裝置 | ONNX (.onnx) | `yolo export format=onnx` |

---

## 11.5 Jetson 部署：環境狀態與加速方案

> 模型位置：Jetson `~/ros2_ws/yolo_model/`
> 確認日期：2026-03-19

### Jetson 硬體與系統資訊

| 項目 | 規格 |
|------|------|
| 型號 | NVIDIA Jetson Orin Nano Super (8GB) |
| SoC | Tegra234 (p3767-0005-super) |
| RAM | 7.4 GB |
| 磁碟 | 456 GB NVMe（已用 47GB / 剩 387GB） |
| L4T | R36.4.4 (nvidia-l4t-core 36.4.4) |
| JetPack | 6.1 → 可升級 6.2.1 |
| NVIDIA apt repo | `repo.download.nvidia.com/jetson/common r36.4` |

### Jetson 環境狀態（2026-03-18 完成建置）

| 套件 | 版本 | 狀態 |
|------|------|------|
| ultralytics | 8.4.22 | OK |
| opencv | 4.13.0 | OK |
| pyrealsense2 | 已安裝 | OK |
| cuDNN | 9.3.0 (CUDA 12) | OK |
| CUDA Toolkit | 12.6.11 | OK（nvidia-jetpack 安裝） |
| TensorRT | 10.3.0 | OK（nvidia-jetpack 安裝） |
| nvidia-jetpack | 6.2.1+b38 | OK |
| torch | 2.3.0 (CUDA 12.4, aarch64) | OK（NVIDIA 官方 wheel） |
| torchvision | 0.18.0a0 (源碼建置) | OK |
| onnx | 1.20.1 | OK |

### 加速方案

| 方案 | 做法 | 推論速度 | 備註 |
|------|------|---------|------|
| **A. 裝 Jetson 版 PyTorch** | 安裝 NVIDIA 提供的 JetPack PyTorch wheel | ~15-30ms/frame | 需對應 JetPack 版本 |
| **B. 匯出 TensorRT** | 在 Jetson 上把 .pt 轉成 .engine | ~5-10ms/frame（最快） | 需先完成方案 A |
| C. 先用 CPU 測試 | 不改，先確認流程能跑 | ~200ms/frame (~5fps) | 適合驗證流程 |

### 方案 A+B 安裝流程（推薦）

#### Step 1: 安裝 nvidia-jetpack（一次搞定 CUDA + TensorRT）✅ 已完成

```bash
ssh IDAKA_ROBOT
echo 'idaka987' | sudo -S apt update
echo 'idaka987' | sudo -S apt install -y nvidia-jetpack
```

#### Step 2: 安裝 JetPack 版 PyTorch ✅ 已完成

Jetson **不能用 pip 預設的 PyTorch**（那是 x86 CPU 版）。必須用 NVIDIA 官方 aarch64 wheel：

```bash
# 移除 x86 CPU 版
pip3 uninstall torch torchvision -y

# 下載 NVIDIA 官方 JetPack 6 PyTorch wheel (CUDA 12.4, aarch64)
cd /tmp
wget -O torch-2.3.0-cp310-cp310-linux_aarch64.whl \
  'https://nvidia.box.com/shared/static/zvultzsmd4iuheykxy17s4l2n91ylpl8.whl'
pip3 install --no-cache-dir torch-2.3.0-cp310-cp310-linux_aarch64.whl

# 從源碼建置 torchvision（pip 的 x86 版跟 Jetson torch 不相容）
pip3 install --no-cache-dir setuptools wheel
pip3 install --no-cache-dir --no-build-isolation 'git+https://github.com/pytorch/vision.git@v0.18.0'

# 安裝 onnx（TensorRT 匯出需要）
pip3 install --no-cache-dir onnx onnxruntime

# 驗證
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# → True Orin
```

**踩坑記錄：**
- `pip install torch` 會抓 PyPI 上的 x86 CPU 版（即使指定 `--extra-index-url` Jetson AI Lab repo）
- 必須用 `wget` 下載 `.whl` 再 `pip install` 本地檔案
- `torchvision` 也不能用 pip 預建版（C++ 擴展跟 Jetson torch 不相容），必須 `--no-build-isolation` 從源碼建置
- Jetson AI Lab repo (`pypi.jetson-ai-lab.dev`) DNS 可能解析失敗，改用 `nvidia.box.com` 直鏈更可靠

#### Step 3: 匯出 TensorRT engine ✅ 已完成

```bash
cd ~/ros2_ws/yolo_model
python3 -c "
from ultralytics import YOLO
model = YOLO('can_v3_best.pt')
model.export(format='engine', half=True)
"
# → .pt → .onnx → .engine (在 Jetson 上約 8 分鐘)
# → TensorRT engine 必須在目標裝置上建置，不能跨平台
```

#### 模型檔案（~/ros2_ws/yolo_model/）

| 檔案 | 大小 | 用途 |
|------|------|------|
| can_v3_best.pt | 5.3 MB | Detect v3 PyTorch（舊版保留） |
| can_v3_best.engine | 8.5 MB | Detect v3 TensorRT |
| can_v4_best.pt | 5.3 MB | Detect v4 PyTorch（手指遮擋增強） |
| can_v4_best.engine | 8.5 MB | Detect v4 TensorRT |
| **can_pose_v1_best.pt** | **5.5 MB** | **Pose v1 PyTorch（6 關鍵點）** |
| **can_pose_v1_best.engine** | **9.0 MB** | **Pose v1 TensorRT（目前使用）** |
| can_tracker_config.yaml | - | 自訂追蹤器參數 |
| realsense_yolo_viewer.py | - | Web 即時追蹤 viewer |
| can_tracker.py | - | 獨立追蹤腳本 |

### 訓練版本演進

| 版本 | 模型類型 | cutouts | 合成數量 | 特色 | 結果 |
|------|---------|---------|---------|------|------|
| v1 | Detect | 7 (俯視) | 500 | 程式化背景 | 大量誤報 |
| v2 | Detect | 7 | 500 | 真實背景 (inpainting) | 誤報消除，但分框 |
| v3 | Detect | 7 | 900 | SCALE 0.65, ROTATION ±60° | 7/7 正確 |
| v4 | Detect | 43 (含影片) | 1100 | 手指遮擋 40% | 多角度支援 |
| **pose_v1** | **Pose** | **43** | **1100** | **6 關鍵點 + 骨架** | **bbox + 結構點** |

### YOLO Pose 模型：罐子關鍵點追蹤

> 用 Detect 模型的 cutout mask 自動提取關鍵點，零手動標註。

#### 6 個關鍵點定義

```
        ●0 top (紅)
       ╱│╲
      ╱ ●1 cap_center (紫)
     ╱  │  ╲
 ●2 left ●4 center (青)  ●3 right (綠)
     ╲  │  ╱
      ╲ │ ╱
        ●5 bottom (紅)
```

| 點 | 名稱 | 自動提取方式 |
|---|------|------------|
| 0 | top | mask 最上面 pixel 的中心 |
| 1 | cap_center | 上方 15% 區域的質心 |
| 2 | left | mask 中段最左邊 pixel |
| 3 | right | mask 中段最右邊 pixel |
| 4 | center | mask 整體質心 |
| 5 | bottom | mask 最下面 pixel 的中心 |

#### 為什麼用 Pose 而不是純 Detect

- **遮擋抗性**：手遮住下半部時，模型看到上方關鍵點仍可推測整體
- **姿態資訊**：知道罐子是直立、傾斜還是躺倒
- **結構約束**：模型學會關鍵點之間的幾何關係，減少誤報

#### YOLO Pose 標註格式

```
# 每行: class cx cy w h x0 y0 v0 x1 y1 v1 ... x5 y5 v5
# v: 0=不可見  1=被遮擋但有標  2=可見
0 0.50 0.40 0.15 0.30 0.50 0.25 2 0.50 0.28 2 0.43 0.40 2 0.57 0.40 2 0.50 0.40 2 0.50 0.55 2
```

#### data.yaml（Pose 格式）

```yaml
path: .../can_dataset_pose
train: images/train
val: images/val

kpt_shape: [6, 3]              # 6 個關鍵點, 每個 3 個值 (x, y, visibility)
flip_idx: [0, 1, 3, 2, 4, 5]  # 水平翻轉時 left(2) ↔ right(3)

names:
  0: can
```

### 追蹤器：ByteTrack vs BoT-SORT

追蹤器**不需要訓練**，是純演算法。它的工作是在每一幀之間匹配物件，維持 ID。

#### Kalman Filter（追蹤器的核心）

```
追蹤器怎麼知道「這幀的罐子 = 上幀的罐子」？

幀 1: 罐子在 (100, 200), 速度 (10, 0)
         │
         ▼  Kalman Filter 預測：下一幀應該在 (110, 200)
         │
幀 2: YOLO 偵測到 (112, 201) → 跟預測接近 → 是同一個！ID 不變
         │
         ▼  Kalman Filter 預測：(122, 201)
         │
幀 3: YOLO 沒偵測到 → 用預測值 (122, 201) 暫時頂替 → ID 保留
         │
幀 4: YOLO 偵測到 (130, 202) → 匹配回來 → ID 接上
```

**Kalman Filter = 「預測 + 修正」的數學模型**，讓物件短暫消失時還能追蹤。

#### ByteTrack vs BoT-SORT 比較

| | ByteTrack | BoT-SORT |
|---|---|---|
| 匹配方式 | 只看位置 (IoU) | 位置 + 外觀特徵 |
| 速度 | 更快 | 稍慢 |
| Pose 相容性 | **有 bug：異常 bbox 導致 Kalman Filter 崩潰** | 更穩健 |
| 推薦用途 | Detect 模型 | **Pose 模型（推薦）** |

#### 踩坑：ByteTrack + Pose 模型的 Kalman Filter 崩潰

```
問題鏈:
Pose 模型偶爾輸出異常 bbox (寬=0 或高=0)
  → ByteTrack 做 ret[2] /= ret[3] (寬高比計算)
  → 除以零 (divide by zero)
  → Kalman Filter 矩陣不正定 (not positive definite)
  → crash: numpy.linalg.LinAlgError
```

**解法：**
1. Pose 模型改用 `botsort.yaml`（處理異常 bbox 更穩健）
2. 加 try/except，tracker 出錯時 fallback 到純偵測

```python
# Pose 模型用 BoT-SORT
try:
    results = model.track(frame, persist=True, tracker="botsort.yaml", conf=0.8)
except Exception:
    results = model.predict(frame, conf=0.8)  # fallback
```

### Web Viewer（realsense_yolo_viewer.py）

> 位置：Jetson `~/realsense/realsense_yolo_viewer.py`
> 啟動：`python3 ~/realsense/realsense_yolo_viewer.py`
> 瀏覽：`http://192.168.0.15:8081/`

#### 模式

| 按鈕 | 功能 |
|------|------|
| RGB | 原始彩色畫面 |
| Depth | 深度圖 |
| RGB+Depth | 疊合 |
| **YOLO Track** | **罐子追蹤 + 關鍵點骨架 + 計數線** |
| **YOLO+Depth** | **追蹤 + 深度疊合** |
| Snapshot | 截圖 |
| Reset Count | 重置計數器 |

#### API

| 端點 | 回傳 |
|------|------|
| `/yolo_status` | `{"count": 3, "detections": 1, "inference_ms": 8, "active_ids": [1]}` |
| `/reset_count` | 重置計數為 0 |
| `/imu` | IMU 加速度計 + 陀螺儀數據 |

---

## 12. OpenArm 機械臂追蹤：RealSense + YOLO + 3D 定位

> 目標：用 RealSense D435i 追蹤 OpenArm 機械臂（尤其是末端執行器），取得 3D 座標並回饋給 ROS 2 / Unity。

### 三種方案比較

| | 方案 A：YOLO 2D 追蹤 | 方案 B：YOLO + Depth 3D | 方案 C：AprilTag/ArUco |
|---|---|---|---|
| **做法** | YOLO 偵測 + ByteTrack | YOLO 偵測 + depth 反投影 3D | 末端貼 marker + pose estimation |
| **輸出** | 2D bbox + 追蹤 ID | 3D 座標 (相機座標系) | 6DoF pose (位置+姿態) |
| **精度** | 低（pixel 級） | 中（~1-2cm） | 高（~mm 級） |
| **穩定性** | 遮擋時易跳 ID | 遮擋時易跳 ID | 遮擋時丟失，但不跳 |
| **適用場景** | Demo、UI 顯示、安全監看 | 粗略閉迴路、避障 | 精確控制、手眼校正 |
| **訓練成本** | 需訓練 YOLO | 需訓練 YOLO | 零訓練 |
| **與 ROS/Unity 對接** | 困難 | 可行（需外參） | 最容易 |

### 推薦策略：方案 B + C 混合

```
RealSense D435i
├── RGB → YOLO11 偵測 [ehand, openarm_wrist, can] → ByteTrack 追蹤
├── Depth → align to color → 偵測點深度反投影 → 3D 座標 (相機座標系)
├── AprilTag → 末端精確 6DoF pose（高精度需求時）
└── 外參標定 T_camera_base → 轉換到 robot base frame → ROS 2 TF / Unity
```

**為什麼混合：**
- YOLO 負責「看到什麼」— 罐子、夾爪、手腕的 2D 偵測 + 追蹤
- Depth 反投影負責「在哪裡」— 從 pixel 算出 3D 座標
- AprilTag 負責「精確在哪」— 末端 6DoF，閉迴路控制用
- 三者互補，不衝突

---

### 12.1 YOLO 偵測目標定義

> 重點：不要追蹤整支手臂，先追蹤末端。

**末端執行器：HITBOT eHand-6 靈巧手**
- 6 指 DOF，CAN FD 控制（0x11=右手, 0x12=左手）
- 外觀特徵明顯（多指、黑色/金屬質感），比普通夾爪更容易被 YOLO 學到
- URDF: `ehand/灵巧手URDF 20250904/eHand-6-L/` 及 `eHand-6-R/`

**偵測類別（建議）：**

```yaml
names:
  0: can                # 罐子（計數用）
  1: ehand              # eHand-6 靈巧手整體
  2: openarm_wrist      # 手腕關節區域（J6-J7 附近）
  3: fingertip          # （進階）個別指尖追蹤
```

> 注意：先只做 class 0 + 1 就好，fingertip 是進階目標。

**為什麼靈巧手比夾爪更適合 YOLO 追蹤：**
- 外觀獨特 — 多指結構在畫面中辨識度高，不容易跟背景或 link 混淆
- 形狀變化豐富 — 不同手勢（張開/握拳/捏取）提供天然的資料多樣性
- 尺寸較大 — 比夾爪佔更多 pixel，偵測更穩定

**為什麼只追末端不追整支手臂：**
- 機械臂 link 外觀重複（都是黑色/銀色管狀），YOLO 難區分
- 自遮擋嚴重（某些姿態下 link 互擋）
- ByteTrack 適合追蹤明確獨立的目標，不適合拆分多個相似連桿
- 靈巧手外觀最獨特，最容易學

**素材準備：**
- 拍 eHand-6 在不同手勢下的照片 10-20 張
- 多角度（俯視、側面、正面）
- 不同手勢狀態（張開、握拳、捏取、指向等）
- 建議也拍「手持罐子」的照片 — 讓模型學會遮擋情境
- SAM 切割 → 合成資料 → 訓練（同 Section 11 流程）

---

### 12.2 RealSense Depth 反投影 3D

偵測到末端的 2D pixel 後，用 depth 算出 3D 座標：

```python
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

# === RealSense 初始化 ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# depth 對齊到 color
align = rs.align(rs.stream.color)

# 取得相機內參
profile = pipeline.get_active_profile()
color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# === YOLO 載入 ===
model = YOLO("best.pt")

def pixel_to_3d(depth_frame, u, v, intrinsics):
    """將 pixel (u, v) + depth 轉成相機座標系 3D 點"""
    depth_m = depth_frame.get_distance(int(u), int(v))
    if depth_m == 0:
        return None
    # rs2_deproject_pixel_to_point
    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m)
    return point_3d  # [x, y, z] in meters, camera frame

# === 主迴圈 ===
while True:
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())

    # YOLO 偵測 + 追蹤
    results = model.track(color_image, persist=True, tracker="bytetrack.yaml")

    if results[0].boxes.id is not None:
        boxes = results[0].boxes.xyxy.cpu().numpy()
        ids = results[0].boxes.id.cpu().numpy().astype(int)
        classes = results[0].boxes.cls.cpu().numpy().astype(int)

        for box, obj_id, cls in zip(boxes, ids, classes):
            # bbox 中心
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2

            # 反投影 3D
            point_3d = pixel_to_3d(depth_frame, cx, cy, color_intrinsics)
            if point_3d:
                class_name = model.names[cls]
                print(f"[{class_name}] ID={obj_id} → "
                      f"Camera 3D: x={point_3d[0]:.3f} y={point_3d[1]:.3f} z={point_3d[2]:.3f} m")
```

**深度值穩定化技巧：**
- 不要只取 1 個 pixel 的 depth，取 bbox 中心周圍 5x5 區域的中位數
- 加時間濾波（取連續 3 幀的移動平均）
- RealSense 有內建 temporal/spatial filter 可啟用

```python
def get_stable_depth(depth_frame, u, v, kernel=5):
    """取 pixel 周圍 kernel x kernel 區域的中位數深度"""
    half = kernel // 2
    depths = []
    for dy in range(-half, half + 1):
        for dx in range(-half, half + 1):
            d = depth_frame.get_distance(int(u + dx), int(v + dy))
            if d > 0:
                depths.append(d)
    return np.median(depths) if depths else 0
```

---

### 12.3 AprilTag / ArUco 精確定位（方案 C）

在 OpenArm 末端貼 AprilTag，用 OpenCV 偵測 + pose estimation：

```bash
pip install opencv-contrib-python  # 包含 aruco module
pip install pupil-apriltags        # 或用 apriltag 套件
```

```python
import cv2
import numpy as np

# ArUco 偵測器
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# 相機內參矩陣（從 RealSense intrinsics 取得）
camera_matrix = np.array([
    [color_intrinsics.fx, 0, color_intrinsics.ppx],
    [0, color_intrinsics.fy, color_intrinsics.ppy],
    [0, 0, 1]
])
dist_coeffs = np.array(color_intrinsics.coeffs)

marker_size = 0.04  # marker 實際邊長 (米)

def detect_aruco_pose(frame):
    """偵測 ArUco marker 並回傳 6DoF pose"""
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return None

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, marker_size, camera_matrix, dist_coeffs
    )

    for i, marker_id in enumerate(ids.flatten()):
        tvec = tvecs[i][0]  # [x, y, z] 平移 (米)
        rvec = rvecs[i][0]  # 旋轉向量
        R, _ = cv2.Rodrigues(rvec)  # 轉成旋轉矩陣

        print(f"Marker #{marker_id}: pos=({tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}) m")

        # 畫出座標軸
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size * 0.5)

    return tvecs, rvecs
```

**AprilTag vs ArUco 選擇：**

| | AprilTag | ArUco |
|---|---|---|
| 精度 | 稍高 | 稍低 |
| 速度 | 稍慢 | 快 |
| OpenCV 內建 | 否（需額外套件） | 是 |
| 推薦 | 精度優先 | 快速落地 |

對 OpenArm 來說，ArUco 就夠了。

---

### 12.4 座標轉換：相機 → Robot Base Frame

> 這是最容易被忽略、但最關鍵的一步。

沒有外參標定，你只能得到「相機座標系」下的位置，無法跟 ROS TF / Unity 世界座標對齊。

#### 外參標定方法

**方法 1：手動量測（快速但粗略）**

如果相機固定在已知位置（如機械臂旁邊的固定支架），直接量測相機相對於 robot base 的位移和旋轉。

```python
import numpy as np

# 手動量測的外參（範例）
# 相機在 robot base 座標系中的位置 (米)
t_camera_in_base = np.array([0.3, 0.0, 0.5])  # x, y, z

# 相機旋轉（看向下方，Z 軸朝下）- 根據實際安裝調整
R_camera_to_base = np.array([
    [1,  0,  0],
    [0, -1,  0],
    [0,  0, -1],
])

def camera_to_base(point_camera):
    """將相機座標系的 3D 點轉到 robot base 座標系"""
    return R_camera_to_base @ np.array(point_camera) + t_camera_in_base
```

**方法 2：用 ArUco 標定板做正式標定（推薦）**

1. 在 robot base 附近放一個已知位置的 ArUco 標定板
2. 用 RealSense 拍攝，算出相機到標定板的 transform
3. 標定板位置已知 → 得到 T_camera_base

**方法 3：ROS 2 手眼標定**

如果已經有 ROS 2 + MoveIt，可以用 `hand_eye_calibration` 套件自動標定。

---

### 12.5 完整 Pipeline：罐子計數 + 手臂追蹤

```python
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# --- 初始化 ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

profile = pipeline.get_active_profile()
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

model = YOLO("best.pt")  # 訓練好的模型（can + ehand + openarm_wrist）

# --- 計數狀態 ---
can_count = 0
counted_ids = set()
LINE_Y = 300

# --- 外參（需根據實際安裝標定） ---
R_cam2base = np.eye(3)       # TODO: 實際標定後填入
t_cam2base = np.zeros(3)     # TODO: 實際標定後填入

def pixel_to_base(depth_frame, u, v):
    """pixel → camera 3D → base frame 3D"""
    d = depth_frame.get_distance(int(u), int(v))
    if d == 0:
        return None
    p_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], d)
    p_base = R_cam2base @ np.array(p_cam) + t_cam2base
    return p_base

# --- 主迴圈 ---
while True:
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        continue

    frame = np.asanyarray(color_frame.get_data())
    results = model.track(frame, persist=True, tracker="bytetrack.yaml")

    if results[0].boxes.id is not None:
        boxes = results[0].boxes.xyxy.cpu().numpy()
        ids = results[0].boxes.id.cpu().numpy().astype(int)
        classes = results[0].boxes.cls.cpu().numpy().astype(int)

        for box, obj_id, cls in zip(boxes, ids, classes):
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2
            class_name = model.names[cls]

            if class_name == "can":
                # 罐子越線計數
                if cy > LINE_Y and obj_id not in counted_ids:
                    can_count += 1
                    counted_ids.add(obj_id)

            elif class_name in ("ehand", "openarm_wrist"):
                # 手臂末端 3D 位置
                p = pixel_to_base(depth_frame, cx, cy)
                if p is not None:
                    print(f"[{class_name}] base frame: "
                          f"x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")

    # 畫面顯示
    cv2.line(frame, (0, LINE_Y), (640, LINE_Y), (0, 0, 255), 2)
    cv2.putText(frame, f"Cans: {can_count}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    annotated = results[0].plot()
    cv2.imshow("OpenArm + Can Tracker", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()
```

---

### 12.6 RealSense D435i 注意事項

| 項目 | D435i 規格 |
|------|-----------|
| 最小深度距離 | ~28 cm |
| 理想範圍 | 0.3 ~ 3 m |
| Depth 解析度 | 最高 1280x720 |
| RGB 解析度 | 最高 1920x1080 |
| 幀率 | 30fps (640x480) / 15fps (1280x720) |
| IMU | 有（加速度計 + 陀螺儀） |

- 如果相機離手臂 < 28cm，depth 會無效 → 需調整安裝距離
- 建議安裝距離：30-80cm（桌面型機械臂場景）
- 啟用 RealSense post-processing filters 可改善 depth 品質

```python
# RealSense depth 後處理濾波器
decimation = rs.decimation_filter()
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()

def filter_depth(depth_frame):
    depth_frame = decimation.process(depth_frame)
    depth_frame = spatial.process(depth_frame)
    depth_frame = temporal.process(depth_frame)
    depth_frame = hole_filling.process(depth_frame)
    return depth_frame
```

---

### 12.7 開發順序建議

```
Phase 1 (1-2 天)
├── 拍罐子 + 夾爪照片
├── SAM 去背
├── 合成資料 500 張（can + ehand）
└── 訓練 YOLOv11n → 驗證偵測效果

Phase 2 (1 天)
├── RealSense RGB + YOLO + ByteTrack 即時追蹤
├── 罐子越線計數功能
└── 驗證追蹤穩定性

Phase 3 (1-2 天)
├── 加入 depth → 3D 反投影
├── 手臂末端 3D 座標輸出
├── （選配）ArUco marker 精確定位
└── 外參標定 camera → base

Phase 4 (進階)
├── ROS 2 整合（publish TF / topic）
├── Unity 整合（透過 ROS-TCP-Connector）
└── 閉迴路控制回饋
```
