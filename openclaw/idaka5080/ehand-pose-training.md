# eHand-6 靈巧手 YOLO Pose 訓練指南

> 目標：用 YOLOv11 Pose Estimation 追蹤 eHand-6 靈巧手的關鍵點（指尖、關節等），實現即時手指姿態追蹤。
> 最後更新：2026-03-19

---

## 可行性評估

**結論：可行。有兩條路線，URDF 合成資料路線可大幅減少人工標註。**

| 項目 | 評估 |
|------|------|
| YOLOv11 支援自訂關鍵點 | YES — 官方已有 Dog-Pose (24點)、Hand (21點) 等自訂範例 |
| 預訓練模型能否直接辨識 eHand | **NO** — COCO 沒有機械手類別，pose 只有人體 17 點 |
| YOLO-World 開放詞彙偵測 | 可能框出 bbox，但**無法輸出關鍵點**（只是 detect） |
| URDF 合成資料可行性 | **YES** — 有完整 URDF + STL mesh，可用 PyBullet 渲染 + 自動標註 |
| 訓練難度 | 低 — 跟 YOLO detect 訓練流程幾乎一樣，只是資料格式不同 |
| 即時性 | 好 — yolo11n-pose 在 RTX 5080 上可達 100+ FPS |

### 為什麼需要自訂訓練（預訓練不可用）

| 預訓練模型 | 能辨識 eHand-6 嗎 | 原因 |
|---|---|---|
| yolo11-detect (COCO 80 類) | 不行 | COCO 沒有「機械手」「靈巧手」「gripper」等類別 |
| yolo11-pose (COCO 17 keypoints) | 不行 | 只有人體關鍵點，不認識機械手結構 |
| YOLO-World (開放詞彙偵測) | **bbox 可能** | 可用 "robot hand" 文字提示框出整體，但**沒有關鍵點** |

→ 要做到指尖級別的關鍵點追蹤，**必須自訂訓練**。

### 為什麼用 Pose 而非 Detect

- **Detect**：只輸出 ehand 的 bounding box → 知道「手在哪」
- **Pose**：輸出每個指尖/關節的精確座標 → 知道「每根手指在哪」
- Pose 對靈巧手控制回饋、手勢辨識、抓取狀態判斷都更有用

### 兩條資料路線比較

| | 路線 A：手動標註 | 路線 B：URDF 合成資料 (推薦) |
|---|---|---|
| **資料來源** | 真實影片擷取幀 | PyBullet 渲染 URDF 3D 模型 |
| **標註方式** | 人工用 CVAT/Roboflow | 自動（3D 關節投影到 2D） |
| **人力成本** | 高（200+ 張 x 6 個關鍵點） | **極低**（寫一次腳本，生成無限量） |
| **姿態多樣性** | 受限於影片內容 | **無限**（隨機關節角度） |
| **真實感** | 高 | 低（需 domain randomization 補強） |
| **建議** | 用於 fine-tune / 驗證 | **主要訓練資料來源** |

---

## 1. URDF 結構分析

### 檔案位置

```
ehand/灵巧手URDF 20250904/
├── eHand-6-L/          ← 左手
│   ├── meshes/*.STL    ← 23 個 STL mesh 檔
│   └── urdf/eHand-6-L.urdf
└── eHand-6-R/          ← 右手
    ├── meshes/*.STL
    └── urdf/eHand-6-R.urdf
```

> URDF 由 SolidWorks URDF Exporter 自動產生。

### 手指結構（5 指，共 16 個可動關節）

```
base_link (手掌底座)
│
├── joint_1 (fixed) → link_1 (拇指底座)
│   └── joint_1_1 (revolute) → link_1_1
│       └── joint_2_1 (revolute, rpy=1.57 0.16 0.70) → link_2_1 (拇指第1指節)
│           └── joint_2_2 (revolute, xyz=0.062) → link_2_2 (拇指第2指節)
│               └── joint_2_3 (revolute, xyz=0.019) → link_2_3 ← 拇指指尖
│
├── joint_3 (fixed, rpy=-1.606) → link_3 (食指底座)
│   └── joint_3_1 (revolute) → link_3_1 (食指第1指節)
│       └── joint_3_2 (revolute, xyz=-0.028) → link_3_2 (食指第2指節)
│           └── joint_3_3 (revolute, xyz=-0.021) → link_3_3 ← 食指指尖
│
├── joint_4 (fixed, rpy=-1.571) → link_4 (中指底座)
│   └── joint_4_1 (continuous) → link_4_1 (中指第1指節)
│       └── joint_4_2 (revolute, xyz=-0.028) → link_4_2 (中指第2指節)
│           └── joint_4_3 (revolute, xyz=-0.021) → link_4_3 ← 中指指尖
│
├── joint_5 (fixed, rpy=-1.518) → link_5 (無名指底座)
│   └── joint_5_1 (revolute) → link_5_1 (無名指第1指節)
│       └── joint_5_2 (revolute, xyz=-0.028) → link_5_2 (無名指第2指節)
│           └── joint_5_3 (revolute, xyz=-0.021) → link_5_3 ← 無名指指尖
│
└── joint_6 (fixed, rpy=-1.466) → link_6 (小指底座)
    └── joint_6_1 (revolute) → link_6_1 (小指第1指節)
        └── joint_6_2 (revolute, xyz=-0.028) → link_6_2 (小指第2指節)
            └── joint_6_3 (revolute, xyz=-0.021) → link_6_3 ← 小指指尖
```

> 注意：URDF 中所有 joint limit 都是 `lower=0 upper=0`（SolidWorks 匯出的通病），
> 需要手動定義合理的關節限位。

### 可動關節清單（16 個）

| 關節 | 類型 | 所屬手指 | 功能 |
|------|------|---------|------|
| joint_1_1 | revolute | 拇指 | 底座旋轉 |
| joint_2_1 | revolute | 拇指 | MCP 關節 |
| joint_2_2 | revolute | 拇指 | IP 關節 |
| joint_2_3 | revolute | 拇指 | 指尖 |
| joint_3_1 | revolute | 食指 | MCP 關節 |
| joint_3_2 | revolute | 食指 | PIP 關節 |
| joint_3_3 | revolute | 食指 | DIP 關節 |
| joint_4_1 | continuous | 中指 | MCP 關節 |
| joint_4_2 | revolute | 中指 | PIP 關節 |
| joint_4_3 | revolute | 中指 | DIP 關節 |
| joint_5_1 | revolute | 無名指 | MCP 關節 |
| joint_5_2 | revolute | 無名指 | PIP 關節 |
| joint_5_3 | revolute | 無名指 | DIP 關節 |
| joint_6_1 | revolute | 小指 | MCP 關節 |
| joint_6_2 | revolute | 小指 | PIP 關節 |
| joint_6_3 | revolute | 小指 | DIP 關節 |

---

## 2. 關鍵點定義（基於 URDF）

### 6 點方案（建議先用）

直接對應 URDF 的末端 link，用 `getLinkState()` 自動取得 3D 座標：

```
關鍵點 → URDF link              → 說明
─────────────────────────────────────────
0: palm_center  → base_link     — 手掌中心
1: thumb_tip    → link_2_3      — 拇指指尖
2: index_tip    → link_3_3      — 食指指尖
3: middle_tip   → link_4_3      — 中指指尖
4: ring_tip     → link_5_3      — 無名指指尖
5: pinky_tip    → link_6_3      — 小指指尖
```

### 16 點完整方案（進階）

加入所有中間關節，提供更完整的手指姿態描述：

```
關鍵點 → URDF link              → 說明
─────────────────────────────────────────
 0: palm_center   → base_link   — 手掌中心
 1: thumb_base    → link_1_1    — 拇指根部
 2: thumb_mcp     → link_2_1    — 拇指 MCP
 3: thumb_ip      → link_2_2    — 拇指 IP
 4: thumb_tip     → link_2_3    — 拇指指尖
 5: index_mcp     → link_3_1    — 食指 MCP
 6: index_pip     → link_3_2    — 食指 PIP
 7: index_tip     → link_3_3    — 食指指尖
 8: middle_mcp    → link_4_1    — 中指 MCP
 9: middle_pip    → link_4_2    — 中指 PIP
10: middle_tip    → link_4_3    — 中指指尖
11: ring_mcp      → link_5_1    — 無名指 MCP
12: ring_pip      → link_5_2    — 無名指 PIP
13: ring_tip      → link_5_3    — 無名指指尖
14: pinky_mcp     → link_6_1    — 小指 MCP
15: pinky_pip     → link_6_2    — 小指 PIP
16: pinky_tip     → link_6_3    — 小指指尖
```

---

## 2. 素材來源

### 現有影片

```
idaka5080/video/hand/PXL_20260319_073613617.mp4  (~632 MB)
```

### 從影片擷取訓練幀

```bash
# 在 5080 上執行（或本機有 ffmpeg 也可以）
conda activate yolo

# 每 0.5 秒擷取一幀（假設 30fps 影片 = 每 15 幀取 1 幀）
ffmpeg -i PXL_20260319_073613617.mp4 -vf "fps=2" -q:v 2 frames/frame_%05d.jpg

# 或每 1 秒擷取一幀（資料量較少，但可能不夠多樣）
ffmpeg -i PXL_20260319_073613617.mp4 -vf "fps=1" -q:v 2 frames/frame_%05d.jpg
```

> 擷取後手動篩選，保留 200-500 張有代表性的幀（不同手勢、角度、遮擋狀態）。
> 刪除重複度高、模糊、手不在畫面中的幀。

### 建議補拍（提升泛化能力）

- 不同光線條件（日光、燈光、陰影）
- 不同背景（桌面、手臂上、空中）
- 不同手勢狀態（全開、半握、握拳、捏取、指向）
- 手持物體（罐子、工具）的遮擋情境
- 不同距離（近拍、中距離、遠距離）

---

## 3. 標註工具與格式

### YOLO Pose 標註格式

每張圖片對應一個 `.txt` 檔，每行格式：

```
<class> <x> <y> <w> <h> <px1> <py1> <v1> <px2> <py2> <v2> ... <pxN> <pyN> <vN>
```

| 欄位 | 說明 |
|------|------|
| `class` | 類別索引（0 = ehand） |
| `x y w h` | 歸一化 bounding box（中心 x, y, 寬, 高） |
| `pxN pyN` | 第 N 個關鍵點的歸一化座標 (0~1) |
| `vN` | 可見性：0=不可見, 1=被遮擋但位置已知, 2=可見 |

**7 點方案範例（一行）：**

```
0 0.45 0.52 0.30 0.40 0.43 0.50 2 0.35 0.38 2 0.40 0.35 2 0.45 0.33 2 0.50 0.35 2 0.55 0.38 2 0.48 0.42 1
```

### 推薦標註工具

| 工具 | 優點 | 缺點 |
|------|------|------|
| **[CVAT](https://app.cvat.ai)** | 免費、功能強大、支援 keypoint skeleton | 學習曲線稍高 |
| **[Roboflow](https://roboflow.com)** | 界面友善、可直接匯出 YOLO 格式、有 auto-annotate | 免費版有限制 |
| **[Label Studio](https://labelstud.io)** | 開源、高度可自訂 | 需自架 |
| **[Labellerr](https://labellerr.com)** | 直接支援 YOLO pose 格式 | 需付費 |

**推薦流程：用 CVAT 或 Roboflow 標註 → 匯出 YOLO 格式**

### CVAT 關鍵點標註設定

1. 建立新專案 → 定義 skeleton template
2. 設定 ehand skeleton：
   - 建立 7 個 keypoint（palm_center, finger1~6_tip）
   - 定義骨架連線（palm_center → 各 fingertip）
3. 上傳擷取的幀 → 逐張標註
4. 匯出格式選 YOLO → 手動調整為 pose 格式

---

## 4. 資料集結構

```
~/openarm_yolo_training/datasets/ehand_pose/
├── images/
│   ├── train/          ← 80% (約 160-400 張)
│   │   ├── frame_00001.jpg
│   │   └── ...
│   └── val/            ← 20% (約 40-100 張)
│       ├── frame_00201.jpg
│       └── ...
├── labels/
│   ├── train/
│   │   ├── frame_00001.txt   ← 每行: class x y w h px1 py1 v1 ... px7 py7 v7
│   │   └── ...
│   └── val/
│       ├── frame_00201.txt
│       └── ...
└── data.yaml
```

### data.yaml

```yaml
path: /home/idaka_5080/openarm_yolo_training/datasets/ehand_pose
train: images/train
val: images/val

# 類別
nc: 1
names:
  0: ehand

# 關鍵點設定 (7 點方案)
kpt_shape: [7, 3]    # [關鍵點數量, 維度] — 3 = (x, y, visibility)

# 左右翻轉時的對應索引（用於資料增強）
# palm_center 不翻轉(0→0)，finger1~6 根據 eHand 對稱性設定
# 如果 eHand 左右不對稱，設為自身索引（不翻轉）
flip_idx: [0, 1, 2, 3, 4, 5, 6]
```

> **`kpt_shape: [7, 3]`** 是關鍵設定：
> - 第一個數字 = 關鍵點數量（7 或 13）
> - 第二個數字 = 每個點的維度（3 = x, y, visibility）

---

## 5. 訓練

### 上傳資料到 5080

```bash
# 從 Windows
scp -r ehand_pose/ IDAKA_5080:~/openarm_yolo_training/datasets/
```

### 訓練指令

```bash
ssh IDAKA_5080
conda activate yolo
cd ~/openarm_yolo_training

# YOLOv11n-pose（最快，建議先跑這個驗證）
yolo pose train \
  model=yolo11n-pose.pt \
  data=datasets/ehand_pose/data.yaml \
  epochs=200 \
  imgsz=640 \
  batch=32 \
  workers=8 \
  device=0 \
  patience=50 \
  project=runs \
  name=ehand_pose_v1

# YOLOv11s-pose（精度更高）
yolo pose train \
  model=yolo11s-pose.pt \
  data=datasets/ehand_pose/data.yaml \
  epochs=200 \
  imgsz=640 \
  batch=16 \
  device=0 \
  patience=50 \
  project=runs \
  name=ehand_pose_v1_s
```

### Python 腳本方式

```python
from ultralytics import YOLO

model = YOLO("yolo11n-pose.pt")

results = model.train(
    data="/home/idaka_5080/openarm_yolo_training/datasets/ehand_pose/data.yaml",
    epochs=200,
    imgsz=640,
    batch=32,
    device=0,
    patience=50,
    project="/home/idaka_5080/openarm_yolo_training/runs",
    name="ehand_pose_v1",
)
```

### RTX 5080 Pose 模型建議參數

| 模型 | batch size | 預估 VRAM | 備註 |
|------|-----------|----------|------|
| yolo11n-pose | 32-64 | ~4-6 GB | 先用這個驗證 |
| yolo11s-pose | 16-32 | ~6-8 GB | 精度/速度平衡 |
| yolo11m-pose | 8-16 | ~10-12 GB | 精度更好 |
| yolo11l-pose | 4-8 | ~14-16 GB | 最高精度 |

---

## 6. 驗證與推論

```bash
# 驗證
yolo pose val \
  model=runs/ehand_pose_v1/weights/best.pt \
  data=datasets/ehand_pose/data.yaml

# 單張圖片推論
yolo pose predict \
  model=runs/ehand_pose_v1/weights/best.pt \
  source=test_image.jpg

# 影片推論
yolo pose predict \
  model=runs/ehand_pose_v1/weights/best.pt \
  source=video.mp4
```

### Python 推論

```python
from ultralytics import YOLO
import cv2

model = YOLO("runs/ehand_pose_v1/weights/best.pt")

# 影片推論
results = model.predict("video.mp4", stream=True)
for r in results:
    if r.keypoints is not None:
        # keypoints.xy: [N, num_kpts, 2] — pixel 座標
        # keypoints.conf: [N, num_kpts] — 每個關鍵點的信心度
        kpts = r.keypoints.xy[0].cpu().numpy()    # 第一個偵測到的手
        confs = r.keypoints.conf[0].cpu().numpy()

        palm = kpts[0]      # palm_center
        finger1 = kpts[1]   # finger1_tip
        # ...
        print(f"Palm: ({palm[0]:.0f}, {palm[1]:.0f})")
        for i in range(1, 7):
            print(f"  Finger {i}: ({kpts[i][0]:.0f}, {kpts[i][1]:.0f}) conf={confs[i]:.2f}")

    # 畫出結果
    annotated = r.plot()
    cv2.imshow("eHand Pose", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## 7. 結合 RealSense Depth 取得 3D 指尖座標

偵測到 2D 關鍵點後，用 depth 反投影取得各指尖的 3D 座標：

```python
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

# RealSense 初始化（同 Section 12.2）
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

profile = pipeline.get_active_profile()
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

model = YOLO("ehand_pose_best.pt")

KEYPOINT_NAMES = [
    "palm_center",
    "finger1_tip", "finger2_tip", "finger3_tip",
    "finger4_tip", "finger5_tip", "finger6_tip",
]

def get_3d_point(depth_frame, u, v, intrinsics, kernel=5):
    """pixel → 3D (中位數深度，更穩定)"""
    half = kernel // 2
    depths = []
    for dy in range(-half, half + 1):
        for dx in range(-half, half + 1):
            d = depth_frame.get_distance(int(u + dx), int(v + dy))
            if d > 0:
                depths.append(d)
    if not depths:
        return None
    depth_m = np.median(depths)
    return rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m)

while True:
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    results = model.predict(color_image)

    if results[0].keypoints is not None:
        kpts = results[0].keypoints.xy[0].cpu().numpy()
        confs = results[0].keypoints.conf[0].cpu().numpy()

        for i, (name, kpt, conf) in enumerate(zip(KEYPOINT_NAMES, kpts, confs)):
            if conf < 0.5:
                continue
            pt3d = get_3d_point(depth_frame, kpt[0], kpt[1], intrinsics)
            if pt3d:
                print(f"  {name}: 3D=({pt3d[0]:.3f}, {pt3d[1]:.3f}, {pt3d[2]:.3f}) m  conf={conf:.2f}")
```

---

## 8. URDF 合成資料方案（推薦路線）

> 核心概念：用 PyBullet 載入 URDF → 隨機關節角度 → 渲染圖片 → 3D 關節位置自動投影到 2D
> → **零人工標註**，可無限生成訓練資料。

### 為什麼 URDF 合成資料可行

1. **URDF 有完整的 STL mesh**（23 個零件），可以渲染出 eHand 的外觀
2. **URDF 定義了所有關節位置**，PyBullet 的 `getLinkState()` 直接給出 3D 座標
3. **3D→2D 投影是精確的**（已知相機矩陣），自動產生 YOLO pose 標註
4. **隨機關節角度 = 無限姿態多樣性**，比影片擷取幀更全面

### 整體流程

```
ehand/灵巧手URDF 20250904/eHand-6-R/urdf/eHand-6-R.urdf
                │
                ▼
        PyBullet 載入 URDF + STL meshes
                │
                ├── 隨機化關節角度 (16 個可動關節)
                ├── 隨機化相機視角 (yaw, pitch, distance)
                ├── 隨機化背景 (替換 or 合成)
                │
                ▼
        渲染 RGB 圖片 (640x480)
                │
                ├── getLinkState() → 各 link 的 3D 世界座標
                ├── 3D → 2D 投影 (相機內參矩陣)
                ├── 計算 bounding box (包圍所有可見關鍵點)
                │
                ▼
        自動輸出 YOLO Pose 標註
        (class x y w h px1 py1 v1 px2 py2 v2 ... px6 py6 v6)
```

### 安裝依賴

```bash
# 在 5080 上
conda activate yolo
pip install pybullet numpy opencv-python Pillow
```

### PyBullet 合成資料腳本架構

```python
import pybullet as p
import pybullet_data
import numpy as np
import cv2
import os
from pathlib import Path

# ============================================================
# eHand-6 URDF 合成資料生成器
# ============================================================

class EHandSyntheticGenerator:
    """用 PyBullet 載入 eHand URDF，渲染不同姿態並自動標註關鍵點。"""

    # URDF joint limit 都是 0 → 手動定義合理範圍 (rad)
    # 需要根據實際 eHand-6 規格調整
    JOINT_LIMITS = {
        'joint_1_1': (-0.5, 0.5),     # 拇指底座旋轉
        'joint_2_1': (-0.3, 1.2),     # 拇指 MCP
        'joint_2_2': (-0.2, 1.5),     # 拇指 IP
        'joint_2_3': (-0.2, 1.0),     # 拇指指尖
        'joint_3_1': (-0.2, 1.5),     # 食指 MCP
        'joint_3_2': (-0.2, 1.5),     # 食指 PIP
        'joint_3_3': (-0.2, 1.2),     # 食指 DIP
        'joint_4_1': (-0.2, 1.5),     # 中指 MCP
        'joint_4_2': (-0.2, 1.5),     # 中指 PIP
        'joint_4_3': (-0.2, 1.2),     # 中指 DIP
        'joint_5_1': (-0.2, 1.5),     # 無名指 MCP
        'joint_5_2': (-0.2, 1.5),     # 無名指 PIP
        'joint_5_3': (-0.2, 1.2),     # 無名指 DIP
        'joint_6_1': (-0.2, 1.5),     # 小指 MCP
        'joint_6_2': (-0.2, 1.5),     # 小指 PIP
        'joint_6_3': (-0.2, 1.2),     # 小指 DIP
    }

    # 關鍵點定義：6 點方案 (palm + 5 fingertips)
    # value = 對應 URDF 中的 link 名稱
    KEYPOINT_LINKS_6PT = {
        0: 'base_link',   # palm_center
        1: 'link_2_3',    # thumb_tip
        2: 'link_3_3',    # index_tip
        3: 'link_4_3',    # middle_tip
        4: 'link_5_3',    # ring_tip
        5: 'link_6_3',    # pinky_tip
    }

    def __init__(self, urdf_path):
        self.client = p.connect(p.DIRECT)  # 無頭渲染
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 載入 URDF
        self.body_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True,
        )

        # 建立 joint name → index 對照表
        self.joint_map = {}
        self.link_map = {}
        for i in range(p.getNumJoints(self.body_id)):
            info = p.getJointInfo(self.body_id, i)
            joint_name = info[1].decode('utf-8')
            link_name = info[12].decode('utf-8')
            self.joint_map[joint_name] = i
            self.link_map[link_name] = i

    def set_random_pose(self):
        """隨機設定所有可動關節角度。"""
        for joint_name, (lo, hi) in self.JOINT_LIMITS.items():
            if joint_name in self.joint_map:
                angle = np.random.uniform(lo, hi)
                joint_idx = self.joint_map[joint_name]
                p.resetJointState(self.body_id, joint_idx, angle)

    def get_keypoints_3d(self):
        """取得所有關鍵點的 3D 世界座標。"""
        points = {}
        for kp_idx, link_name in self.KEYPOINT_LINKS_6PT.items():
            if link_name == 'base_link':
                # base_link 用 getBasePositionAndOrientation
                pos, _ = p.getBasePositionAndOrientation(self.body_id)
            else:
                link_idx = self.link_map[link_name]
                state = p.getLinkState(self.body_id, link_idx)
                pos = state[0]  # worldLinkFramePosition
            points[kp_idx] = np.array(pos)
        return points

    def render(self, cam_distance, cam_yaw, cam_pitch, img_size=(640, 480)):
        """渲染一張圖片，回傳 RGB array + view/proj matrix。"""
        target = [0, 0, 0.07]  # 看向 base_link 附近

        view_mat = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=target,
            distance=cam_distance,
            yaw=cam_yaw,
            pitch=cam_pitch,
            roll=0,
            upAxisIndex=2,
        )
        proj_mat = p.computeProjectionMatrixFOV(
            fov=60, aspect=img_size[0] / img_size[1],
            nearVal=0.01, farVal=5.0,
        )

        _, _, rgba, _, _ = p.getCameraImage(
            width=img_size[0], height=img_size[1],
            viewMatrix=view_mat, projectionMatrix=proj_mat,
            renderer=p.ER_TINY_RENDERER,
        )

        rgb = np.array(rgba, dtype=np.uint8).reshape(
            img_size[1], img_size[0], 4)[:, :, :3]

        return rgb, view_mat, proj_mat

    def project_3d_to_2d(self, point_3d, view_mat, proj_mat, img_size):
        """3D 世界座標 → 2D pixel 座標。"""
        p4 = np.array([*point_3d, 1.0])

        V = np.array(view_mat).reshape(4, 4, order='F')
        P = np.array(proj_mat).reshape(4, 4, order='F')

        clip = P @ V @ p4
        if clip[3] == 0:
            return None
        ndc = clip[:3] / clip[3]

        px = int((ndc[0] + 1) * img_size[0] / 2)
        py = int((1 - ndc[1]) * img_size[1] / 2)

        if 0 <= px < img_size[0] and 0 <= py < img_size[1]:
            return (px, py)
        return None

    def generate_dataset(self, output_dir, num_train=800, num_val=100):
        """生成完整的 YOLO pose 資料集。"""
        for split, count in [('train', num_train), ('val', num_val)]:
            img_dir = Path(output_dir) / 'images' / split
            lbl_dir = Path(output_dir) / 'labels' / split
            img_dir.mkdir(parents=True, exist_ok=True)
            lbl_dir.mkdir(parents=True, exist_ok=True)

            for i in range(count):
                # 隨機姿態
                self.set_random_pose()

                # 隨機相機
                dist = np.random.uniform(0.25, 0.60)
                yaw = np.random.uniform(0, 360)
                pitch = np.random.uniform(-60, 30)
                img_size = (640, 480)

                rgb, view_mat, proj_mat = self.render(
                    dist, yaw, pitch, img_size)

                # 取得 2D 關鍵點
                kp3d = self.get_keypoints_3d()
                keypoints = []
                visible_pixels = []

                for kp_idx in sorted(kp3d.keys()):
                    px = self.project_3d_to_2d(
                        kp3d[kp_idx], view_mat, proj_mat, img_size)
                    if px:
                        x_norm = px[0] / img_size[0]
                        y_norm = px[1] / img_size[1]
                        keypoints.extend([x_norm, y_norm, 2])  # 2=可見
                        visible_pixels.append(px)
                    else:
                        keypoints.extend([0, 0, 0])  # 0=不可見

                if len(visible_pixels) < 3:
                    continue  # 太少關鍵點可見，跳過

                # 從可見關鍵點計算 bounding box
                xs = [p[0] for p in visible_pixels]
                ys = [p[1] for p in visible_pixels]
                margin = 20  # pixel
                x1 = max(0, min(xs) - margin)
                y1 = max(0, min(ys) - margin)
                x2 = min(img_size[0], max(xs) + margin)
                y2 = min(img_size[1], max(ys) + margin)

                cx = ((x1 + x2) / 2) / img_size[0]
                cy = ((y1 + y2) / 2) / img_size[1]
                bw = (x2 - x1) / img_size[0]
                bh = (y2 - y1) / img_size[1]

                # 儲存圖片
                fname = f"syn_{split}_{i:05d}"
                cv2.imwrite(
                    str(img_dir / f"{fname}.jpg"),
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

                # 儲存標註
                kp_str = " ".join(f"{v:.6f}" for v in keypoints)
                label = f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f} {kp_str}"
                (lbl_dir / f"{fname}.txt").write_text(label)

            print(f"  {split}: {count} images generated")

        # 產生 data.yaml
        num_kpts = len(self.KEYPOINT_LINKS_6PT)
        yaml_content = f"""path: {os.path.abspath(output_dir)}
train: images/train
val: images/val

nc: 1
names:
  0: ehand

kpt_shape: [{num_kpts}, 3]
flip_idx: {list(range(num_kpts))}
"""
        (Path(output_dir) / 'data.yaml').write_text(yaml_content)
        print(f"Done! Dataset at {output_dir}")


# ============================================================
# 使用範例
# ============================================================
if __name__ == '__main__':
    gen = EHandSyntheticGenerator(
        urdf_path='ehand/灵巧手URDF 20250904/eHand-6-R/urdf/eHand-6-R.urdf'
    )
    gen.generate_dataset(
        output_dir='datasets/ehand_pose_synthetic',
        num_train=800,
        num_val=100,
    )
```

### URDF 注意事項

**1. Joint Limits 需要手動定義**

URDF 中所有 limit 都是 `lower=0 upper=0`（SolidWorks 匯出問題）。
腳本中的 `JOINT_LIMITS` 是估計值，需要根據 eHand-6 實際規格調整：
- 查看 HITBOT eHand-6 技術手冊中的關節活動範圍
- 或在 PyBullet GUI 模式下手動測試合理角度

```python
# 用 GUI 模式互動測試
client = p.connect(p.GUI)
p.loadURDF("eHand-6-R.urdf", useFixedBase=True)
# → 用滑桿調整各關節，記錄合理範圍
```

**2. STL mesh 路徑**

URDF 中用 `package://eHand-6-R/meshes/xxx.STL` 格式。
PyBullet 載入時需要設定正確的搜尋路徑：

```python
# 方法 A: 設定 package 搜尋路徑
import pybullet_data
p.setAdditionalSearchPath("ehand/灵巧手URDF 20250904/eHand-6-R/")

# 方法 B: 修改 URDF 中的路徑為相對路徑
# 把 package://eHand-6-R/meshes/ 改成 meshes/
```

**3. 渲染品質 — Sim-to-Real Gap**

PyBullet TinyRenderer 渲染品質有限（CAD 外觀，無光影）。改善方法：

| 方法 | 效果 | 複雜度 |
|------|------|--------|
| **Domain Randomization** | 隨機背景、亮度、對比度、噪點 | 低 |
| **混合真實圖片 fine-tune** | 用少量真實標註圖微調 | 中 |
| **pyrender 取代 TinyRenderer** | OpenGL 渲染，品質更好 | 中 |
| **BlenderProc** | 光線追蹤，接近照片品質 | 高（且不直接支援 URDF） |

**建議策略：PyBullet 合成 800 張 + Domain Randomization → 訓練初版 → 用少量真實圖片 fine-tune**

### Domain Randomization 增強

```python
import albumentations as A

# 在儲存圖片前套用 augmentation
augment = A.Compose([
    A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.8),
    A.GaussNoise(var_limit=(10, 50), p=0.5),
    A.GaussianBlur(blur_limit=(3, 7), p=0.3),
    A.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=30, p=0.5),
    A.RandomGamma(gamma_limit=(70, 130), p=0.3),
])

# 套用
augmented = augment(image=rgb)['image']
```

背景替換（用真實場景背景）：

```python
def replace_background(rendered_img, mask, bg_img):
    """把 PyBullet 的灰色背景換成真實背景。"""
    # PyBullet 預設背景是灰色 (198, 198, 198) 左右
    bg_resized = cv2.resize(bg_img, (rendered_img.shape[1], rendered_img.shape[0]))
    # 用 mask（alpha channel 或色差偵測）合成
    result = np.where(mask[:, :, None], rendered_img, bg_resized)
    return result
```

---

## 8.5 半自動標註真實圖片（補充路線 A）

如果合成資料效果不夠好，可以用少量真實圖片 fine-tune：

### 方法 A：先訓練粗略模型 → 自動預標註 → 人工修正

```
1. 用 URDF 合成資料訓練初版 yolo11n-pose
2. 用初版模型推論真實影片擷取的幀 → 輸出預標註
3. 載入預標註到 CVAT → 人工修正（比從零標快 3-5 倍）
4. 合併合成 + 真實標註 → 訓練最終版
```

### 方法 B：MediaPipe 輔助排除人手干擾

如果影片中有人手在操作 eHand，可以先用 MediaPipe Hand Landmark 偵測人手，
再只標註 eHand 的關鍵點。

---

## 9. 與現有系統整合

### 用途 1：靈巧手控制回饋

追蹤指尖 3D 位置 → 跟 CAN FD 控制指令比對 → 驗證手指是否到達目標位置

### 用途 2：抓取狀態偵測

- 指尖距離收斂 → 正在抓取
- 指尖距離發散 → 正在放開
- 特定指尖組合 → 辨識手勢（捏、握、指）

### 用途 3：VR 遙控回饋

Unity 中顯示 eHand 指尖追蹤結果 → 操作者可以看到真實手指位置 vs VR 指令的偏差

---

## 10. 5080 工作目錄結構

所有 eHand 前處理和訓練資料統一放在 `~/openarm_yolo_training/hand/`：

```
~/openarm_yolo_training/hand/
├── source_video/       ← 原始影片
│   └── ehand_video.mp4 (從 PXL_20260319_073613617.mp4 上傳)
├── source_frames/      ← 從影片截取的真實幀（用於驗證 + fine-tune）
├── rendered/           ← PyBullet/pyrender 渲染的合成圖 + 自動關鍵點標註
│   ├── images/
│   └── labels/
├── cutouts/            ← SAM 去背結果（如果走 SAM 路線）
├── backgrounds/        ← 真實環境背景圖（用於合成時背景替換）
├── dataset_pose/       ← 最終 YOLO Pose 格式資料集
│   ├── images/{train,val}/
│   ├── labels/{train,val}/
│   └── data.yaml
└── scripts/            ← 渲染、合成、訓練腳本
```

---

## 11. Sim-to-Real 迭代工作流

> 核心：用 Mesh 渲染訓練 → 真實影片驗證 → 不通過就回上一步調參 → 通過後自動標註真實幀

### 完整流程圖

```
┌──────────────────────────────────────────────────────────────┐
│  Step 1: Mesh 渲染 (5080 上執行)                              │
│                                                              │
│  URDF + STL mesh → PyBullet/pyrender 渲染                    │
│  ├── 隨機關節角度 (16 個可動關節)                             │
│  ├── 隨機相機視角 (yaw, pitch, distance)                     │
│  ├── 色調對齊真實影片 (金屬灰+黑色塑膠)                      │
│  ├── Domain Randomization (背景、亮度、噪點)                 │
│  └── 自動輸出 YOLO Pose 標註 (3D關節 → 2D投影)              │
│                                                              │
│  → rendered/ (800-1000 張 + 自動關鍵點標註)                   │
└──────────────────────┬───────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────┐
│  Step 2: 訓練 yolo11n-pose (5080, ~10 分鐘)                  │
│                                                              │
│  → ~/openarm_yolo_training/runs/ehand_pose_vN/weights/best.pt│
└──────────────────────┬───────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────┐
│  Step 3: 用真實影片影格驗證                                    │
│                                                              │
│  模型推論 source_frames/ 中的真實幀                           │
│  檢查：bbox 正確？關鍵點位置合理？                             │
│                                                              │
│  ┌─────────┐     ┌──────────────────────────────────────┐    │
│  │ 通過 ✓  │──→  │ Step 4: 自動標註真實幀                │    │
│  └─────────┘     │ 用模型推論影片幀 → 輸出預標註         │    │
│                  │ → 人工快速修正（比從零標快 3-5 倍）    │    │
│  ┌─────────┐     │ → 合成 + 真實混合重新訓練 → 最終版    │    │
│  │ 不通過 ✗│     └──────────────────────────────────────┘    │
│  └────┬────┘                                                 │
│       │                                                      │
│       ▼ 回 Step 1 調整：                                      │
│       ├── 色調不對 → 調整材質顏色 / 換渲染引擎               │
│       ├── 姿態不對 → 校正 JOINT_LIMITS                       │
│       ├── 背景差異大 → 加入真實背景圖到 backgrounds/          │
│       └── 偵測不到 → 增加 domain randomization 強度          │
└──────────────────────────────────────────────────────────────┘
```

### 色調對齊策略

eHand-6 真實外觀（從影片觀察 2026-03-19）：

| 部位 | 外觀 | 渲染設定 |
|------|------|---------|
| 手掌主體 | 銀灰色金屬，有反光 | RGB ~(160, 165, 170), metallic |
| 手指關節 | 深灰/暗銀金屬 | RGB ~(100, 105, 110) |
| 指尖 | 黑色光滑塑膠 | RGB ~(30, 30, 35), glossy |
| 手背蓋板 | 黑色霧面 | RGB ~(40, 40, 45), matte |
| 螺絲/鉸鏈 | 亮銀色 | RGB ~(200, 200, 205) |
| LED 指示燈 | 藍色小點 | 可忽略 |

### 渲染引擎選擇

| 引擎 | 色調對齊能力 | 品質 | 建議 |
|------|------------|------|------|
| **PyBullet TinyRenderer** | 低（只能設 RGBA） | 低 | **先試，配合 domain randomization** |
| **pyrender + OpenGL** | 中（支援 PBR 材質） | 中高 | Step 1 不通過時升級 |
| **BlenderProc** | 高（光線追蹤） | 最高 | 最後手段 |

### 預訓練偵測測試（2026-03-19 已完成）

| 模型 | 能偵測 eHand 嗎 | 說明 |
|------|----------------|------|
| YOLOv11n (COCO 80類) | **不行** | 誤認為 motorcycle、airplane 等 |
| YOLO-World (開放詞彙) | **幾乎不行** | 僅 1/6 幀偵測到 "mechanical hand" (conf=0.18) |

→ 確認**必須自訂訓練**，預訓練模型無法辨識 eHand-6。

測試結果位置：`~/openarm_yolo_training/test/hand/`

---

## 12. 開發步驟（更新版）

```
Phase 1: 環境準備 + 渲染測試
├── 5080 安裝 pybullet: pip install pybullet
├── 上傳 URDF + STL 到 5080
├── PyBullet GUI 測試載入 eHand-6 → 確認 mesh 正確
├── 測試關節角度範圍 → 校正 JOINT_LIMITS
├── 渲染一張 → 比對真實影片色調 → 調整材質顏色
└── 輸出: 渲染品質確認 OK

Phase 2: 合成資料生成
├── 隨機姿態 + 隨機視角渲染 800-1000 張
├── 3D 關鍵點 → 2D 投影 → 自動標註
├── Domain Randomization (背景替換 + 色彩增強)
├── 目視檢查：渲染圖 + 關鍵點投影是否正確
└── 輸出: hand/rendered/ + hand/dataset_pose/

Phase 3: 訓練 + 驗證（迭代）
├── yolo11n-pose 訓練 (~10 分鐘)
├── 用 source_frames/ 真實幀驗證
├── 不通過 → 回 Phase 1/2 調參
└── 通過 → 進入 Phase 4

Phase 4: 自動標註 + Fine-tune
├── 用 Phase 3 模型推論影片幀 → 輸出預標註
├── CVAT/Roboflow 人工修正（比從零標快 3-5 倍）
├── 合成 + 真實標註混合重新訓練
└── 輸出: 最終版 ehand_pose_best.pt

Phase 5: 部署 + 整合
├── Jetson TensorRT 匯出（需指定 task=pose）
├── 整合到 realsense_yolo_viewer.py
├── 結合 RealSense depth → 3D 指尖座標
└── ROS 2 / Unity 整合
```

---

## 故障排除

| 問題 | 解法 |
|------|------|
| PyBullet 載入 URDF 失敗 | 檢查 STL mesh 路徑，修改 `package://` 為相對路徑 |
| 渲染色調跟真實差太多 | 升級到 pyrender / 加強 domain randomization |
| 關鍵點偏移很大 | 增加標註數量 / 確認標註品質 / 加大 imgsz (640→1280) |
| 某些手勢偵測不到 | 校正 JOINT_LIMITS / 增加旋轉增強範圍 |
| 訓練 loss 不下降 | 檢查 data.yaml 的 `kpt_shape` 是否正確 / 確認標註格式 |
| 推論時關鍵點閃爍 | 加時間平滑（EMA）/ 用 `persist=True` 追蹤 |
| TensorRT 匯出後 task 不對 | 載入時明確指定 `YOLO(path, task="pose")` |
| ByteTrack + Pose 崩潰 | 改用 `botsort.yaml` + 加 try/except fallback |

---

## 影片素材資訊

| 項目 | 數值 |
|------|------|
| 檔案 | `idaka5080/video/hand/PXL_20260319_073613617.mp4` |
| 大小 | ~604 MB |
| 解析度 | 2160x3840（直式 4K） |
| FPS | 30 |
| 總幀數 | 3110 |
| 時長 | 103.7 秒 |
| 拍攝日期 | 2026-03-19 |
| 5080 上傳位置 | `~/openarm_yolo_training/hand/source_video/ehand_video.mp4` |
| 內容 | eHand-6 裝在 OpenArm 上，多角度多手勢（張開、握拳、側面等） |
| 用途 | 擷取幀 → 驗證模型 → 自動預標註 → fine-tune |
