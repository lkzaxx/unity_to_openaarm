# OpenArm YOLO Models

訓練於 idaka-5080 (RTX 5080 16GB, `192.168.0.245`)，
原始訓練目錄：`~/openarm_yolo_training/runs/`

## Models

### Detection（物件偵測）

| 檔案 | 版本 | 基礎模型 | 類別 | Epochs | mAP50 | 大小 |
|------|------|---------|------|--------|-------|------|
| `can_detect_v4_best.pt` | can_v4 | yolo11n | can | 200 | 0.979 | 5.3M |
| `can_detect_v3_best.pt` | can_v3 | yolo11n | can | 112 (早停) | 0.995 | 5.3M |
| `multiclass_detect_v3_best.pt` | multiclass_detect_v3 | yolo11n (從 v2 微調) | can, ehand | 21 (早停) | 0.966 | 5.3M |

### Pose（關鍵點追蹤）

| 檔案 | 版本 | 基礎模型 | 類別 | Keypoints | Epochs | Pose mAP50 | 大小 |
|------|------|---------|------|-----------|--------|-----------|------|
| `ehand_pose_sam_v4_best.pt` | ehand_16kpt_sam_v4 | yolo11n-pose | can, ehand | 16 kpt | 200 | 0.995 | 5.8M |
| `ehand_pose_sam_v3_best.pt` | ehand_16kpt_sam_v3 | yolo11n-pose | can, ehand | 16 kpt | 200 | 0.885 | 5.8M |

## 16 Keypoint 定義（eHand-6 靈巧手）

Pose 模型輸出 16 個關鍵點，對應 eHand-6 URDF 的關節與指尖：

```
Index  名稱             URDF link    說明
─────────────────────────────────────────────
 0     palm_center      base_link    手掌中心
 1     thumb_base       link_1_1     拇指根部
 2     thumb_mcp        link_2_1     拇指 MCP
 3     thumb_ip         link_2_2     拇指 IP
 4     thumb_tip        link_2_3     拇指指尖
 5     index_mcp        link_3_1     食指 MCP
 6     index_pip        link_3_2     食指 PIP
 7     index_tip        link_3_3     食指指尖
 8     middle_mcp       link_4_1     中指 MCP
 9     middle_pip       link_4_2     中指 PIP
10     middle_tip       link_4_3     中指指尖
11     ring_mcp         link_5_1     無名指 MCP
12     ring_pip         link_5_2     無名指 PIP
13     ring_tip         link_5_3     無名指指尖
14     pinky_mcp        link_6_1     小指 MCP
15     pinky_pip        link_6_2     小指 PIP
```

> kpt_shape: [16, 3]（x, y, visibility）
> 資料來源：URDF 合成（PyBullet 渲染 + 3D 關節自動投影到 2D）+ SAM 去背合成

```
手掌結構示意：

        4(thumb_tip)
        │
        3(thumb_ip)
        │
        2(thumb_mcp)     7(index)  10(middle)  13(ring)  15(pinky)
        │                │         │           │         │
        1(thumb_base)    6         9           12        14
                         │         │           │         │
                         5(mcp)    8(mcp)      11(mcp)   pinky_mcp
                         └─────────┴───────────┴─────────┘
                                   0(palm_center)
```

## 推薦用途

- **罐子偵測**：`can_detect_v4_best.pt` — 單類別，最穩定
- **罐子 + 手偵測**：`multiclass_detect_v3_best.pt` — 雙類別 bbox
- **手部姿態追蹤**：`ehand_pose_sam_v4_best.pt` — 最新最準，SAM 合成資料 + 16 keypoints

## 使用方式

```python
from ultralytics import YOLO

# Detection — 罐子偵測
model = YOLO("models/can_detect_v4_best.pt")
results = model("image.jpg")

# Pose — eHand 16 keypoint 追蹤
model = YOLO("models/ehand_pose_sam_v4_best.pt")
results = model("image.jpg")
for r in results:
    if r.keypoints is not None:
        kpts = r.keypoints.xy   # (N, 16, 2) — 每個偵測物件的 16 個關鍵點 xy
        conf = r.keypoints.conf # (N, 16) — 每個關鍵點的信心度
        # 取指尖座標
        thumb_tip = kpts[0, 4]   # index 4
        index_tip = kpts[0, 7]   # index 7
        middle_tip = kpts[0, 10] # index 10
```

## 訓練設定

- 輸入解析度：640x640
- 資料集：SAM 合成（URDF PyBullet 渲染 + SAM 去背 + domain randomization）+ 手動標註混合
- 訓練環境：RTX 5080, CUDA, conda env `yolo`
- 框架：Ultralytics YOLOv11
- 詳細訓練流程：`idaka5080/ehand-pose-training.md`

## 5080 訓練機資訊

- SSH: `ssh idaka_5080@192.168.0.245`
- 原始 runs: `~/openarm_yolo_training/runs/`
- 資料集: `~/openarm_yolo_training/datasets/`
