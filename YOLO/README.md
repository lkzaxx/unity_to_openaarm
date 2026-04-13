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

## 推薦用途

- **罐子偵測**：`can_detect_v4_best.pt` — 單類別，最穩定
- **罐子 + 手偵測**：`multiclass_detect_v3_best.pt` — 雙類別 bbox
- **手部姿態追蹤**：`ehand_pose_sam_v4_best.pt` — 最新最準，SAM 合成資料 + 16 keypoints

## 使用方式

```python
from ultralytics import YOLO

# Detection
model = YOLO("models/can_detect_v4_best.pt")
results = model("image.jpg")

# Pose
model = YOLO("models/ehand_pose_sam_v4_best.pt")
results = model("image.jpg")
for r in results:
    if r.keypoints is not None:
        kpts = r.keypoints.xy  # (N, 16, 2)
```

## 訓練設定

- 輸入解析度：640x640
- 資料集：SAM 合成 + 手動標註混合
- 訓練環境：RTX 5080, CUDA, conda env `yolo`
- 框架：Ultralytics YOLOv11

## 5080 訓練機資訊

- SSH: `ssh idaka_5080@192.168.0.245`
- 原始 runs: `~/openarm_yolo_training/runs/`
- 資料集: `~/openarm_yolo_training/datasets/`
