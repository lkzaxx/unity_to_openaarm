# Multi-Agent Sim-to-Real 工作流

> 目標：產出一個模型同時偵測 **can (罐子)** + **ehand (靈巧手)**，各帶 Pose 關鍵點
> 執行環境：5080 (RTX 5080 16GB) 訓練渲染 / Jetson Orin Nano 部署
> 最後更新：2026-03-19

---

## 架構總覽

```
┌─────────────────────────────────────────────────────┐
│                   主 Agent (協調者)                    │
│                                                     │
│  職責：                                              │
│  1. 派發任務給子 Agent                               │
│  2. 驗收每個步驟的產出                               │
│  3. 判斷通過/不通過 → 決定下一步或回滾               │
│  4. 最終整合並部署到 Jetson                          │
└──────────┬──────────┬──────────┬────────────────────┘
           │          │          │
     ┌─────▼────┐ ┌───▼────┐ ┌──▼───────┐
     │ Agent A  │ │Agent B │ │ Agent C  │  ... 依需求動態派發
     │ 截取影格 │ │渲染環境│ │ 背景準備 │
     └──────────┘ └────────┘ └──────────┘
```

---

## 最終產出

### 模型規格

```yaml
# data.yaml (多類別 Pose)
names:
  0: can      # 罐子
  1: ehand    # 靈巧手

kpt_shape: [6, 3]  # 兩個類別各 6 個關鍵點

# can 關鍵點:
#   0: top, 1: cap_center, 2: left, 3: right, 4: center, 5: bottom

# ehand 關鍵點:
#   0: palm_center, 1: thumb_tip, 2: index_tip, 3: middle_tip, 4: ring_tip, 5: pinky_tip

flip_idx: [0, 1, 3, 2, 4, 5]  # 翻轉時 left↔right / index↔pinky 等
```

### 檔案產出

```
~/openarm_yolo_training/runs/multiclass_pose_vN/weights/best.pt    ← PyTorch
~/ros2_ws/yolo_model/multiclass_pose_best.engine                   ← Jetson TensorRT
```

---

## Phase 1: 平行準備（3 個 Agent 同時跑）

### Agent A: 截取 eHand 影片幀

```
任務: 從 ehand_video.mp4 截取驗證用幀
輸入: ~/openarm_yolo_training/hand/source_video/ehand_video.mp4
輸出: ~/openarm_yolo_training/hand/source_frames/ (40-50 張 jpg)

步驟:
1. SSH 到 IDAKA_5080
2. source ~/miniconda3/etc/profile.d/conda.sh && conda activate yolo
3. 每 2 秒截取一幀 (影片 103.7s, 約 50 幀)
4. 縮小到 1280x720 (原始 2160x3840 直式 4K)
5. 輸出到 source_frames/

驗收標準:
- 至少 40 張幀
- 涵蓋不同手勢 (張開、握拳、側面)
- 圖片清晰不模糊
```

### Agent B: 安裝 PyBullet + 上傳 URDF + 測試渲染

```
任務: 建立 PyBullet 渲染環境，確認 eHand-6 URDF 能正確載入和渲染
輸入: ehand/灵巧手URDF 20250904/eHand-6-R/urdf/eHand-6-R.urdf + meshes/*.STL
輸出: ~/openarm_yolo_training/hand/rendered/test_render.jpg (測試渲染圖)

步驟:
1. SSH 到 IDAKA_5080
2. pip install pybullet (在 yolo conda env)
3. 上傳 URDF + STL 到 5080: ~/openarm_yolo_training/hand/urdf/
4. 修正 URDF 中的 mesh 路徑 (package:// → 相對路徑)
5. PyBullet DIRECT 模式載入 URDF
6. 渲染一張測試圖 → 確認 mesh 正確顯示
7. 設定材質顏色 (對齊真實外觀):
   - 手掌主體: RGB ~(160, 165, 170)
   - 手指關節: RGB ~(100, 105, 110)
   - 指尖: RGB ~(30, 30, 35)
8. 測試隨機關節角度 → 確認關節能動

已知問題:
- URDF 所有 joint limit 都是 lower=0 upper=0 (SolidWorks 匯出問題)
  → 需要用 JOINT_LIMITS dict 手動設定合理範圍
  → 參考 ehand-pose-training.md Section 8 的 JOINT_LIMITS
- STL mesh 路徑用 package:// 格式
  → 改成相對路徑或設定 setAdditionalSearchPath

驗收標準:
- URDF 載入不報錯
- 渲染圖能看到 eHand-6 的外觀
- 關節能隨機旋轉，手指會動
- 色調大致接近真實影片 (銀灰金屬 + 黑色塑膠)
```

### Agent C: 準備真實背景圖

```
任務: 從 eHand 影片中提取不含手的背景幀，或從罐子訓練的背景圖複製
輸入: source_frames/ + 既有 ~/openarm_yolo_training/datasets/can_backgrounds/
輸出: ~/openarm_yolo_training/hand/backgrounds/ (10-20 張)

步驟:
1. 從 can_backgrounds/ 複製既有背景圖
2. 從 eHand 影片截取幾張不含手的幀 (如果有的話)
3. 或用 inpainting 把手的區域塗掉

驗收標準:
- 至少 10 張背景圖
- 背景來自真實環境 (不是程式化生成)
```

---

## Phase 2: 合成資料生成（等 Phase 1 完成）

### Agent D: PyBullet 渲染 eHand 合成資料

```
任務: 用 PyBullet 渲染 eHand-6 不同姿態，自動產生 YOLO Pose 標註
輸入:
  - URDF: ~/openarm_yolo_training/hand/urdf/eHand-6-R.urdf
  - 背景: ~/openarm_yolo_training/hand/backgrounds/
輸出:
  - ~/openarm_yolo_training/hand/rendered/images/ (800-1000 張)
  - ~/openarm_yolo_training/hand/rendered/labels/ (自動關鍵點標註)

步驟:
1. 載入 URDF
2. 對每張圖:
   a. 隨機設定 16 個關節角度 (在 JOINT_LIMITS 範圍內)
   b. 隨機相機視角 (distance: 0.25-0.60, yaw: 0-360, pitch: -60~30)
   c. 渲染 RGB 圖 (640x480)
   d. getLinkState() 取得 6 個關鍵點 3D 座標
   e. 3D → 2D 投影
   f. 計算 bbox (包圍所有可見關鍵點 + margin)
   g. 背景替換 (PyBullet 灰色背景 → 真實背景)
   h. Domain Randomization (亮度、對比度、噪點、模糊)
   i. 輸出 YOLO Pose 格式標註
3. 分割 train/val (90/10)
4. 產生 data.yaml

關鍵點定義 (ehand, 6 點):
  0: palm_center → base_link
  1: thumb_tip   → link_2_3
  2: index_tip   → link_3_3
  3: middle_tip  → link_4_3
  4: ring_tip    → link_5_3
  5: pinky_tip   → link_6_3

JOINT_LIMITS (需校正):
  joint_1_1: (-0.5, 0.5)
  joint_2_1: (-0.3, 1.2)
  joint_2_2: (-0.2, 1.5)
  joint_2_3: (-0.2, 1.0)
  joint_3_1~6_3: (-0.2, 1.5) 大致範圍

驗收標準:
- 至少 800 張渲染圖
- 關鍵點投影在視覺上正確 (指尖點在指尖位置)
- 背景已替換為真實環境
- data.yaml 格式正確
```

### Agent E: 合併 Can + eHand 資料集

```
任務: 把罐子 Pose 資料和 eHand Pose 資料合併成多類別資料集
輸入:
  - ~/openarm_yolo_training/datasets/can_dataset_pose/ (罐子, class 0)
  - ~/openarm_yolo_training/hand/rendered/ (eHand, class 1)
輸出:
  - ~/openarm_yolo_training/datasets/multiclass_pose/

步驟:
1. 複製 can 資料 (保持 class=0)
2. 複製 ehand 資料 (改 class=1)
3. 合併 train/val
4. 產生新的 data.yaml (兩個類別)

注意:
- 兩個類別的關鍵點數量必須相同 (都是 6 個)
- flip_idx 需要考慮兩個類別的對稱性
- 如果關鍵點數量不同，YOLO Pose 不支援混合 → 需要統一為 6 點

驗收標準:
- data.yaml 包含兩個類別
- kpt_shape: [6, 3]
- 標註格式正確 (class cx cy w h + 6*3 個數字)
```

---

## Phase 3: 訓練（等 Phase 2 完成）

### Agent F: 訓練多類別 Pose 模型

```
任務: 訓練 yolo11n-pose 同時偵測 can + ehand
輸入: ~/openarm_yolo_training/datasets/multiclass_pose/data.yaml
輸出: ~/openarm_yolo_training/runs/multiclass_pose_v1/weights/best.pt

步驟:
1. SSH 到 IDAKA_5080
2. conda activate yolo
3. 訓練:
   from ultralytics import YOLO
   model = YOLO("yolo11n-pose.pt")
   model.train(
       data="datasets/multiclass_pose/data.yaml",
       epochs=200,
       imgsz=640,
       batch=32,  # pose 比 detect 吃更多 VRAM
       workers=8,
       device=0,
       patience=40,
       project="runs",
       name="multiclass_pose_v1",
   )

驗收標準:
- 訓練完成，沒有 error
- mAP50 > 0.7 (合成驗證集)
- 兩個類別都有合理的 AP
```

---

## Phase 4: 主 Agent 驗證（關鍵決策點）

```
主 Agent 親自執行驗證，不委派給子 Agent，不詢問用戶。

驗證原則:
- 主 Agent 自行下載渲染圖 + 真實影格，比對色調/結構/關鍵點
- 主 Agent 自行判斷通過/不通過，並決定下一步
- 只在最終部署到 Jetson 後，才讓用戶確認即時效果
- 中間過程的踩坑和調整全部記錄到此 MD

驗證流程:

─── 驗證 0: 渲染品質驗證 (Phase 2 完成後) ───
1. 下載渲染圖樣本 (5-10 張不同姿態)
2. 下載真實影格樣本 (hand/source_frames/)
3. 主 Agent 自行比對:
   - 色調: 渲染的金屬色/黑色是否接近真實
   - 結構: 手指比例、關節角度是否自然
   - 關鍵點: 投影位置是否在正確的指尖/手掌上
   - 背景: 替換後是否自然
4. 通過 → Phase 3 訓練
   不通過 → 調整渲染參數，重跑 Agent D

─── 驗證 A: 靜態圖片驗證 (Phase 3 完成後) ───
1. 用模型推論 can 真實照片 (source/ 和 source_video/)
   → 主 Agent 確認罐子偵測正確 + 關鍵點位置合理
2. 用模型推論 eHand 真實影格 (hand/source_frames/)
   → 主 Agent 確認靈巧手偵測正確 + 指尖點位置合理
3. 主 Agent 自行判斷通過/不通過

─── 驗證 B: Jetson RealSense 即時驗證 ───
4. 部署模型到 Jetson (匯出 TensorRT, task=pose)
5. 啟動 realsense_yolo_viewer.py
6. eHand-6 在 RealSense 前做隨機動作:
   - 張開手掌
   - 握拳
   - 逐指彎曲
   - 快速開合
   - 不同角度 (正面、側面、背面)
   - 手持罐子 (同時偵測 can + ehand)
7. 在 Web 介面 (http://192.168.0.15:8081/) 即時觀察:
   - bbox 是否穩定跟隨
   - 關鍵點是否跟著手指移動
   - 追蹤 ID 是否持續不跳
   - 同時偵測 can + ehand 是否互不干擾

驗證 B 的優勢:
- eHand 目前在 Jetson 上做隨機動作 → 不需要人操作就能測試
- RealSense 即時影像 = 最真實的測試環境
- Web 介面可以遠端觀察，不需要接螢幕
- 可以長時間跑 → 統計偵測率和追蹤穩定性

判斷標準:
┌───────────────────────────────────────────────────┐
│ 驗證 A: 靜態圖片                                   │
│                                                   │
│ Can 偵測                                          │
│   ✅ 通過: bbox 正確框住罐子, conf > 0.7           │
│            關鍵點在罐子上 (top/bottom/left/right)  │
│   ❌ 不通過: 漏偵測 / 誤報 / 關鍵點亂飄           │
│                                                   │
│ eHand 偵測                                        │
│   ✅ 通過: bbox 正確框住靈巧手, conf > 0.5         │
│            指尖點大致在指尖位置 (允許 ±20px 誤差)  │
│   ❌ 不通過: 偵測不到 / 指尖點完全偏移             │
├───────────────────────────────────────────────────┤
│ 驗證 B: Jetson RealSense 即時                      │
│                                                   │
│   ✅ 通過:                                         │
│   - eHand bbox 穩定跟隨 (不閃爍/不跳)             │
│   - 指尖關鍵點跟著手指動作移動                     │
│   - 張開/握拳時指尖點位置合理                      │
│   - 追蹤 ID 連續 30 秒不丟失                       │
│   - can + ehand 同時出現時都能偵測                 │
│   - 推論速度 < 30ms (TensorRT)                    │
│                                                   │
│   ❌ 不通過:                                       │
│   - eHand 在某些角度完全偵測不到                   │
│   - 關鍵點跟手指動作無關 (亂飄)                    │
│   - 追蹤 ID 頻繁跳號 (> 每 5 秒跳一次)            │
│   - can 和 ehand 互相干擾 (框錯類別)              │
└───────────────────────────────────────────────────┘

不通過時的回滾策略:
├── Can 不通過 → 不太可能，已有 v3/v4 成功經驗
├── eHand 偵測不到 → 回 Phase 2，調整渲染:
│   ├── 色調差異太大 → 調整材質顏色 / 升級 pyrender
│   ├── 視角不夠多 → 增加相機隨機範圍
│   └── 背景差異 → 加更多真實背景圖
├── eHand 指尖偏移 → 回 Phase 2:
│   ├── 3D→2D 投影有 bug → 檢查 project_3d_to_2d()
│   ├── 關節角度範圍不對 → 校正 JOINT_LIMITS
│   └── URDF 結構問題 → 檢查 link 對應是否正確
├── 即時追蹤不穩 → 調整追蹤器:
│   ├── ID 跳號 → 增加 track_buffer (90→150)
│   ├── 閃爍 → 降低 new_track_thresh
│   └── 追蹤器崩潰 → 確認用 botsort.yaml + try/except
├── 兩個類別互相干擾 → 增加各自的訓練資料量
└── RealSense 角度跟訓練資料差太多 →
    用 RealSense 截圖加入訓練 (見下方「RealSense 即時截圖回饋機制」)
```

### RealSense 即時截圖回饋機制

> 驗證不通過時，可直接從 RealSense 截圖加入訓練資料或標註，縮短 Sim-to-Real gap。

```
驗證不通過
    │
    ▼
RealSense 截圖 (Jetson 上)
    │
    ├── 用途 A: 加入背景圖 → 回 Phase 2 重新合成
    │   適用: 背景差異導致誤報
    │   做法: 截幾張不含 eHand/can 的空景 → 上傳到 5080 backgrounds/
    │
    ├── 用途 B: 加入訓練資料 (自動標註) → 回 Phase 3 fine-tune
    │   適用: 特定角度/手勢偵測不到
    │   做法: 截圖 → 現有模型預標註 → 人工修正 → 混合重新訓練
    │
    └── 用途 C: 即時錄影截幀 → 大量擴充訓練資料
        適用: 整體偵測率太低，需要大量真實資料
        做法: 錄 30-60 秒 → 截取 30-50 幀 → 標註 → fine-tune
```

#### 截圖指令

```bash
# 方法 1: Web API 單張截圖 (從任何電腦)
curl http://192.168.0.15:8081/snapshot?mode=color -o realsense_capture.jpg

# 方法 2: 批量截圖腳本 (Jetson 上執行)
ssh IDAKA_ROBOT
python3 -c "
import pyrealsense2 as rs
import cv2, os, time
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

out_dir = os.path.expanduser('~/ros2_ws/yolo_model/realsense_captures')
os.makedirs(out_dir, exist_ok=True)

for i in range(30):  # 截 30 張，每秒一張
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if color:
        import numpy as np
        img = np.asanyarray(color.get_data())
        cv2.imwrite(f'{out_dir}/rs_capture_{i:03d}.jpg', img)
    time.sleep(1)

pipeline.stop()
print(f'Saved 30 frames to {out_dir}')
"

# 方法 3: 從 Web Viewer 的 YOLO 模式截圖 (含標註框)
curl http://192.168.0.15:8081/snapshot?mode=yolo -o realsense_yolo_capture.jpg
```

#### 截圖上傳到 5080 訓練

```bash
# 從 Jetson 截圖 → 傳到 5080
scp IDAKA_ROBOT:~/ros2_ws/yolo_model/realsense_captures/*.jpg \
    IDAKA_5080:~/openarm_yolo_training/hand/realsense_captures/

# 用途 A: 當背景圖 (不含 eHand 的幀)
scp IDAKA_ROBOT:~/ros2_ws/yolo_model/realsense_captures/empty_*.jpg \
    IDAKA_5080:~/openarm_yolo_training/hand/backgrounds/

# 用途 B: 當訓練資料 (含 eHand 的幀)
# → 先用模型預標註，再人工修正
ssh IDAKA_5080
conda activate yolo
python3 -c "
from ultralytics import YOLO
model = YOLO('runs/multiclass_pose_vN/weights/best.pt')
model.predict(
    source='hand/realsense_captures/',
    save=True, save_txt=True,
    project='hand/realsense_captures/', name='auto_labels'
)
"
# → 輸出預標註到 hand/realsense_captures/auto_labels/
# → 上傳到 CVAT 修正 → 加入 dataset_pose/ → 重新訓練
```

#### 迭代加速效果

| 迭代輪 | 資料來源 | 預期效果 |
|--------|---------|---------|
| v1 | 純合成 (PyBullet 渲染) | 基礎偵測能力 |
| v2 | 合成 + RealSense 背景圖 | 減少誤報 |
| v3 | 合成 + RealSense 截圖 fine-tune | 真實環境準確率大幅提升 |
| final | 合成 + 大量 RealSense 標註 | 產線級穩定性 |

> **關鍵：每次迭代不需要從零開始，只需要加入 RealSense 截圖補強弱項。**

### Jetson 即時驗證指令

```bash
# 1. 部署模型
scp IDAKA_5080:~/openarm_yolo_training/runs/multiclass_pose_vN/weights/best.pt \
    /tmp/multiclass_pose.pt
scp /tmp/multiclass_pose.pt IDAKA_ROBOT:~/ros2_ws/yolo_model/

# 2. Jetson 上匯出 TensorRT
ssh IDAKA_ROBOT
cd ~/ros2_ws/yolo_model
python3 -c "
from ultralytics import YOLO
model = YOLO('multiclass_pose.pt', task='pose')
model.export(format='engine', half=True)
"

# 3. 更新 realsense_yolo_viewer.py 的 YOLO_MODEL 路徑
# 4. 啟動
python3 ~/realsense/realsense_yolo_viewer.py

# 5. 瀏覽器開 http://192.168.0.15:8081/ → YOLO Track
# 6. 觀察 eHand 隨機動作時的偵測效果
```

### 即時驗證 API（可程式化統計）

```python
# 用 /yolo_status API 自動統計偵測率
import requests, time

stats = {"detected": 0, "missed": 0}
for _ in range(300):  # 統計 300 次 (~30 秒 @10Hz)
    r = requests.get("http://192.168.0.15:8081/yolo_status").json()
    if r["detections"] > 0:
        stats["detected"] += 1
    else:
        stats["missed"] += 1
    time.sleep(0.1)

rate = stats["detected"] / (stats["detected"] + stats["missed"]) * 100
print(f"Detection rate: {rate:.1f}%")
# ✅ 通過標準: > 85%
```

---

## Phase 5: 自動標註 + Fine-tune（驗證通過後）

### Agent G: 自動預標註真實影格

```
任務: 用 Phase 3 的模型推論真實影片幀，輸出預標註
輸入:
  - 模型: runs/multiclass_pose_v1/weights/best.pt
  - 真實幀: hand/source_frames/
輸出:
  - hand/auto_labels/ (預標註 txt 檔)

步驟:
1. 模型推論每張真實幀
2. 輸出 YOLO Pose 格式標註
3. 同時儲存標註視覺化圖 (用於人工檢查)

人工修正:
- 上傳預標註到 CVAT/Roboflow
- 修正偏移的關鍵點 (比從零標快 3-5 倍)
- 匯出修正後的標註
```

### Agent H: 混合資料重新訓練

```
任務: 合成 + 真實標註混合，訓練最終版模型
輸入:
  - 合成資料 (Phase 2)
  - 修正後的真實標註 (Phase 5 Agent G)
輸出:
  - runs/multiclass_pose_final/weights/best.pt

策略:
- 真實資料比例提高 (合成:真實 = 7:3)
- 或先用合成預訓練 → 再用真實 fine-tune
```

---

## Phase 6: 部署到 Jetson

### Agent I: Jetson 部署

```
任務: 匯出 TensorRT + 更新 Web Viewer
輸入: best.pt (最終版)
輸出: ~/ros2_ws/yolo_model/multiclass_pose_best.engine

步驟:
1. SCP 模型到 Jetson
2. 匯出 TensorRT: YOLO(path, task="pose").export(format="engine", half=True)
   ⚠️ 必須指定 task="pose"，否則 TensorRT engine 會被當成 detect 載入
3. 更新 realsense_yolo_viewer.py:
   - YOLO_MODEL 指向新 engine
   - 載入時指定 task="pose"
   - 追蹤器用 botsort.yaml (不要用 bytetrack，跟 pose 有相容性問題)
   - 加 try/except 防 tracker 崩潰
   - 畫兩組關鍵點 (can 用罐子骨架, ehand 用手指骨架)
4. 測試 Web Viewer

已知踩坑:
- TensorRT engine 不帶 task 資訊 → 載入時必須 YOLO(path, task="pose")
- ByteTrack + Pose 會因為異常 bbox 導致 Kalman Filter 崩潰 → 用 BoT-SORT
- Jetson 上 scipy 版本跟 numpy 不相容有 warning → 可忽略但不影響功能
```

---

## 機器連線資訊

### 5080 (訓練/渲染)

```bash
ssh IDAKA_5080
# Host: 192.168.0.245, User: idaka_5080
# SSH Key 免密碼已設定
# conda activate yolo
```

### Jetson Orin Nano (部署)

```bash
ssh IDAKA_ROBOT
# Host: 192.168.0.15, User: idaka
# SSH Key 免密碼已設定
# PyTorch 2.3.0 (CUDA 12.4, aarch64)
# TensorRT 10.3.0
```

---

## 5080 目錄結構

```
~/openarm_yolo_training/
├── datasets/
│   ├── can_source/              ← 罐子原始照片 (7 張 RealSense)
│   ├── can_source_video/        ← 罐子影片截取幀 (39 張)
│   ├── can_cutouts/             ← 罐子 SAM 去背 (7 張)
│   ├── can_cutouts_video/       ← 罐子影片去背 (36 張)
│   ├── can_backgrounds/         ← 罐子真實背景 (7 張 inpainting)
│   ├── can_dataset_v3/          ← 罐子 Detect v3 資料集
│   ├── can_dataset_v4/          ← 罐子 Detect v4 資料集
│   ├── can_dataset_pose/        ← 罐子 Pose 資料集 (1100 張)
│   └── multiclass_pose/         ← 最終多類別 Pose 資料集 (待生成)
│
├── hand/                        ← eHand 前處理目錄
│   ├── source_video/            ← ehand_video.mp4 (604MB)
│   │   └── ehand_video.mp4
│   ├── source_frames/           ← 影片截取幀 (待生成, ~50 張)
│   ├── rendered/                ← PyBullet 渲染圖 (待生成)
│   │   ├── images/
│   │   └── labels/
│   ├── cutouts/                 ← SAM 去背 (備用路線)
│   ├── backgrounds/             ← 真實背景圖
│   ├── dataset_pose/            ← eHand Pose 資料集 (待生成)
│   ├── scripts/                 ← 渲染/合成腳本
│   └── urdf/                    ← eHand-6 URDF + STL (待上傳)
│
├── runs/                        ← 訓練結果
│   ├── can_v3/                  ← Detect v3
│   ├── can_v4/                  ← Detect v4
│   ├── can_pose_v1/             ← 罐子 Pose v1
│   ├── multiclass_pose_v1/      ← 多類別 Pose v1 (待訓練)
│   └── multiclass_pose_final/   ← 最終版 (待訓練)
│
├── test/                        ← 測試結果
│   └── hand/                    ← eHand 偵測測試
│       ├── world/               ← YOLO-World 結果
│       ├── yolov11/             ← YOLOv11 預訓練結果
│       └── preview/             ← 影片預覽幀
│
└── scripts/                     ← 訓練腳本
    └── can_training/            ← 罐子訓練全套腳本
```

---

## Jetson 模型目錄

```
~/ros2_ws/yolo_model/
├── can_v3_best.engine           ← Detect v3 (舊)
├── can_v4_best.engine           ← Detect v4
├── can_pose_v1_best.engine      ← 罐子 Pose
├── multiclass_pose_best.engine  ← 最終版 can+ehand Pose (待部署)
├── can_tracker_config.yaml      ← 追蹤器設定
└── realsense_yolo_viewer.py → ~/realsense/realsense_yolo_viewer.py
```

---

## 迭代記錄

> 每次迭代在這裡記錄版本、問題、調整

### Iteration 0: 預訓練測試 (2026-03-19)

```
測試: YOLO-World + YOLOv11 預訓練能否偵測 eHand
結果: 基本不行 (World 僅 1/6 幀 conf=0.18, v11 全是誤報)
結論: 必須自訂訓練
```

### Iteration 1: Phase 1 進行中 (2026-03-19)

```
Phase 1 結果:

Agent A (截取影格): ✅ 完成 — 53 張 (frame_000~052, 1280x720, 每 2 秒)
Agent B (PyBullet 環境): ✅ 完成
  - pybullet 3.2.7 安裝 OK
  - URDF 載入 OK (21 joints, 16 可動)
  - Mesh 路徑修正: package://eHand-6-R/meshes/ → meshes/ (sed 替換)
  - 渲染測試 OK (預設姿態 + 隨機姿態)
  - 6 個關鍵點 3D 座標取得 OK
  - ⚠️ 色調: 目前是 PyBullet 預設灰色，需要在 Phase 2 調整
Agent C (背景準備): ✅ 完成 — 11 張 (7 罐子背景 + 4 影片首尾截取)

踩坑:
  - URDF mesh 路徑用 package:// 格式，PyBullet 無法識別
    → 解法: sed -i 's|package://eHand-6-R/meshes/|meshes/|g' eHand-6-R.urdf

渲染引擎: PyBullet TinyRenderer
材質設定: changeVisualShape 設定深灰/黑色
關節範圍: JOINT_LIMITS 估計值，關節可正常旋轉
合成數量: 900 張 (800 train + 100 val)

Phase 2 結果: ✅ 渲染完成 900 張 + 自動關鍵點標註
Phase 3 結果: ✅ 訓練完成 (Box mAP50=0.991, Pose mAP50=0.986)

Phase 4 驗證 A 結果:
  Can:   7/7 正確 ✅
  eHand: 0/53 偵測率 0% ❌❌❌ 完全失敗
  問題: 全部被誤認為 can 或偵測不到
  根因: PyBullet 渲染 (灰色 CAD) 跟真實 eHand (金屬反光) 差異太大
        Sim-to-Real gap 無法靠 domain randomization 彌補

決策: 放棄純渲染路線 → 改用 SAM 切割路線 (跟罐子相同方法, 已驗證有效)
```

### Iteration 2: SAM 切割路線 (2026-03-19)

```
策略變更: 放棄 PyBullet 渲染 → 改用影片 SAM 切割 + 合成
理由: 罐子用此方法已達 0.93-0.95 信心度，eHand 應該也能用

流程:
1. 從 eHand 影片截取幀 (已有 53 張) ✅
2. SAM 切割 eHand 去背
   第一次嘗試: SAM auto mode (argmax areas) → ❌ 全部切到背景桌面
   踩坑: eHand 不是畫面最大物件，argmax 選錯
   修正: 改用中心點提示 + 面積過濾 (5%~60%)
   結果: 53 張切出，品質篩選後 32 張通過
   篩選條件: fill_ratio 0.25~0.95, min_size 80px, aspect < 5

   ⚠️ 踩坑: 影片截取時比例錯誤！
   原始影片: 2160x3840 (直式 9:16)
   錯誤縮放: cv2.resize(frame, (1280, 720)) → 強制變成橫式，eHand 被橫向拉伸
   修正: cv2.resize(frame, (540, 960)) → 保持原始 9:16 比例
   → 需要重跑全部: SAM 切割 → 篩選 → 合成 → 訓練 → 測試
3. 合成訓練資料 (eHand cutouts 貼到真實背景) ✅
   32 張 cutouts + 11 張背景 → 900 張合成 (800 train + 100 val)
4. 合併 can(v4) + eHand → 多類別 Detect 訓練 ✅
   Box mAP50: 0.987, mAP50-95: 0.927
5. 驗證 A: ✅✅ 通過！
   Can:   7/7 正確 (零誤報)
   eHand: 52/53 = 98% 偵測率 (零誤報為 can, 只有 1 張漏偵測)

   對比: Iteration 1 (渲染路線) eHand 0% → Iteration 2 (SAM 路線) 98%
   結論: SAM 切割路線大幅優於 PyBullet 渲染路線

6. 比例修正後重跑 (v2):
   截取修正: 2160x3840 → 540x960 (保持 9:16)
   SAM 重切 → 篩選 → 合成 → 訓練 → 測試
   v2 結果: Can 7/7 ✅, eHand 53/53 = 100% ✅ (從 98% 提升)
   mAP50: 0.988, mAP50-95: 0.930

7. Jetson 部署 v2: multiclass_detect_v2.engine ✅
8. Pose v2 訓練: Box mAP50=0.983, Pose mAP50=0.931
   Can pose: ✅ 正常
   eHand pose: ❌ 不通過
   問題: 重疊框(2-3個bbox/手), 關鍵點飄到框外, 信心度低(0.3-0.5)
   根因: SAM 輪廓指尖提取不夠精確 + cutout 品質參差
   → 疊代: 用 detect bbox 輔助 SAM 重切 + 改進指尖提取演算法

9. Pose v3 疊代:
   改進: detect bbox→SAM 更精確切割 + convex hull 指尖提取 + 1000張合成
   訓練指標: Box mAP50=0.990, Pose mAP50=0.983, Pose mAP50-95=0.890 (v2: 0.486→0.890 翻倍)
   Can: 7/7 ✅ (conf=0.5)
   eHand: 64/104 = 62% (conf=0.3) ⚠️ 偵測率比 detect 低
   原因: Pose 模型需要同時預測 bbox+6點，信心度門檻更高
   部署: Jetson multiclass_pose_v3.engine (task=pose)

10. Pose v4 疊代: 擴充背景圖 (inpaint 104 幀 → 100+張背景) + 更多合成資料
    eHand 單獨: 99/104 = 95% ✅ (超過 85% 目標！)
    Can: 0/7 ❌ (Agent 合併時 kpt_shape 寫成 [4,3] 而非 [6,3])
    v4_fixed (Agent 版, 4 關鍵點): Can 7/7 ✅ eHand 97/104 = 93% ✅ 零類別混淆
    v4_fixed (我的版, 6 關鍵點): Can 7/7 ✅ eHand 41/104 = 39% ❌

    ⚠️ 踩坑: Agent 的合成腳本只提取了 4 個關鍵點 (不是 6)
    但意外發現: 4 關鍵點版本偵測率 93% >> 6 關鍵點版本 39-62%
    原因: 少的關鍵點更容易學準，降低了模型負擔

    決策: 部署 4 關鍵點版本 (93%) 到 Jetson
    同時等 v5 (6 kpt + 擴充背景) 完成，如果 >85% 就替換為 6 kpt 版

11. Pose v5 計劃: 混合路線 — 真實顏色 + URDF 精確關鍵點 (v4 完成後實作)

---

### 混合路線技術方案 (Pose v5+)

> 核心概念: 從真實照片提取 eHand 顏色 → 套到 URDF 上渲染 → 用 URDF 的 3D 關節座標產生精確關鍵點標註
> 解決的問題: SAM convex hull 指尖提取不精確 + URDF 渲染色調太假

```
真實照片                              URDF 3D 模型
    │                                     │
    ▼                                     ▼
detect bbox → SAM mask              PyBullet 載入
    │                                     │
    ▼                                     ▼
分區域提取顏色                      changeVisualShape(真實顏色)
├── 手掌: RGB平均色                       │
├── 手指: RGB平均色                       ▼
└── 指尖: RGB平均色               隨機姿態 + 真實色調渲染
                                          │
                                          ▼
                                   getLinkState() 自動關鍵點
                                   (精確的 3D → 2D 投影)
                                          │
                                          ▼
                                   + 真實背景替換
                                   + Domain Randomization
                                          │
                                          ▼
                                   YOLO Pose 格式標註
                                   (精確關鍵點 + 真實色調)
```

#### 為什麼混合路線能解決目前問題

| 目前問題 | 根因 | 混合路線解法 |
|---------|------|------------|
| eHand Pose 偵測率低 (62%) | 合成外觀跟真實差異 | 真實顏色套到 URDF |
| 指尖關鍵點不準 | convex hull 只是猜測 | URDF getLinkState() 精確 3D 座標 |
| 姿態多樣性不足 | 只有 82 張 cutout | URDF 隨機 16 個關節 = 無限姿態 |
| Iteration 1 URDF 渲染 0% | 預設灰色 vs 真實金屬色 | 用真實照片的顏色 |

#### 實作步驟

```
Step 1: 從真實圖片提取 eHand 各部位顏色

  使用 detect v3 模型 + SAM 分割:
  for 每張 ehand 真實幀:
    1. detect model → ehand bbox
    2. SAM(bbox prompt) → 精確 mask
    3. 在 mask 內分區域 (上/中/下 三等分):
       - 上 1/3 (手掌/手腕): 取 RGB 平均色 → palm_color
       - 中 1/3 (手指根部): 取 RGB 平均色 → finger_color
       - 下 1/3 (指尖): 取 RGB 平均色 → tip_color
    4. 對所有幀的顏色取中位數 → 穩定的顏色估計

Step 2: 套用顏色到 URDF

  URDF link → 對應顏色:
  base_link, link_1, link_3~6    → palm_color  (手掌主體)
  link_X_1, link_X_2             → finger_color (手指關節)
  link_X_3 (所有指尖 link)       → tip_color   (指尖)

  p.changeVisualShape(body_id, link_idx, rgbaColor=color)

Step 3: URDF 渲染合成資料 (帶精確關鍵點)

  for 1000-1500 張:
    1. 隨機關節角度 (JOINT_LIMITS)
    2. 隨機相機視角
    3. 渲染 (已套用真實顏色)
    4. getLinkState() → 6 個關鍵點精確 3D 座標
    5. 3D → 2D 投影 → 精確像素座標
    6. 真實背景替換 (backgrounds_expanded/)
    7. Domain Randomization
    8. 輸出 YOLO Pose 標註

Step 4: 混合訓練

  方案 A: URDF 合成 (精確關鍵點) + SAM cutout 合成 (真實外觀) 混合
  方案 B: 只用 URDF 合成 (如果真實顏色套用後夠好)

  合併 can_pose + ehand_pose → multiclass_pose_v5

Step 5: 驗證 + 部署

  目標:
  - eHand 偵測率 > 85% (conf=0.3)
  - 指尖關鍵點在正確位置 (比 v3 更準)
  - Can 維持 100%
```

#### 預期效果

| | v3 (SAM only) | v5 (混合) |
|---|---|---|
| 外觀真實感 | ✅ 高 (真實照片切割) | ✅ 高 (真實顏色套 URDF) |
| 關鍵點精度 | ❌ convex hull 猜測 | ✅ URDF 3D 關節精確座標 |
| 姿態多樣性 | ⚠️ 82 張 cutout | ✅ 無限隨機姿態 |
| 背景多樣性 | ⚠️ 11-100 張 | ✅ 100+ 張 (inpaint 擴充) |
| 偵測率預期 | 62% | **>85%** |

#### 迭代收斂機制

> 不需要一次提取完美顏色。每次迭代模型變好 → 偵測更多幀 → 提取更準顏色 → 渲染更像 → 模型更好。正向循環。

```
v5: 粗略顏色 → 渲染 → 訓練 → 偵測率 ~70%
     │  模型更好 → 偵測更多幀 → 更準的顏色提取
     ▼
v6: 修正顏色 → 重新渲染 → 訓練 → 偵測率 ~80%
     │  更多幀 → 更穩定的顏色估計 + 可能分更細的區域
     ▼
v7: 精準顏色 → 渲染接近真實 → 訓練 → 偵測率 90%+
     │
     ▼  通過驗證 → 部署

每輪自動執行:
1. 用當前最佳模型偵測所有影片幀 (conf > 模型信心度中位數)
2. 從偵測到的 bbox + SAM mask 提取分區域顏色
3. 更新 URDF changeVisualShape 顏色
4. 重新渲染 + 合成 + 訓練
5. 測試偵測率 → 不到 85% 就繼續迴圈
```

---

### Pose v9 計劃：反向色域對齊（待實作）

> 核心概念：不讓 URDF 變真實，而是讓真實變 URDF。把真實照片的 eHand 換成灰色 → 跟 URDF 同色域。

```
真實照片 → SAM mask → mask 內顏色換成 URDF 灰色 → 灰色化真實圖
URDF 渲染 → 本來就是灰色 → 精確關鍵點標註

混合訓練:
  灰色化真實圖 (真實姿態+背景, convex hull 關鍵點)
  + URDF 渲染 (隨機姿態, 精確關鍵點)
  → 同色域，模型同時學到結構和精確關鍵點
```

#### 實作步驟

```
Step 1: 灰色化真實圖片
  - 用 detect v3 bbox → SAM mask
  - mask 內 pixel → 換成 URDF 灰色 RGB(128,128,128)
  - 或更精確: 轉灰階但保持明暗漸變
  - 背景保持原樣
  - 關鍵點: 用 convex hull 或直接不標 (靠 URDF 資料學)

Step 2: URDF 渲染 (用 PyBullet TinyRenderer 即可，因為色域已對齊)
  - 不需要 pyrender (反正都是灰色)
  - 隨機姿態 + 精確 getLinkState() 關鍵點
  - 背景替換為真實背景

Step 3: 混合
  - 灰色化真實: ~100 張 (source_frames_dense)
  - URDF 渲染: ~1000 張
  - Can pose: 不變 (class 0)
  - eHand: class 1

Step 4: 訓練
  - domain randomization 要包含灰色↔彩色的變化
  - 讓模型能同時認灰色和彩色的 eHand

Step 5: 驗證
  - 用原始彩色影片幀測試 (不是灰色化的)
```

#### 已有資源

| 資源 | 位置 |
|------|------|
| eHand cutouts (SAM, 82張) | ~/openarm_yolo_training/hand/cutouts_v3/ |
| eHand dense frames (104張) | ~/openarm_yolo_training/hand/source_frames_dense/ |
| Detect v3 model | ~/openarm_yolo_training/runs/detect/runs/multiclass_detect_v3/weights/best.pt |
| URDF + meshes | ~/openarm_yolo_training/hand/urdf/eHand-6-R/ |
| 背景 (114張) | ~/openarm_yolo_training/hand/backgrounds_expanded/ |
| Can pose dataset | ~/openarm_yolo_training/datasets/can_dataset_pose/ |
| 用戶標的顏色 | ~/openarm_yolo_training/hand/color_extraction/colors.json + keypoint_mapping.json |

#### 恢復指令

usage reset 後開新 session，說：
「繼續 agent-workflow，執行 Pose v9 反向色域對齊方案」

主 Agent 需要:
1. 檢查 v8 Agent 結果 (可能已完成)
2. 實作 v9 灰色化 + URDF 混合
3. 訓練 + 驗證
4. 通過後部署到 Jetson
```

### 完整迭代總表 (截至 2026-03-19)

| 版本 | 方法 | Can | eHand | Pose mAP50 | 關鍵點 | 狀態 |
|------|------|-----|-------|-----------|--------|------|
| Iteration 1 | PyBullet URDF 渲染 | 7/7 ✅ | 0/53 (0%) ❌ | 0.986 (合成) | 6 | 失敗: Sim-to-Real gap |
| Detect v1 | SAM cutout 比例錯 | 7/7 ✅ | 52/53 (98%) | - | - | bbox OK |
| Detect v2 | SAM cutout 比例修正 | 7/7 ✅ | 53/53 (100%) | - | - | bbox 完美 |
| Pose v2 | SAM + convex hull 6kpt | 7/7 ✅ | 100/104 (96%) | 0.931 | 6 | 指尖偏移 |
| Pose v3 | detect bbox→SAM + hull 6kpt | 7/7 ✅ | 64/104 (62%) | 0.983 | 6 | 偵測率降 |
| Pose v4 (Agent) | 擴充背景 + 4kpt | 7/7 ✅ | 97/104 (93%) ✅ | 0.983 | 4 | **目前部署** |
| Pose v4_fixed | 6kpt + v3資料 | 7/7 ✅ | 41/104 (39%) | 0.978 | 6 | 背景不足 |
| Pose v5 | 6kpt + 擴充背景 | 7/7 ✅ | 100/104 (96%) ✅ | 0.978 | 6 | **最終部署** |
| Pose v6 | URDF + 真實顏色 (純渲染) | 7/7 ✅ | 0/104 (0%) ❌ | - | 6 | 失敗: 即使套真實顏色 gap 仍太大 |
| Pose v7 | URDF + SAM 混合 | 7/7 ✅ | 102/104 (98%) | 0.974 | 6 | 偵測率好但關鍵點不準 |
| Pose v8 | pyrender + 真實顏色 + SAM 混合 | 7/7 ✅ | 100/104 (96%) | 0.973 | 6 | 偵測OK但關鍵點不準(URDF+SAM矛盾) |
| Pose v9 | 灰色化真實圖(kpt=0) + URDF | 7/7 ✅ | 100/104 (96%) | 0.945 | 6 | kpt 0%: visibility=0 壓制了輸出 |
| **Pose v9b** | **灰色化真實圖(kpt=convex hull) + URDF** | **7/7 ✅** | **92/104 (88%)** | **0.946** | **6** | **kpt 75%! 方向對但精度不足** |
| Pose v10 | 灰色域訓練→回算kpt→彩色域訓練 | 跑中 | 跑中 | - | 6 | 反向標註方案 |

### 待加入資料 (v10 完成後)

- RealSense 即時截圖: `debug/螢幕擷取畫面 2026-03-20 103013.png`
  - 場景: can + eHand 同框，RealSense 俯視角
  - 用途: 加入 dataset 訓練 + 作為驗證基準
  - 需要從 Jetson RealSense 截更多同類場景
- RealSense BAG 錄影: `idaka@192.168.0.15:/tmp/realsense_hand_20260320_024715.bag`
  - 大小: 5.9 GB
  - 內容: eHand 使用中的 RealSense RGB+Depth 錄影
  - 用途: 截取影格 → 加入 dataset (最真實的訓練資料)
  - 處理: 用 pyrealsense2 讀取 bag → 截取 color frames → 灰色化/直接加入

### 關鍵踩坑總結

| 踩坑 | 影響 | 解法 |
|------|------|------|
| 影片截取比例錯 (1280x720 vs 540x960) | eHand 被拉伸 | 保持原始 9:16 比例 |
| SAM auto mode 選最大 mask | 切到背景不是手 | 改用中心點提示 + 面積過濾 |
| PyBullet 渲染色調太假 | eHand 0% 偵測率 | 改用 SAM 真實照片切割 |
| Agent 合併時 kpt_shape 錯 (4 vs 6) | Can 0% | 驗證 data.yaml 格式 |
| 程式化背景 vs 真實背景 | 大量誤報 | 必須用 inpaint 真實背景 |
| ByteTrack + Pose 崩潰 | Kalman Filter crash | 改用 BoT-SORT + try/except |
| TensorRT engine 不帶 task 資訊 | Pose 被當 Detect 載入 | YOLO(path, task="pose") |
| convex hull 指尖提取不穩定 | 關鍵點飄移 | 4 kpt 反而比 6 kpt 穩定 |

---

## 參考文件

| 文件 | 內容 |
|------|------|
| `ehand-pose-training.md` | eHand Pose 完整技術文件 (URDF 結構、關鍵點定義、PyBullet 腳本) |
| `yolo-training-setup.md` | YOLO 訓練環境 + 罐子訓練迭代記錄 + Jetson 部署 |
| `system-info.md` | 5080 硬體規格 |
| `JETSON_SSH_GUIDE.md` | Jetson 連線 + 已安裝服務 |

---

## 常見問題速查

| 問題 | 解法 | 參考 |
|------|------|------|
| SSH 連不上 5080 | `ssh IDAKA_5080`，確認同網段 192.168.0.x | yolo-training-setup.md Section 0 |
| SSH 連不上 Jetson | `ssh IDAKA_ROBOT`，密碼 idaka987 | JETSON_SSH_GUIDE.md |
| Port 8081 被佔 | `sudo fuser -k 8081/tcp` | — |
| conda env 找不到 | `source ~/miniconda3/etc/profile.d/conda.sh && conda activate yolo` | — |
| PyBullet URDF 路徑錯 | 修改 `package://` 為相對路徑，或 `setAdditionalSearchPath` | ehand-pose-training.md Section 8 |
| URDF joint limit 都是 0 | 手動設定 JOINT_LIMITS dict | ehand-pose-training.md Section 8 |
| TensorRT engine task 錯 | `YOLO(path, task="pose")` 明確指定 | yolo-training-setup.md Section 11.5 |
| ByteTrack + Pose 崩潰 | 改用 `botsort.yaml` + try/except | yolo-training-setup.md 追蹤器章節 |
| Jetson torch CPU 版 | 必須用 nvidia.box.com 的 aarch64 wheel | yolo-training-setup.md Step 2 |
| torchvision 不相容 | `--no-build-isolation` 源碼建置 | yolo-training-setup.md Step 2 踩坑 |
| 合成資料誤報多 | 加真實背景圖 (inpainting) | yolo-training-setup.md v1→v2 記錄 |
| 罐子上下分框 | SCALE_MAX 調到 0.65, ROTATION ±60° | yolo-training-setup.md v3 記錄 |
| 渲染色調差太多 | 升級 pyrender / 加強 domain randomization | ehand-pose-training.md Section 11 |
