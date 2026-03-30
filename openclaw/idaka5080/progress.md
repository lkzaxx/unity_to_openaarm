# OpenArm YOLO + eHand Pose 進度總覽

> 最後更新：2026-03-22
> 工作目錄：5080 `~/openarm_yolo_training/` / Jetson `~/ros2_ws/yolo_model/`

---

## 已完成

### Can (罐子) 偵測 + Pose ✅

| 項目 | 結果 |
|------|------|
| Detect bbox | 100% (7/7) |
| Pose 6 關鍵點 | 正確 (top, cap, left, right, center, bottom) |
| 模型版本 | can_v3 (Detect), can_pose_v1 (Pose) |
| 訓練方法 | SAM 切割 43 張 cutout → 合成 900 張 → 訓練 |
| Jetson 部署 | TensorRT engine 已匯出 |
| 迭代次數 | v1→v2→v3 (3 次，解決誤報和上下分框問題) |

### eHand (靈巧手) 偵測 ✅

| 項目 | 結果 |
|------|------|
| Detect bbox | 98-100% |
| 模型版本 | multiclass_detect_v2 |
| 訓練方法 | SAM 切割 32 張 cutout → 合成 + 真實背景 |
| 迭代次數 | detect v1→v2 (比例修正後 100%) |

### eHand Pose 關鍵點 ⚠️ 進行中

| 項目 | 結果 |
|------|------|
| 關鍵點出現率 | 97-98% |
| **關鍵點精度** | **❌ 不準（核心問題）** |
| 唯一準確的幀 | open_028 (model1b 灰色化版) |
| 迭代次數 | v2→v3→v4→v5→v6→v7→v8→v9→v9b→v10→Isaac→Isaac_mix→flow (13 次) |

---

## 嘗試過的方法與結論

| # | 方法 | eHand 偵測 | 關鍵點 | 結論 |
|---|------|-----------|--------|------|
| v2 | SAM + convex hull 6kpt | 96% | 有但不準 | convex hull 猜指尖不精確 |
| v3 | detect bbox → SAM + hull | 62% | 有但不準 | 偵測率反降 |
| v4 | 擴充背景 + 4kpt | 93% | 有 | 4 點比 6 點穩定但資訊少 |
| v5 | 6kpt + 擴充背景 | 96% | 有但不準 | 最佳偵測率的 6kpt 版 |
| v6 | URDF 純渲染 (PyBullet) | **0%** | - | Sim-to-Real gap 太大 |
| v7 | URDF + SAM 混合 | 98% | 有但不準 | URDF 和 SAM kpt 矛盾 |
| v8 | pyrender + SAM 混合 | 96% | 有但不準 | 同上 |
| v9 | 灰色化圖(kpt=0) + URDF | 96% | **0%** | visibility=0 壓制輸出 |
| v9b | 灰色化圖(kpt=hull) + URDF | 88% | **75%** | 首次有方向對的 kpt |
| v10 | 灰色域→回算→彩色域 | 97% | **97%** | 方向對但精度不足 |
| Isaac pure | Isaac Sim URDF 渲染 | **0%** | 79% | 全分類為 can |
| Isaac+SAM | Isaac + SAM 混合 | 98% | 98% | 偵測好但 kpt 不準 |
| **flow v1** | **光流追蹤 + URDF + SAM** | **83%** | **96% kpt** | **待驗收關鍵點精度（圖片）** |
| 16kpt_v1 | URDF 16pt 渲染 + 手動標註 28 張 | 64.4% box | **0% pose** | URDF domain gap 太大，ehand pose 完全不準 |
| **16kpt_sam_v1** | **SAM cutout + 手動 16pt 標註合成** | **71.8% box** | **18.7% pose** | 偵測 29/29，pose 精度待改善 |

---

## 關鍵踩坑

| 踩坑 | 解法 |
|------|------|
| 影片截取比例錯 (1280x720 vs 540x960) | 保持原始 9:16 比例 |
| SAM auto mode 選最大 mask | 改用中心點提示 + 面積過濾 |
| PyBullet 渲染色調太假 | 改用 SAM 切割 / Isaac Sim |
| Agent 合併時 kpt_shape 錯 (4 vs 6) | 驗證 data.yaml 格式 |
| 程式化背景 vs 真實背景 | 必須用 inpaint 真實背景 |
| ByteTrack + Pose 崩潰 | 改用 BoT-SORT + try/except |
| TensorRT engine 不帶 task | YOLO(path, task="pose") |
| Jetson torch CPU 版 | 用 nvidia.box.com wheel |
| torchvision 不相容 | --no-build-isolation 源碼建置 |
| Isaac Sim mesh instanceable | prim.SetInstanceable(False) |
| Isaac Sim RTX 5080 相機 | 放大 robot 10x 填充預設相機 |
| FoundationPose Docker CUDA 太舊 | 需要重建 CUDA 12.4 映像 (進行中) |

---

## 環境

### 5080 (訓練/渲染)
- SSH: `ssh IDAKA_5080`
- RTX 5080 16GB, i7-14700, 64GB RAM
- conda env: yolo (PyTorch 2.10+cu128), isaaclab (Isaac Sim 5.1)
- Docker: foundationpose-cu124 (建置中)

### Jetson Orin Nano (部署)
- SSH: `ssh IDAKA_ROBOT`
- PyTorch 2.3.0 (CUDA 12.4, aarch64)
- TensorRT 10.3, ultralytics 8.4.22
- RealSense D435i
- Web Viewer: `http://192.168.0.15:8081/`

### Jetson 已部署模型
```
~/ros2_ws/yolo_model/
├── multiclass_detect_v2.engine   ← Detect: Can 100% + eHand 100% (最穩)
├── multiclass_pose_v5.engine     ← Pose: 目前部署版
├── can_v3_best.engine            ← Can only
└── realsense_yolo_viewer.py → ~/realsense/
```

---

## 2026-03-22 進度更新

### 16 點 keypoint 方案確定
- **16 點** (index 0-15)：palm_center → thumb (mcp/ip/tip) → index → middle → ring → pinky
- **沒有 thumb_base**（原文件寫 17 點但實際標 16 個）
- URDF 手掌朝鏡頭 = Z 軸轉 -90° (`euler [0, 0, -pi/2]`)

### 手動標註工具 + 標註完成
- 建了 Flask web 標註工具（cloudflared 外部存取）
- 28 張 source_frames_open 全部標完
  - 完整 16 點：17 張
  - 部分點 (3-15 點，手指被遮擋)：11 張
- 標註位置：`~/openarm_yolo_training/web/annotations/*.json`

### 訓練嘗試

**16kpt_v1 (URDF 合成)**
- URDF 渲染 800+100 張 + 手動 28 張 + can 1000 張
- Box mAP50: 0.644 / Pose mAP50: 0.338
- ehand pose mAP = 0 → URDF domain gap 太大，完全污染
- 結論：❌ 不用 URDF

**16kpt_sam_v1 (SAM cutout 合成)** ← 目前最新
- SAM 從 28 張真實圖切出 26 張 ehand cutout
- cutout + 16pt keypoint 貼到隨機背景合成 776+98 張
- 合併 can 1000+100 張 → 總計 1776 train / 198 val
- Box mAP50: **0.718** / Pose mAP50: **0.187**
- 真實圖偵測 29/29 (100%)
- 結論：偵測好轉，pose 精度待改善

### 檔案位置
- 16pt cutout：`~/openarm_yolo_training/hand/cutouts_16kpt/` (26 張 + keypoints JSON)
- SAM 資料集：`~/openarm_yolo_training/datasets/ehand_16kpt_sam_v1/`
- SAM 模型：`~/openarm_yolo_training/runs/pose/ehand_16kpt_sam_v1/weights/best.pt`
- URDF 資料集：`~/openarm_yolo_training/datasets/ehand_16kpt_v1/`
- URDF 模型：`~/openarm_yolo_training/runs/pose/ehand_16kpt_v1/weights/best.pt`
- URDF 渲染腳本 (16pt)：`~/openarm_yolo_training/hand/scripts/render_ehand_16kpt.py`

---

## 待辦

1. 🔧 改善 SAM 合成時 keypoint 座標變換（有溢出 bug）→ 重練
2. 📱 標更多真實圖（不同手勢/角度）增加 cutout 多樣性
3. 🖼️ 把手動標的 28 張原圖也直接加進訓練集
4. 🔧 FoundationPose Docker 建置 (CUDA 12.4)
5. 🎥 RealSense bag 截取影格加入訓練

---

## 文件索引

| 文件 | 內容 |
|------|------|
| `yolo-training-setup.md` | YOLO 環境 + Can 訓練 + Jetson 部署 |
| `ehand-pose-training.md` | eHand Pose 技術文件 + URDF 結構 |
| `agent-workflow-sim2real.md` | Multi-Agent 工作流 + 迭代記錄 |
| `ehand-pose-roadmap.md` | Pose 改善路線圖 (6 條路線) |
| `pending_review.md` | 待驗收圖片清單 |
| `environment.md` | 5080 環境說明 (放在 5080 ~/environment.md) |
