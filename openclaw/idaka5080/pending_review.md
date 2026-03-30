# 待驗收圖片清單

> 下次有電腦時逐一檢查這些結果
> 最後更新：2026-03-21

---

## 優先驗收（影響下一步方向）

### 1. 光流追蹤品質（路線 1 核心）
```
5080: ~/openarm_yolo_training/hand/optical_flow_dense/
  - SEED.jpg                    ← seed 幀（open_028 的 pose 點）
  - flow_XXXXXX.jpg (59 張)     ← 光流追蹤的結果
  - dense_tracked.json          ← 座標數據
```
**檢查重點：** pose 點是否跟著手指移動？離 seed 越遠的幀是否漂移？

### 2. 光流 + URDF + SAM 混合訓練結果
```
5080: ~/openarm_yolo_training/test_results/pose_flow_ehand/
  - 104 張真實幀的推論結果
```
**檢查重點：** 跟之前版本比，指尖關鍵點有沒有更準？

### 3. Isaac Sim OmniPBR 渲染
```
5080: ~/openarm_yolo_training/hand/isaac_sim/renders/19_omnipbr.jpg
本機: c:\code\vr_robot\openarm\idaka5080\video\hand\render_test\19_omnipbr.jpg
```
**檢查重點：** 金屬光澤是否接近真實 eHand？

---

## 次要驗收（已有初步結論）

### 4. 灰色化品質
```
5080: ~/openarm_yolo_training/test/gray/dataset_preview/     ← 59 張灰色化圖
5080: ~/openarm_yolo_training/test/gray/transparency_test/   ← 不同灰度對比
5080: ~/openarm_yolo_training/test/gray/model1b_85pct/       ← 85% 灰 + model1b 結果
```

### 5. URDF 16 點 pose demo
```
5080: ~/openarm_yolo_training/test/gray/urdf_pose_16pt/      ← 5 姿態 x 3 角度 = 15 張
5080: ~/openarm_yolo_training/test/gray/urdf_pose_demo/      ← 6 點版 demo
```

### 6. URDF 顏色測試
```
5080: ~/openarm_yolo_training/test/gray/urdf_color_test/     ← 不同灰度 URDF 渲染
```

### 7. silhouette 比對結果
```
5080: ~/openarm_yolo_training/hand/diffrender/results/silhouette_open028.jpg
```

---

## 歷代 Pose 測試結果（參考用）

```
5080: ~/openarm_yolo_training/test_results/
├── pose_v2_ehand/          ← v2 SAM convex hull
├── pose_v3_ehand/          ← v3 detect bbox→SAM
├── pose_v4_ehand/          ← v4 擴充背景 (4kpt, 93%)
├── pose_v5_ehand/          ← v5 6kpt 擴充背景 (96%)
├── pose_v7_ehand/          ← v7 URDF+SAM 混合 (98%)
├── pose_v8_ehand/          ← v8 pyrender+SAM
├── pose_v9b_ehand/         ← v9b 灰色化+URDF (kpt 75%)
├── pose_v10_ehand/         ← v10 灰色域→回算 (kpt 97%)
├── pose_isaac_ehand/       ← Isaac 純渲染 (0% class混淆)
├── pose_isaac_mix_ehand/   ← Isaac+SAM 混合 (98%)
└── pose_flow_ehand/        ← 光流追蹤版 (等結果) ⭐
```

---

## RealSense 截圖（待加入訓練）

```
本機: c:\code\vr_robot\openarm\idaka5080\debug\螢幕擷取畫面 2026-03-20 103013.png
  - can + eHand 同框的 RealSense 俯視圖
5080: ~/openarm_yolo_training/hand/source_video/realsense_hand.bag (5.9GB)
  - 待截取影格加入訓練
```
