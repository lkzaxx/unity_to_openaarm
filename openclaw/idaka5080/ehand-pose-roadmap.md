# eHand Pose 關鍵點精度改善路線圖

> 目標：在真實影片/RealSense 上精確追蹤 eHand 16 個關鍵點
> 前提：bbox 偵測已解決 (98%)，關鍵點精度是唯一瓶頸
> 最後更新：2026-03-21

---

## 目前最佳成果

| 項目 | 結果 |
|------|------|
| Detect bbox | Can 100% + eHand 98% |
| Pose 關鍵點出現率 | 97-98% |
| **Pose 關鍵點精度** | **❌ 不準（核心問題）** |
| 唯一準的幀 | open_028 (model1b 灰色化版) |

---

## 路線排序（成功率由高到低）

### 路線 1：光流追蹤擴展 ⭐⭐⭐⭐⭐ （已有初步成果）

```
成功率: 最高
原因: 已從 open_028 追蹤出 59 張，零手動標註
狀態: 已完成初步測試
```

**做法：**
1. 從 open_028 (唯一準的) 用光流追蹤到相鄰幀 → 已得到 59 張
2. 用這 59 張訓練 pose 模型
3. 用訓練好的模型推論其他影片 → 找出新的「準確幀」
4. 用新的準確幀當 seed 繼續光流追蹤 → 更多標註
5. 迭代

**優點：**
- 零手動標註
- 基於真實影片，不需要渲染
- 已驗證可行（59 張）

**風險：**
- 光流會漂移（離 seed 越遠越不準）
- 只能追蹤姿態變化小的連續幀
- seed 本身的 pose 點如果有微小誤差，會累積

**待驗證：** 59 張的追蹤品質（在 optical_flow_dense/ 目錄）

---

### 路線 2：灰色化真實圖 + URDF 回算 (v10 方案改良) ⭐⭐⭐⭐

```
成功率: 高
原因: v10 已達 97% 關鍵點出現率，方向對但位置不夠精確
狀態: 已多次迭代 (v9, v9b, v10)，需要改善灰色化品質
```

**做法：**
1. 改善灰色化（85% 灰 + 更準的 SAM mask）
2. 調整 URDF 顏色匹配（75% 設定 pixel~119）
3. 灰色域訓練 → 回算 → 彩色域訓練
4. 結合光流追蹤的資料一起訓練

**改良點：**
- 用 Isaac Sim OmniPBR 渲染取代 PyBullet（品質更好）
- 用光流追蹤的 59 張作為彩色域的補充資料
- URDF 資料增加到 3000+ 張

---

### 路線 3：材質貼圖到 URDF ⭐⭐⭐

```
成功率: 中
原因: 能讓渲染接近真實外觀，但需要解決 UV mapping
狀態: 未嘗試
```

**做法：**
1. 從 SAM cutout 取 eHand 的真實紋理
2. 投影貼圖到 STL mesh（不需要 UV，用 projective mapping）
3. 或在 Isaac Sim 中設定多材質（不只 3 個 material，手動分區域）
4. 渲染 + 精確關鍵點 → 訓練

**實作方式：**
- Isaac Sim: 把 cutout 當 texture 投影到 mesh
- 或 Blender: 匯入 STL → UV unwrap → 貼圖 → 匯出 → Isaac/PyBullet 渲染
- 或簡單版: SAM cutout + URDF 骨架對齊 → 直接合成

---

### 路線 4：FoundationPose 6DoF → 關鍵點投影 ⭐⭐⭐

```
成功率: 中（技術可行但安裝困難）
原因: 理論上最精確，但需要 RGB-D 且 Docker 環境有問題
狀態: Docker 建置失敗（RTX 5080 CUDA 不相容）
```

**做法：**
1. 解決 Docker CUDA 問題（或用 conda 安裝）
2. 用 RealSense bag (RGB-D) 餵 FoundationPose
3. 得到 6DoF pose → URDF 投影 16 個精確關鍵點
4. 自動標註大量真實幀

**阻礙：**
- nvdiffrast 編譯失敗（RTX 5080 sm_120）
- Docker 內 CUDA 11.3 太舊
- 需要 RGB-D 資料（手機影片沒有深度）
- 可能等 Isaac Sim 6.0 或 FoundationPose 更新才能解決

---

### 路線 5：微分渲染 pose fitting ⭐⭐

```
成功率: 中低
原因: 需要可微分渲染庫，安裝困難
狀態: silhouette 比對得到 IoU 59.6%，但庫裝不上
```

**做法：**
1. 安裝 PyTorch3D 或 nvdiffrast（需要解決 RTX 5080 相容性）
2. 把 URDF mesh 渲染的 silhouette 跟 SAM mask 比對
3. 梯度下降優化 6DoF pose + 關節角度
4. 收斂後投影關鍵點

**阻礙：**
- PyTorch3D、nvdiffrast 都裝不上
- 簡化版 (PyBullet silhouette) IoU 只有 59.6%

---

### 路線 6：手動標註 5-10 張 ⭐⭐⭐⭐

```
成功率: 最高（但需要人工）
原因: 直接提供精確標註，最可靠
狀態: 工具已準備好 (label_ehand_pose.py)，用戶目前無法操作
```

**做法：**
1. 用 label_ehand_pose.py 標 5-10 張張開手的幀
2. 結合 URDF 渲染 + SAM 合成 → 訓練
3. 光流追蹤擴展更多幀

**備註：** 用戶目前手邊沒電腦，暫時無法操作。等有空時再做。

---

### 路線 7：已知姿態 + RealSense 截圖配對 ⭐⭐⭐⭐⭐

```
成功率: 非常高
原因: 控制 eHand 做指定動作 → 已知關節角度 → URDF 算出精確關鍵點 → 同時 RealSense 截圖
狀態: 未嘗試，需要 eHand CAN FD 控制 + RealSense 同步截圖
```

**做法：**
1. 透過 CAN FD 控制 eHand 做特定姿態（張開、握拳、捏取等）
2. 發送指令時記錄每個關節的角度值
3. 同時用 RealSense 截取 RGB-D 圖
4. 用已知關節角度 + URDF 正向運動學 → 計算 16 個精確 3D 關鍵點
5. 用 RealSense 相機內參 → 投影到 2D = 精確標註
6. 需要手眼標定（camera-to-base transform）

**優點：**
- 真實照片 + 精確關鍵點（不是猜的）
- 可自動化：程式控制手勢 → 自動截圖 → 自動標註
- 任意數量的姿態（只要 eHand 能做的動作）

**需要：**
- eHand CAN FD 控制可用
- RealSense 同步截圖
- 手眼標定（一次性）
- 知道 URDF 關節角度跟 CAN FD 指令的映射

---

## 執行順序

```
現在:
  路線 1 (光流追蹤) → 用 59 張追蹤資料訓練看效果
    ├── 效果好 → 繼續迭代擴展
    └── 效果不好 → 路線 2 (灰色化+URDF改良)

之後:
  路線 3 (材質貼圖) → 研究 Isaac Sim 投影貼圖
  路線 4 (FoundationPose) → 等 CUDA 相容性解決
  路線 6 (手動標註) → 用戶有空時補充

長期:
  路線 5 (微分渲染) → 等庫更新支援 sm_120
```

---

## 所有測試結果位置

```
~/openarm_yolo_training/
├── hand/optical_flow/          ← 光流追蹤 (截取幀, 6 張)
├── hand/optical_flow_dense/    ← 光流追蹤 (原始影片, 59 張) ⭐
├── hand/diffrender/results/    ← 微分渲染 silhouette 比對
├── hand/foundationpose/        ← FoundationPose (Docker 失敗)
├── hand/isaac_sim/renders/     ← Isaac Sim 渲染 (19_omnipbr.jpg 最好)
├── test/gray/model1b_85pct/    ← 灰色化 + model1b (open_028 準的)
├── test/gray/urdf_pose_16pt/   ← URDF 16 點 pose demo (準的)
└── test_results/pose_*         ← 各版本 pose 測試結果
```
