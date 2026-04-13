# OpenArm VLA Fine-Tune Pipeline

## 目標

訓練一個 VLA 模型（SmolVLA / π0.5），讓 OpenArm 能自主完成：
**「用夾爪把桌上的東西夾到旁邊的箱子」**

---

## 0. 運行架構

### 雙終端模式

```
Terminal 1 (Jetson):  ./start_follower.sh          ← 啟動手臂控制
Terminal 2 (Jetson):  python3 waypoint_collector.py ← 自動移動 + 收資料
```

`start_follower.sh` 啟動後提供：
- 500Hz MIT 控制迴圈（馬達即時控制）
- Ruckig 軌跡平滑（自動把離散目標插值成平滑動作）
- 相機發布（IMX219 雙目）
- 關節狀態回讀（50Hz）

`waypoint_collector.py` 只需做：
- 以 20Hz 發布目標角度 → follower 的 Ruckig 自動平滑
- 訂閱關節狀態 + 相機影像 → 錄成 episode

### 通訊拓撲

```
waypoint_collector.py                      start_follower.sh
─────────────────────                      ──────────────────
                                           (unity_interface_follower.py)

publish 目標角度 ───→ /unity/joint_commands ───→ Ruckig 平滑 → 500Hz MIT → 馬達
subscribe 狀態   ←── /openarm/joint_states ←── 馬達回傳 (50Hz)
subscribe 影像   ←── /camera/color/compressed ← RealSense D435i (30Hz)
subscribe 深度   ←── /camera/depth/compressed ← RealSense D435i (30Hz)
publish 夾爪     ───→ /unity/ehand_commands ──→ 夾爪控制 (30Hz)
       │
       ↓
  儲存 episode
  (data.npz + exterior.mp4)
```

### ROS2 Topics 規格

| Topic | 方向 | 型別 | QoS | 頻率 |
|-------|------|------|-----|------|
| `/unity/joint_commands` | → follower | `sensor_msgs/JointState` | VOLATILE, RELIABLE | 20Hz（腳本發） |
| `/openarm/joint_states` | ← follower | `sensor_msgs/JointState` | default | 50Hz |
| `/camera/color/compressed` | ← camera | `sensor_msgs/CompressedImage` | RELIABLE | 30Hz |
| `/camera/depth/compressed` | ← camera | `sensor_msgs/CompressedImage` | RELIABLE | 30Hz |
| `/camera/left/compressed` | ← camera | `sensor_msgs/CompressedImage` | RELIABLE | 30Hz (向後相容，與 color 相同) |
| `/unity/ehand_commands` | → follower | `sensor_msgs/JointState` | default | 按需 |

### 連續移動原理

follower 內建 Ruckig 軌跡產生器，**不需要自己做 500Hz 插值**：

```
你的腳本（20Hz）：    ●──────●──────●──────●    (稀疏 waypoints)
                       ↓
Ruckig 自動平滑：     ╱‾‾‾‾‾‾‾╲___╱‾‾‾‾‾‾╲    (S-curve, 500Hz 輸出)
                       ↓
馬達實際運動：        ≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈    (平滑連續)
```

Ruckig 參數（已在 follower 中設定）：
- 最大速度：J1-J2 14 rad/s, J3-J4 5 rad/s, J5-J7 18 rad/s
- 最大加速度：J1-J2 48 rad/s², J3-J4 24 rad/s², J5-J7 72 rad/s²
- 最大 jerk：J1-J2 240 rad/s³, J3-J4 120 rad/s³, J5-J7 360 rad/s³

### 指令格式參考

```python
# 移動右臂（7 個關節角度，單位 rad）
msg = JointState()
msg.name = ['R_J1', 'R_J2', 'R_J3', 'R_J4', 'R_J5', 'R_J6', 'R_J7']
msg.position = [0.13, 0.05, 0.01, 0.61, 0.03, 0.0, 1.20]
pub.publish(msg)

# 夾爪控制（0=全閉, 1=全開）
grip_msg = JointState()
grip_msg.name = ['R_F1', 'R_F2', 'R_F3', 'R_F4', 'R_F5', 'R_F6']
grip_msg.position = [0.5] * 6  # 半開
grip_pub.publish(grip_msg)

# 特殊指令
msg.name = ['HOME']       # 雙臂歸零
msg.name = ['R_ENABLE']   # 右臂啟用
```

---

## 1. 系統現狀

### 已有的基礎設施

| 元件 | 狀態 | 位置 |
|------|------|------|
| ROS2 控制節點 | ✅ 可用 | `unity_to_openaarm/ros2_ws/` |
| 關節指令發送 | ✅ `arm_cmd.py` | 發 `/unity/joint_commands` 即可動 |
| 夾爪控制 | ✅ `ehand_cmd.py` | 支援 open/close/grip 0~1 |
| 相機 | ✅ IMX219 雙目 + D435i | Jetson 上 `/camera/left/compressed` 等 |
| 關節狀態回讀 | ✅ 50Hz | `/openarm/joint_states` |
| 增量移動+拍照 | ✅ `move_and_capture.py` | 已有範例 |
| SmolVLA 訓練腳本 | ✅ 全套 | `VLA/SmolVLA/openarm_config/` |
| LeRobot 轉換 | ✅ | `convert_to_lerobot.py` |
| MoveIt2 | ✅ 已配置 | `openarm_bimanual_moveit_config/` |
| GPU 訓練機 | ✅ RTX 5080 16GB | `192.168.0.245` |

### 問題：VR 遙操作不夠準確

VR (Quest 3) 目前用於遙操作，但精度不足以穩定完成「夾取→放置」，
導致收集到的 demonstration 品質差、成功率低。

---

## 2. 資料收集策略（取代 VR）

### 方案比較

| 方案 | 精度 | 效率 | 難度 | 推薦度 |
|------|------|------|------|--------|
| A. 腳本式 waypoint 軌跡 | ★★★★★ | ★★★★★ | ★★ | **首選** |
| B. 視覺引導腳本 (YOLO) | ★★★★ | ★★★★ | ★★★ | Phase 2 |
| C. Kinesthetic 手動導引 | ★★★★ | ★★ | ★★ | 備選 |
| D. MoveIt 路徑規劃 | ★★★★ | ★★★ | ★★★★ | 備選 |
| E. VR + 腳本修正 | ★★★ | ★★ | ★★★ | 不推薦 |

### 方案 A（首選）：腳本式 Waypoint 軌跡 + 隨機化

**核心思路**：手動標定幾個關鍵姿態（approach → grasp → lift → transport → release），
程式在關鍵點之間做插值並加入隨機擾動，自動產生大量 episodes。

```
流程：
  HOME → approach (物體上方) → pre-grasp (物體旁) → grasp (閉合夾爪)
    → lift (抬起) → transport (移到箱子上方) → release (張開夾爪) → HOME

每個 episode 加入的隨機化：
  - 物體位置偏移 ±2-3cm（對應 approach/grasp waypoint 微調）
  - 抬升高度 ±1cm
  - 移動速度 ±20%
  - 夾爪閉合力度微調
```

**優勢**：
- 精度 100%（腳本控制，每次都成功）
- 效率極高（1 episode ≈ 10-15 秒，自動化可連續跑）
- 搭配相機同步錄製，直接產出訓練資料
- **1 小時可收集 200+ episodes**

**關鍵檔案**（待開發）：
```
finetune_pipeline/
├── README.md                    # 本文件
├── waypoint_collector.py        # 腳本式資料收集（核心）
├── waypoints/
│   └── pick_and_place.yaml      # 預定義 waypoint 組
├── utils/
│   ├── trajectory_interpolator.py   # waypoint 間插值
│   └── randomizer.py               # 隨機擾動產生器
└── configs/
    └── collect_config.yaml      # 收集參數設定
```

### 方案 B（Phase 2）：YOLO 視覺引導

用 YOLO 偵測桌上物體位置 → 計算 IK 目標 → 自動產生 grasp waypoint。
這能覆蓋更多物體位置變化，但依賴：
- 相機標定（hand-eye calibration）
- 目標物體的 YOLO 模型（你已有罐子的 YOLO 模型）
- 座標轉換（pixel → robot frame）

### 方案 C（備選）：Kinesthetic 手動導引

設定 KP=0, KD=小值（阻力極低），手動移動手臂同時錄製軌跡。
OpenArm 的 MIT 控制模式理論上支持，但：
- 需確認馬達的 backdrivability（DM8009/DM4340 可能較難推）
- 效率低（每 episode 需人工操作）
- 適合作為補充資料，不適合批量生成

---

## 3. 訓練路線

### Phase 1：SmolVLA Fine-Tune（推薦先做）

```
資料收集 (waypoint_collector.py)
    ↓ 30 Hz, 8-dim state + 2 cameras
    ↓ raw episodes (npz + mp4)
convert_to_lerobot.py
    ↓ LeRobot Dataset (Parquet + MP4)
train.sh (SmolVLA fine-tune)
    ↓ RTX 5080, batch=32, 20K steps
    ↓ ~2-4 hours
inference_server.py
    ↓ 部署到實體手臂
評估 → 失敗分析 → 補充資料 → 重新訓練
```

**資料量目標**：
- 最低：50 episodes（基本能動）
- 建議：100-200 episodes（穩定執行）
- 理想：500+ episodes（泛化到不同物體）

**每 episode 約 5-15 秒 → 100 episodes ≈ 30-60 分鐘自動收集**

### Phase 2：ACT / Diffusion Policy（如 SmolVLA 不夠好）

- 非 VLA（無語言輸入），但 pick-and-place 這種固定任務可能更穩定
- LeRobot 原生支持 ACT 訓練
- 同樣的 LeRobot Dataset 可直接使用

### Phase 3：π0.5（長期，需更大 GPU）

- LoRA fine-tune 需 ≥24GB VRAM
- 需 250+ episodes
- 可用雲端 A100 或未來升級 GPU

---

## 4. 實施步驟（詳細）

### Step 1: 標定 Waypoints（手動，一次性）

在 Jetson 上用 `arm_cmd.py` 逐步試出 pick-and-place 的關鍵姿態：

```bash
# 1. 啟動手臂
python3 arm_cmd.py ENABLE

# 2. 逐步調整到物體上方（approach pose）
python3 arm_cmd.py R 0.13 0.05 0.01 0.61 0.03 0 1.20

# 3. 記錄下每個關鍵姿態的 7 個關節角度
# 4. 寫入 waypoints/pick_and_place.yaml
```

需要標定的姿態（約 6-8 個）：
1. `home` — 安全起始位
2. `approach` — 物體正上方（高）
3. `pre_grasp` — 物體旁（低，夾爪張開）
4. `grasp` — 夾住（夾爪閉合）
5. `lift` — 抬起
6. `transport` — 移到箱子上方
7. `release` — 張開夾爪
8. `retreat` — 退回安全位

### Step 2: 開發 waypoint_collector.py

功能：
- 讀取 waypoint YAML
- waypoint 之間做線性/三次插值（30Hz 輸出）
- 對每個 waypoint 加入可配置的隨機擾動
- 同步錄製 joint states + camera images
- 自動循環執行 N 個 episodes
- 輸出格式與 `collect_data.py` 相容（npz + mp4）

### Step 3: 收集 100+ Episodes

```bash
# 在 Jetson 上執行（ROS2 + camera 已啟動）
python3 waypoint_collector.py \
  --waypoints waypoints/pick_and_place.yaml \
  --episodes 100 \
  --randomize \
  --output-dir ~/datasets/pick_and_place/raw \
  --task "pick up the object from the table and place it in the box"
```

預估時間：每 episode 15 秒 + 3 秒間隔 = **100 episodes ≈ 30 分鐘**

### Step 4: 轉換為 LeRobot Dataset

```bash
# 在訓練機 (192.168.0.245) 上
python convert_to_lerobot.py \
  --input-dir ~/datasets/pick_and_place/raw \
  --output-dir ~/datasets/pick_and_place/lerobot \
  --fps 30
```

### Step 5: SmolVLA Fine-Tune

```bash
# 在訓練機上
conda activate vla
bash train.sh \
  --dataset ~/datasets/pick_and_place/lerobot \
  --batch-size 32 \
  --steps 20000 \
  --output ~/models/openarm_pick_place_v1
```

### Step 6: 部署與評估

```bash
# 在 Jetson 上啟動推論伺服器（或用訓練機遠端推論）
python inference_server.py \
  --model ~/models/openarm_pick_place_v1 \
  --task "pick up the object from the table and place it in the box"
```

### Step 7: 迭代

```
評估結果 → 分析失敗模式：
├── 抓不到  → 增加 grasp 位置變化的訓練資料
├── 掉落    → 增加不同重量/大小物體的資料
├── 放不準  → 增加 release 位置的隨機化
└── 泛化差  → 增加物體種類、光照變化
```

---

## 5. 關鍵注意事項

### 資料品質 > 資料量
- 腳本式收集的優勢：**每個 episode 都是成功的 demonstration**
- VR 收集的問題：失敗的 episode 會汙染訓練資料
- 建議：只保留夾取成功的 episodes

### 隨機化的範圍
- 太小：模型只學會固定動作，無法泛化
- 太大：動作不自然，可能超出安全範圍
- 建議起步：位置 ±2cm，角度 ±5°，速度 ±20%

### 相機一致性
- 訓練時的相機角度 = 推論時的相機角度（極為重要）
- 相機移動 5° 可能導致成功率從 95% 降到 5%
- 建議固定相機支架，並標記位置

### 安全
- 所有 waypoint 必須在 joint limits 內（`openarm_env.py` 定義）
- 加入速度限制（`check_velocity_limit`）
- 測試時用低速（50% max velocity）

---

## 6. 檔案結構

```
VLA/finetune_pipeline/
├── README.md                        # 本文件
├── waypoint_collector.py            # [待開發] 核心收集腳本
├── waypoints/
│   └── pick_and_place.yaml          # [待標定] 關鍵姿態定義
├── configs/
│   └── collect_config.yaml          # [待開發] 收集參數
├── utils/
│   ├── trajectory_interpolator.py   # [待開發] 軌跡插值
│   └── randomizer.py               # [待開發] 隨機擾動
├── scripts/
│   ├── calibrate_waypoints.py       # [待開發] 互動式 waypoint 標定工具
│   └── validate_episode.py          # [待開發] episode 品質驗證
└── docs/
    └── troubleshooting.md           # [待寫] 常見問題
```

## 7. 依賴關係

```
現有系統（不需修改）：
  ├── arm_cmd.py          → 發送關節指令
  ├── ehand_cmd.py        → 控制夾爪
  ├── openarm_env.py      → 關節定義、歸一化
  ├── collect_data.py     → 錄製格式參考
  ├── convert_to_lerobot.py → 資料轉換
  └── train.sh            → SmolVLA 訓練

需要開發：
  └── waypoint_collector.py  → 整合上述元件的自動收集腳本
```

---

## 8. 時間預估

| 階段 | 工作內容 | 時間 |
|------|---------|------|
| Step 1 | 標定 waypoints | 1-2 小時 |
| Step 2 | 開發 waypoint_collector.py | 1 天 |
| Step 3 | 收集 100 episodes | 30-60 分鐘 |
| Step 4 | 資料轉換 | 10 分鐘 |
| Step 5 | SmolVLA 訓練 | 2-4 小時 |
| Step 6 | 部署測試 | 1-2 小時 |
| Step 7 | 迭代 (×3-5 輪) | 每輪 1 天 |
| **總計** | **首次端到端** | **~3 天** |
