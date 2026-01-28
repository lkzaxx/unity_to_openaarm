# Joint Data Logger & Plotter

用於記錄和比較 **Unity 原始目標位置** 與 **OpenArm 實際位置** 的工具模組。

## 目錄結構

```
scripts/utils/
├── __init__.py              # 模組初始化
├── joint_data_logger.py     # 數據記錄模組
├── plot_joint_comparison.py # 繪圖腳本
├── README.md                # 本說明文件
└── pictures/                # 圖片輸出目錄
    └── *.jpg                # 產生的比較圖
```

## 功能說明

### 1. 數據記錄 (JointDataLogger)

#### 系統頻率與週期

| 項目 | 頻率 | 週期 | 說明 |
|------|------|------|------|
| Unity 發送 | ~60 Hz | ~16.67 ms | Unity 傳送關節目標的頻率 |
| OpenArm 控制 | 500 Hz | 2 ms | MIT 控制命令發送給馬達的頻率 |
| 數據記錄 | 50 Hz | 20 ms | 每 10 次控制迴圈記錄一次 |
| 最大記錄時長 | - | 60 秒 | 超過後自動停止記錄 |

> 注意：
> - Unity 的 60 Hz 是近似值，實際可能因網路延遲而有變化
> - 記錄頻率可在 `unity_interface_follower.py` 中調整 `LOG_FREQUENCY` 參數（預設 50 Hz）

#### 記錄的數據

- **Unity 原始目標** (`left_unity_target`, `right_unity_target`): Unity 傳入的關節目標位置
- **OpenArm 實際位置** (`left_actual`, `right_actual`): 馬達回傳的實際位置
- **時間戳** (`timestamp`): 相對時間 (秒)

### 2. 繪圖 (plot_joint_comparison.py)

為每個關節產生獨立的比較圖：
- X 軸：時間 (秒)
- Y 軸：關節位置 (rad)
- 藍線：Unity 原始目標
- 紅線：OpenArm 實際位置
- 綠色區域：追蹤誤差

## 使用方式

### 在 unity_interface_follower.py 中的設定

記錄功能已整合到 `unity_interface_follower.py`，相關程式碼位於：

1. **導入模組** (約第 21-26 行)：
```python
# ===== [JOINT_LOGGER] 關節數據記錄 - 開始 =====
ENABLE_JOINT_LOGGING = True  # 設為 False 可停用
# ===== [JOINT_LOGGER] 關節數據記錄 - 結束 =====
```

2. **初始化** (`__init__` 方法內)：
```python
# ===== [JOINT_LOGGER] 初始化 - 開始 =====
# ...初始化程式碼...
# ===== [JOINT_LOGGER] 初始化 - 結束 =====
```

3. **記錄數據** (`control_loop` 方法內，約在 `recv_all()` 之後)：
```python
# ===== [JOINT_LOGGER] 記錄數據 - 開始 =====
# ...記錄程式碼...
# ===== [JOINT_LOGGER] 記錄數據 - 結束 =====
```

4. **儲存數據** (`shutdown` 方法內)：
```python
# ===== [JOINT_LOGGER] 儲存數據 - 開始 =====
# ...儲存程式碼...
# ===== [JOINT_LOGGER] 儲存數據 - 結束 =====
```

### 停用記錄功能

如果不需要記錄功能，只需將 `ENABLE_JOINT_LOGGING` 設為 `False`：

```python
ENABLE_JOINT_LOGGING = False
```

或者，可以完全移除所有標記為 `[JOINT_LOGGER]` 的程式碼區塊。

### 手動開始/停止記錄

記錄器預設在 Unity 連線後自動開始。也可以手動控制：

```python
# 開始記錄
node.joint_logger.start()

# 停止記錄
node.joint_logger.stop()

# 儲存數據
node.joint_logger.save()
```

## 繪製圖表

### 基本使用

```bash
cd /home/idaka/ros2_ws/src/openarm_ros2/openarm_bringup/scripts/utils

# 繪製所有關節圖（左右手）
python3 plot_joint_comparison.py joint_data_20260120_143000.pkl

# 只繪製左手
python3 plot_joint_comparison.py joint_data_20260120_143000.pkl --arm left

# 只繪製右手
python3 plot_joint_comparison.py joint_data_20260120_143000.pkl --arm right

# 額外產生組合圖（7 個關節在同一張圖）
python3 plot_joint_comparison.py joint_data_20260120_143000.pkl --combined
```

### 輸出檔案命名格式

```
{arm}_joint{1-7}_{YYYYMMDD_HHMMSS}.jpg
```

例如：
- `left_joint1_20260120_143000.jpg`
- `left_joint2_20260120_143000.jpg`
- ...
- `right_joint7_20260120_143000.jpg`

## 參數說明

### JointDataLogger 參數

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `log_frequency` | 50 | 記錄頻率 (Hz) |
| `control_frequency` | 500 | 控制迴圈頻率 (Hz) |
| `max_duration` | 60.0 | 最大記錄時長 (秒) |
| `save_dir` | `utils/` | 數據儲存目錄 |

### plot_joint_comparison.py 參數

| 參數 | 說明 |
|------|------|
| `data_file` | 數據檔案路徑 (.pkl) |
| `--arm, -a` | 繪製的手臂 (left/right/both) |
| `--output, -o` | 輸出目錄 |
| `--combined, -c` | 額外產生組合圖 |
| `--no-error` | 不顯示追蹤誤差 |

## 數據格式

儲存的 `.pkl` 檔案結構：

```python
{
    'timestamp': [0.0, 0.02, 0.04, ...],  # 時間序列 (秒)
    'left_unity_target': [
        [j1_t0, j1_t1, ...],  # Joint 1
        [j2_t0, j2_t1, ...],  # Joint 2
        ...                    # Joint 3-7
    ],
    'left_actual': [...],      # 同上格式
    'right_unity_target': [...],
    'right_actual': [...]
}
```

## 疑難排解

### 1. 找不到模組

確保從正確的目錄執行，或將 `scripts/` 加入 Python 路徑：

```bash
export PYTHONPATH=$PYTHONPATH:/home/idaka/ros2_ws/src/openarm_ros2/openarm_bringup/scripts
```

### 2. 圖片無法顯示（無 GUI 環境）

繪圖腳本已設定使用 `Agg` backend，會直接輸出檔案而非顯示視窗。

### 3. 記錄數據為空

檢查：
1. `ENABLE_JOINT_LOGGING` 是否為 `True`
2. Unity 是否已連線（記錄在連線後自動開始）
3. 是否有呼叫 `joint_logger.start()`

## 相關檔案位置

- 主程式：`/home/idaka/ros2_ws/src/openarm_ros2/openarm_bringup/scripts/unity_interface_follower.py`
- 工具目錄：`/home/idaka/ros2_ws/src/openarm_ros2/openarm_bringup/scripts/utils/`
- 圖片目錄：`/home/idaka/ros2_ws/src/openarm_ros2/openarm_bringup/scripts/utils/pictures/`
