# Unity-OpenArm 控制調參筆記

## 問題總覽

| 問題 | 原因 | 解決方案 | 狀態 |
|------|------|----------|------|
| 追蹤延遲 | Ruckig 平滑限制太嚴格 | 提高 Ruckig 參數 | ✅ 已解決 |
| 靜態震盪 | Kd 阻尼不足 | 增加 Kd | ✅ 已解決 |
| 「山峰」過衝 | 重力補償方向反轉 | 關閉重力補償 | ✅ 已解決 |
| 穩態誤差（下垂） | 無重力補償 + Kp 不足 | 提高 Kp/Kd | 🧪 測試中 |
| 追蹤與穩定矛盾 | 靜態 Kp/Kd 無法兼顧 | 動態 Kp/Kd（方案一） | 🧪 測試中 |

---

## 目前參數設定（2026-02-11）

```python
# ============== 動態 Kp/Kd 方案（方案一：誤差調整） ==============
USE_DYNAMIC_KP_KD = True  # 開啟動態 Kp/Kd

# [2026-02-11] 只對指定關節使用動態 Kp/Kd（J3-7 出現震盪）
DYNAMIC_KP_KD_JOINTS = [0, 1]  # 只對 J1, J2 使用動態調整

# 基礎 Kp/Kd（J1=130 改善動態追蹤）
BASE_KP = [130.0, 80.0, 40.0, 60.0, 30.0, 30.0, 50.0]
BASE_KD = [5.0, 4.5, 1.2, 1.5, 1.0, 1.0, 1.2]

# 動態調整參數（只對 DYNAMIC_KP_KD_JOINTS 中的關節生效）
LARGE_ERROR_THRESHOLD = 0.2   # 大誤差閾值 (rad) ≈ 11.5°
SMALL_ERROR_THRESHOLD = 0.05  # 小誤差閾值 (rad) ≈ 2.9°
LARGE_ERROR_KP_SCALE = 1.5    # 大誤差時 Kp 放大倍數
LARGE_ERROR_KD_SCALE = 0.8    # 大誤差時 Kd 縮小倍數（減少阻力加快追蹤）
SMALL_ERROR_KP_SCALE = 0.7    # 小誤差時 Kp 縮小倍數（避免震盪）
SMALL_ERROR_KD_SCALE = 1.2    # 小誤差時 Kd 放大倍數（增加穩定性）

# Ruckig - 關閉
USE_RUCKIG_SMOOTHING = False

# 重力補償 - 關閉
left_comp = [0.0] * 7
right_comp = [0.0] * 7
```

### 動態 Kp/Kd 策略說明

| 關節 | 動態調整 | 說明 |
|------|----------|------|
| J1, J2 | ✅ 啟用 | 肩部大關節，需快速追蹤 |
| J3-J7 | ❌ 停用 | 使用靜態 BASE_KP/KD，避免震盪 |

| 誤差範圍（J1, J2） | Kp | Kd | 目的 |
|----------|-----|-----|------|
| 大誤差 (>0.2 rad) | 基礎 × 1.5 | 基礎 × 0.8 | 高剛度快速追蹤 |
| 正常範圍 | 基礎值 | 基礎值 | 平衡追蹤與穩定 |
| 小誤差 (<0.05 rad) | 基礎 × 0.7 | 基礎 × 1.2 | 低剛度避免震盪 |

---

## 修改歷程

### 2026-02-11：提高 J1 基礎 Kp 改善動態追蹤

**問題描述**：Joint 1 紅線追蹤藍線時明顯延遲（見 `right_joint1_20260211_172348.jpg`）

**修改**：
```python
BASE_KP[0]: 100.0 → 130.0
```

---

### 2026-02-11：限制動態 Kp/Kd 只對 J1, J2 生效

**問題描述**：
- 動態 Kp/Kd 應用到所有關節後，J3-J7 出現震盪
- 見圖表：`right_all_joints_20260211_172358.jpg`

**解決方案**：新增 `DYNAMIC_KP_KD_JOINTS` 配置，只對指定關節使用動態調整

**修改**：
```python
# 只對 J1, J2 使用動態調整
DYNAMIC_KP_KD_JOINTS = [0, 1]

def _get_dynamic_kp_kd(self, joint_idx, position_error):
    # 非指定關節直接返回靜態值
    if joint_idx not in DYNAMIC_KP_KD_JOINTS:
        return BASE_KP[joint_idx], BASE_KD[joint_idx]
    # ... 動態計算邏輯 ...
```

---

### 2026-02-10：實現動態 Kp/Kd（方案一）

**問題描述**：
- 靜態 Kp/Kd 無法同時滿足「快速追蹤」和「穩態穩定」
- 大誤差時需要高 Kp 快速響應
- 小誤差時需要低 Kp 避免震盪

**解決方案**：根據誤差大小動態調整 Kp/Kd

**實現程式碼**（在 `unity_interface_follower.py`）：
```python
def _get_dynamic_kp_kd(self, joint_idx: int, position_error: float) -> tuple:
    """根據誤差大小動態調整 Kp/Kd"""
    abs_error = abs(position_error)
    
    if abs_error > LARGE_ERROR_THRESHOLD:      # > 0.2 rad
        kp = BASE_KP[joint_idx] * LARGE_ERROR_KP_SCALE  # × 1.5
        kd = BASE_KD[joint_idx] * LARGE_ERROR_KD_SCALE  # × 0.8
    elif abs_error < SMALL_ERROR_THRESHOLD:    # < 0.05 rad
        kp = BASE_KP[joint_idx] * SMALL_ERROR_KP_SCALE  # × 0.7
        kd = BASE_KD[joint_idx] * SMALL_ERROR_KD_SCALE  # × 1.2
    else:
        kp = BASE_KP[joint_idx]
        kd = BASE_KD[joint_idx]
    
    return min(kp, 500.0), min(kd, 5.0)  # 限制在安全範圍
```

**可調參數**：
- `LARGE_ERROR_THRESHOLD`：大誤差閾值（增大 → 更少進入高剛度模式）
- `SMALL_ERROR_THRESHOLD`：小誤差閾值（減小 → 更少進入低剛度模式）
- `LARGE_ERROR_KP_SCALE`：大誤差時 Kp 倍數（增大 → 追蹤更快）
- `SMALL_ERROR_KD_SCALE`：小誤差時 Kd 倍數（增大 → 穩態更穩）

**切換方式**：設 `USE_DYNAMIC_KP_KD = False` 可回退到靜態 Kp/Kd

---

### 2026-02-05：解決「山峰」震盪問題

**問題描述**：
- Unity 目標為水平時，實際位置出現「山峰」狀過衝
- 特別是 Joint 1, 6, 7 最明顯
- 見圖表：`right_joint1_20260205_171844.jpg`

**根因分析**：
重力補償邏輯根據「誤差方向」決定補償方向：
```python
position_error = target_pos[i] - current_pos[i]
if position_error > 0:
    compensation_ratio = 1.0   # 向上推
else:
    compensation_ratio = -1.0  # 向下推（問題！）
```

當馬達**過衝**目標時（actual > target），補償會**反向**，加劇震盪。

**測試步驟**：

1. **關閉 Ruckig** → 仍有山峰
2. **關閉重力補償** → 山峰消失！但有穩態誤差（手臂下垂）
3. **提高 Kp/Kd** → 用高剛度替代重力補償

**最終方案**：
```python
# 關閉重力補償
left_comp = [0.0] * 7
right_comp = [0.0] * 7

# 提高 Kp/Kd 克服重力
KP = [60.0, 60.0, 40.0, 40.0, 30.0, 30.0, 30.0]  # 原: [30, 30, 20, 20, 5, 5, 5]
KD = [4.0, 4.0, 1.2, 1.0, 1.0, 1.0, 1.0]         # 原: [3.5, 3.5, 0.7, 0.4, 0.7, 0.6, 0.5]
```

**相關圖表**：
- 關閉 Ruckig 前：`right_joint7_20260205_165912.jpg`
- 關閉 Ruckig 後（有山峰）：`right_joint1_20260205_171844.jpg`
- 關閉重力補償後（無山峰但有誤差）：`right_joint1_20260205_174944.jpg`

---

### 2026-02-05：Unity 發送頻率調整

**發現**：Unity 原本只有 **2Hz** 發送頻率（非常低！）

**修改**：提升至 **50Hz**

**效果**：藍線（Unity Target）仍呈階梯狀，因為追蹤源本身是離散數據

---

### 2026-02-02：Joint 1-2 增加 Kd 抑制靜態震盪

**問題**：Joint 1-2 在目標靜止時出現鋸齒狀抖動

**原因**：Kp/Kd 比值過高（30/2.5 = 12），阻尼不足

**修改**：
```python
# 舊值
KD = [2.75, 2.5, 0.7, 0.4, 0.7, 0.6, 0.5]

# 新值（J1-2 增加阻尼）
KD = [3.5, 3.5, 0.7, 0.4, 0.7, 0.6, 0.5]
```

---

### 2026-01-30：方案 A+ Ruckig 參數調整

**問題**：追蹤延遲過大

**修改**：
```python
# 舊值
RUCKIG_MAX_VELOCITY = [1.2, 1.2, 1.5, 1.5, 2.0, 2.0, 2.0]
RUCKIG_MAX_ACCELERATION = [8.0, 8.0, 12.0, 12.0, 15.0, 15.0, 15.0]
RUCKIG_MAX_JERK = [40.0, 40.0, 60.0, 60.0, 80.0, 80.0, 80.0]

# 新值（約 30~50% 馬達能力）
RUCKIG_MAX_VELOCITY = [4.0, 4.0, 2.5, 2.5, 6.0, 6.0, 6.0]
RUCKIG_MAX_ACCELERATION = [20.0, 20.0, 15.0, 15.0, 30.0, 30.0, 30.0]
RUCKIG_MAX_JERK = [100.0, 100.0, 80.0, 80.0, 150.0, 150.0, 150.0]
```

**結果**：追蹤延遲改善，但 Joint 1-2 出現靜態震盪

---

## 達妙馬達性能規格

### 馬達型號對照表

| 關節 | 馬達型號 | 用途 |
|------|----------|------|
| Joint 1-2 | DM8009 (DM-J8009-2EC) | 肩部大關節 |
| Joint 3-4 | DM4340 (DM-J4340-2EC) | 手肘關節 |
| Joint 5-7 | DM4310 (DM-J4310-2EC) | 手腕小關節 |

### DM8009 規格
- 額定扭矩：20 Nm / 峰值扭矩：40 Nm
- 額定轉速：100~200 RPM ≈ **10.47 rad/s**
- 減速比：9:1

### DM4340 規格
- 額定扭矩：9 Nm / 峰值扭矩：27 Nm
- 額定轉速：36 RPM ≈ **3.77 rad/s**
- 減速比：40:1

### DM4310 規格
- 額定扭矩：3 Nm / 峰值扭矩：7 Nm
- 額定轉速：120 RPM ≈ **12.57 rad/s**
- 減速比：10:1

---

## MIT 控制參數指南

### 控制公式
```
τ = Kp × (target - actual) + Kd × (target_vel - actual_vel) + τ_ff
```

### 參數範圍
| 參數 | 範圍 | 說明 |
|------|------|------|
| Kp | 0 ~ 500 | 位置增益（剛度） |
| Kd | 0 ~ 5 | 速度阻尼 |

### 常見設定範例

| 應用場景 | Kp | Kd | 說明 |
|----------|-----|-----|------|
| 一般位置控制 | 50 | 1.0 | 標準追蹤 |
| 快速響應 | 80~100 | 0.5~1.0 | 追蹤更快，可能抖動 |
| 柔順控制 | 10~30 | 2.0~3.0 | 適合人機互動 |
| 高精度定位 | 100~150 | 1.5~2.0 | 需要高剛度場景 |

### 重要注意事項

1. **Kd 不能為 0**：位置控制時若 Kd=0，會導致震盪
2. **Kp/Kd 比值**：建議保持在 10~20 之間
3. **逐步調整**：從較低 Kp 開始，逐步提高觀察響應

---

## Kp 設定對比

| 關節 | 馬達 | 原始 Kp | 社群常用 | 目前設定 |
|------|------|---------|----------|----------|
| J1-2 | DM8009 | 30 | 50~100 | **60** |
| J3-4 | DM4340 | 20 | 50~100 | **40** |
| J5-7 | DM4310 | 5 | 50 | **30** |

---

## 備選方案

### 動態 Kp/Kd 方案二：姿態補償

根據關節姿態（需抵抗重力的程度）調整 Kp：

```python
def _get_posture_kp_kd(self, joint_idx: int, current_pos: list) -> tuple:
    """根據姿態調整 Kp（抵抗重力）"""
    import math
    
    base_kp = BASE_KP[joint_idx]
    base_kd = BASE_KD[joint_idx]
    
    # J0, J1, J3 需根據姿態增加 Kp 抵抗重力
    if joint_idx == 0:
        gravity_factor = abs(math.sin(current_pos[0]))
        kp = base_kp * (1.0 + 0.5 * gravity_factor)  # 最多增加 50%
    elif joint_idx == 1:
        gravity_factor = abs(math.sin(current_pos[1]))
        kp = base_kp * (1.0 + 0.3 * gravity_factor)
    elif joint_idx == 3:
        gravity_factor = abs(math.sin(current_pos[3]))
        kp = base_kp * (1.0 + 0.3 * gravity_factor)
    else:
        kp = base_kp
    
    return min(kp, 500.0), base_kd
```

### 動態 Kp/Kd 方案三：速度模式

根據運動狀態調整（移動中 vs 靜止）：

```python
def _get_velocity_kp_kd(self, joint_idx: int, velocity: float) -> tuple:
    """根據速度調整 Kp/Kd"""
    STATIC_THRESHOLD = 0.01  # rad/s
    
    if abs(velocity) < STATIC_THRESHOLD:
        # 靜止：低 Kp 高 Kd 保持穩定
        kp = BASE_KP[joint_idx] * 0.6
        kd = BASE_KD[joint_idx] * 1.5
    else:
        # 移動中：高 Kp 低 Kd 快速追蹤
        kp = BASE_KP[joint_idx] * 1.2
        kd = BASE_KD[joint_idx] * 0.9
    
    return min(kp, 500.0), min(kd, 5.0)
```

---

### 改進版重力補償（如需恢復）

如果高 Kp 仍不足以克服重力，可考慮改進重力補償邏輯：

```python
def improved_gravity_compensation(current_pos, side):
    """純姿態補償，不根據誤差方向"""
    import math
    comp = [0.0] * 7
    
    # J0: 根據姿態給固定補償（不會反向）
    comp[0] = 8.0 * math.sin(current_pos[0])
    comp[1] = 6.0 * math.sin(current_pos[1]) * abs(math.sin(current_pos[0]))
    comp[3] = 3.0 * math.sin(current_pos[3])
    
    return comp
```

關鍵：補償方向由 `sin(角度)` 決定，不會因過衝而反轉。

---

## 參考資料

- [Damiao Series Motors - Seeed Studio Wiki](https://wiki.seeedstudio.com/damiao_series/)
- [DM-J4310-2EC Manual (PDF)](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf)
- [DM-J8009P-2EC Manual (PDF)](https://f.foxtech.com/manual/DM-J8009P-2EC.pdf)
- [GitHub - DM_Control_Python](https://github.com/cmjang/DM_Control_Python)

---

## 相關檔案

- 主程式：`unity_interface_follower.py`
- 數據記錄：`joint_data_logger.py`
- 繪圖工具：`plot_joint_comparison.py`

---

*建立日期：2026-01-30*
*最後更新：2026-02-11*
