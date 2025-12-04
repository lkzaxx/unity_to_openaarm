# OpenArm 左右手關節限制修正建議

## 📋 問題摘要

根據實際測試，左右手在執行相同的關節數值（例如 1.5 rad）時，運動方向相反，但當前配置文件中左右手使用**相同的關節限制**，導致實際運動範圍不對稱。

### 測試結果（設定 1.5 rad，從機器人後方觀察）

| 關節 | 左手動作 | 右手動作 | 方向是否相反 |
|------|---------|---------|-------------|
| Joint 1 | 往後 | 往前 | ✅ 相反 |
| Joint 2 | 往身體靠（逆時針） | 往身體外（逆時針） | ✅ 相反 |
| Joint 3 | 往身體旋轉（順時針） | 往身體外旋轉（順時針） | ✅ 相反 |
| Joint 4 | 肘往上（前） | 肘往上（前） | ❌ 相同 |
| Joint 5 | 往身體旋轉（順時針） | 往身體外旋轉（順時針） | ✅ 相反 |
| Joint 6 | 往身體外轉（順時針） | 往身體轉（順時針） | ✅ 相反 |
| Joint 7 | 往後 | 往前 | ✅ 相反 |

**結論**：除了 Joint 4 外，其他所有關節的左右手運動方向都相反，需要鏡像關節限制。

---

## 📂 當前配置文件位置

### 1. 基礎關節限制定義
**文件**：[`src/openarm_description/config/arm/v10/joint_limits.yaml`](file:///c:/code/vr_robot/openarm/unity_to_openaarm/ros2_ws/src/openarm_description/config/arm/v10/joint_limits.yaml)

定義了所有關節的基礎限制（通用於左右手）：
```yaml
joint1:
  limit:
    lower: -1.396263  # -80°
    upper: 3.490659   # 200°
    velocity: 16.754666
    effort: 40

joint2:
  limit:
    lower: -1.745329  # -100°
    upper: 1.745329   # 100°
    velocity: 16.754666
    effort: 40

# ... 其他關節
```

### 2. URDF 鏡像邏輯
**文件**：[`src/openarm_description/urdf/arm/openarm_arm.xacro`](file:///c:/code/vr_robot/openarm/unity_to_openaarm/ros2_ws/src/openarm_description/urdf/arm/openarm_arm.xacro)

定義左右手的 `reflect` 參數：
- **右手**：`reflect = 1`（L10-12）
- **左手**：`reflect = -1`（L14-16）

### 3. 限制計算邏輯
**文件**：[`src/openarm_description/urdf/arm/openarm_macro.xacro`](file:///c:/code/vr_robot/openarm/unity_to_openaarm/ros2_ws/src/openarm_description/urdf/arm/openarm_macro.xacro#L91-L114)

`openarm-limits` macro（L91-114）：
```xml
<xacro:property name="raw_lower" value="${limits.lower * reflect + offset}" />
<xacro:property name="raw_upper" value="${limits.upper * reflect + offset}" />
```

### 4. Python 保護機制
**文件**：[`src/openarm_ros2/openarm_bringup/scripts/unity_interface.py`](file:///c:/code/vr_robot/openarm/unity_to_openaarm/ros2_ws/src/openarm_ros2/openarm_bringup/scripts/unity_interface.py)

當前使用統一的位置限制（L66-74），沒有區分左右手。

---

## ⚠️ 當前配置問題分析

### 問題 1：URDF 中 reflect 應用不一致

查看 `openarm_arm.xacro`，發現並非所有關節都使用 `reflect` 參數：

| 關節 | 使用 reflect? | 特殊 offset | 行號 |
|------|--------------|------------|------|
| Joint 1 | ❌ | 左手: -2.094396 (-120°) | L35 |
| Joint 2 | ✅ | 左手: -π/2, 右手: +π/2 | L56 |
| Joint 3 | ❌ | 無 | L66 |
| Joint 4 | ❌ | 無 | L76 |
| Joint 5 | ❌ | 無 | L86 |
| Joint 6 | ❌ | 無 | L96 |
| Joint 7 | ❌ | 無 | L106 |

**實際代碼**：
```xml
<!-- Joint 1: 只有 offset，無 reflect -->
<xacro:openarm-limits name="joint1" config="${joint_limits}" 
                      offset="${-2.094396 if arm_prefix=='left_' else 0}"/>

<!-- Joint 2: 有 reflect + offset -->
<xacro:openarm-limits name="joint2" config="${joint_limits}" 
                      reflect="${reflect}" offset="${limit_offset_joint2}"/>

<!-- Joint 3-7: 都沒有 reflect 或 offset -->
<xacro:openarm-limits name="joint3" config="${joint_limits}" />
```

### 問題 2：測試結果與 URDF 配置矛盾

根據測試：Joint 1, 2, 3, 5, 6, 7 都需要鏡像，但 URDF 中只有 Joint 2 使用了 `reflect`。

---

## 💡 修正方案

### 方案 A：修改 URDF（推薦）

修改 `openarm_arm.xacro`，為需要鏡像的關節添加 `reflect` 參數。

#### 修改 Joint 1（L35）
```xml
<!-- 修改前 -->
<xacro:openarm-limits name="joint1" config="${joint_limits}" 
                      offset="${-2.094396 if arm_prefix=='left_' else 0}"/>

<!-- 修改後 -->
<xacro:openarm-limits name="joint1" config="${joint_limits}" 
                      reflect="${reflect}"/>
```

#### 修改 Joint 3（L66）
```xml
<!-- 修改前 -->
<xacro:openarm-limits name="joint3" config="${joint_limits}" />

<!-- 修改後 -->
<xacro:openarm-limits name="joint3" config="${joint_limits}" 
                      reflect="${reflect}"/>
```

#### 修改 Joint 5（L86）
```xml
<!-- 修改前 -->
<xacro:openarm-limits name="joint5" config="${joint_limits}" />

<!-- 修改後 -->
<xacro:openarm-limits name="joint5" config="${joint_limits}" 
                      reflect="${reflect}"/>
```

#### 修改 Joint 6（L96）
```xml
<!-- 修改前 -->
<xacro:openarm-limits name="joint6" config="${joint_limits}" />

<!-- 修改後 -->
<xacro:openarm-limits name="joint6" config="${joint_limits}" 
                      reflect="${reflect}"/>
```

#### 修改 Joint 7（L106）
```xml
<!-- 修改前 -->
<xacro:openarm-limits name="joint7" config="${joint_limits}" />

<!-- 修改後 -->
<xacro:openarm-limits name="joint7" config="${joint_limits}" 
                      reflect="${reflect}"/>
```

### 方案 B：修改 Python 保護邏輯

如果不想修改 URDF，可以在 `unity_interface.py` 中為左右手分別定義限制。

#### 修改位置限制定義
```python
# 定義右手限制（基準）
self.right_joint_position_limits = {
    'joint1': {'lower': -1.1538, 'upper': 3.2482},   # 90% 範圍
    'joint2': {'lower': -1.5708, 'upper': 1.5708},
    'joint3': {'lower': -1.4137, 'upper': 1.4137},
    'joint4': {'lower': 0.1222, 'upper': 2.3213},    # 保持不變
    'joint5': {'lower': -1.4137, 'upper': 1.4137},
    'joint6': {'lower': -0.7069, 'upper': 0.7069},
    'joint7': {'lower': -1.4137, 'upper': 1.4137}
}

# 定義左手限制（鏡像）
self.left_joint_position_limits = {
    'joint1': {'lower': -3.2482, 'upper': 1.1538},   # 鏡像
    'joint2': {'lower': -1.5708, 'upper': 1.5708},   # 鏡像
    'joint3': {'lower': -1.4137, 'upper': 1.4137},   # 鏡像
    'joint4': {'lower': 0.1222, 'upper': 2.3213},    # 保持不變
    'joint5': {'lower': -1.4137, 'upper': 1.4137},   # 鏡像
    'joint6': {'lower': -0.7069, 'upper': 0.7069},   # 鏡像
    'joint7': {'lower': -1.4137, 'upper': 1.4137}    # 鏡像
}
```

#### 修改 clamp 函數
```python
def clamp_joint_positions(self, joint_positions, arm_prefix):
    """限制關節位置在安全範圍內"""
    clamped_positions = {}
    
    # 選擇對應的限制
    if arm_prefix == 'left_':
        limits = self.left_joint_position_limits
    else:
        limits = self.right_joint_position_limits
    
    for joint_name, position in joint_positions.items():
        # 移除前綴以獲取 joint1~7
        base_joint_name = joint_name.replace('openarm_left_', '').replace('openarm_right_', '')
        
        if base_joint_name in limits:
            limit = limits[base_joint_name]
            # ... 後續邏輯相同
```

---

## 🎯 建議採用方案

### ✅ 推薦：**方案 A（修改 URDF）**

**理由**：
1. **根本解決**：在 URDF 層級就正確定義左右手限制
2. **一致性**：所有使用 URDF 的工具（MoveIt、RViz）都會自動獲得正確限制
3. **可維護性**：只需修改一處，不需要在每個使用者（Unity、Python）都重複定義
4. **符合設計**：`reflect` 參數本來就是為此設計的

**缺點**：
- 需要重新編譯 URDF
- 需要重啟 Docker 容器

### ⚠️ 備選：**方案 B（修改 Python）**

**適用情況**：
- 臨時測試或快速驗證
- 不方便修改 URDF 時的權宜之計

**缺點**：
- 治標不治本
- 需要在多處維護限制定義
- MoveIt 等工具仍會使用錯誤的限制

---

## 📝 實施步驟（方案 A）

### 1. 修改 URDF 文件
```bash
# 編輯文件
nano ~/ros2_ws/src/openarm_description/urdf/arm/openarm_arm.xacro
```

按照上述方案 A 修改 Joint 1, 3, 5, 6, 7 的限制定義。

### 2. 重新編譯工作空間
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 驗證 URDF
```bash
# 檢查編譯後的 URDF
ros2 run xacro xacro ~/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
    > /tmp/test.urdf

# 檢查左右手的關節限制
grep -A 5 "openarm_left_joint1" /tmp/test.urdf
grep -A 5 "openarm_right_joint1" /tmp/test.urdf
```

### 4. 重啟系統並測試
```bash
# 重啟 ROS2 控制器
ros2 launch openarm_bringup openarm_v10_bimanual.launch.py

# 使用測試腳本驗證
cd ~/ros2_ws
bash scripts/test_joint_direction.sh
```

### 5. 更新文檔
更新 `JOINT_LIMITS.md`，記錄左右手的實際限制差異。

---

## ❓ 待確認問題

### 1. Joint 1 的 offset 用途
當前 Joint 1 有特殊的 offset（左手 -2.094396 rad ≈ -120°）：
```xml
offset="${-2.094396 if arm_prefix=='left_' else 0}"
```

**問題**：這個 offset 是否應該保留？還是應該完全使用 `reflect` 替代？

**建議**：需要實測確認 Joint 1 的實際零點位置是否需要此 offset。

### 2. Joint 2 的 offset 與 reflect
Joint 2 同時使用了 `reflect` 和 `offset`：
```xml
reflect="${reflect}" offset="${limit_offset_joint2}"
# 其中 limit_offset_joint2 = π/2 (右手) 或 -π/2 (左手)
```

**問題**：這個設計是否正確？是否所有關節都應該採用相同模式？

### 3. 對稱關節的限制值
Joint 2, 3, 5, 6, 7 的基礎限制都是對稱的（例如 ±90°），使用 `reflect` 後：
- 右手（reflect=1）：保持 -90° ~ +90°
- 左手（reflect=-1）：變成 -90° ~ +90°（相同範圍，但方向相反）

**問題**：這是否符合預期？還是需要調整基礎限制值？

---

## 📌 總結

| 項目 | 當前狀態 | 需要修改 |
|------|---------|---------|
| **joint_limits.yaml** | ✅ 定義正確 | ❌ 不需修改 |
| **openarm_arm.xacro** | ❌ 缺少 reflect | ✅ 需要修改 |
| **unity_interface.py** | ⚠️ 使用統一限制 | ⚠️ 建議同步更新 |
| **JOINT_LIMITS.md** | ⚠️ 文檔過時 | ✅ 需要更新 |

**下一步行動**：
1. 確認 Joint 1 的 offset 是否需要保留
2. 修改 `openarm_arm.xacro`
3. 重新編譯並測試
4. 更新文檔

---

**創建日期**：2025-12-02  
**測試環境**：OpenArm v10 雙臂系統  
**參考測試**：基於機器人後方視角的 1.5 rad 測試
