# 修复总结

## 问题
1. ✗ 原始错误：`'mujoco._structs._MjDataBodyViews' object has no attribute 'xangacc'`
2. ✗ 后续问题：车体在控制脚本中浮在半空中

## 已实施的修复

### 1. **添加基座自由度** ✓
   - 在 `output_base.xml` 中添加自由根关节
   - 使基座可以自由移动和旋转（6个自由度）

### 2. **修复 MuJoCo API 兼容性** ✓
   - **移除不存在的属性访问**:
     - `xangacc` → 从 `qacc[3:6]` 获取
     - `xangvel` → 从 `cvel[3:6]` 获取
   
   - **添加坐标转换助手**:
     - `quat2euler()`: 四元数转欧拉角
     - `euler2quat()`: 欧拉角转四元数

### 3. **更新状态空间模型** ✓
   - 从 14 维扩展到 22 维
   - 新状态向量: `[x,y,z, roll,pitch,yaw, vx,vy,vz, wx,wy,wz, hip_angles, hip_vels, wheel_vels]`
   - 更新 `Q` 矩阵权重以适应新维度

### 4. **正确初始化自由关节** ✓
   - 修改 `reset_to_equilibrium()` 使用 `qpos` 设置位置和四元数
   - 直接通过 `data.qpos` 和 `data.qvel` 访问根关节状态

### 5. **修正线性化过程** ✓
   - 更新 `linearize_model()` 使用 `qpos` 而非直接修改 `xpos`/`xquat`
   - 正确处理位置、姿态和速度的扰动

### 6. **增强数值稳定性** ✓
   - 添加 LQR 矩阵诊断信息
   - 实现自适应正则化以应对病态问题
   - 更好的错误处理

## 验证结果

✓ `test_gravity.py`: 基座从 1m 高降到 0.086m  
✓ `test_init.py`: 初始化和状态获取正常  
✓ `test_simple_sim.py`: 基座从 0.3m 降到 0.083m 并停止

## 文件修改

- **output_base.xml**: 添加自由根关节
- **test.py**: 
  - 添加坐标转换函数
  - 更新状态维度和权重矩阵
  - 修正所有 MuJoCo API 调用
  - 改进 LQR 求解鲁棒性

## 使用说明

运行简单仿真（推荐用于测试）:
```bash
python3 test_simple_sim.py
```

运行完整控制脚本（包含 LQR）:
```bash
python3 test.py
```
