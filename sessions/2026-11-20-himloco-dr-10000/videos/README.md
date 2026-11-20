# 视频说明

## `isaac-consolidate-18200.mp4` — 已验证，可引用

Isaac Lab 渲染的 model_18200 回放，8 个工况（Stand / Forward / Reverse / 4 组弧线 / Fast left），前视 + 侧视双机位，**8 段全部无 fall**，与 48/48 存活一致。约 48 秒、6.1 MB。

## `mujoco-draft-unvalidated.mp4` — 未验证，请勿引用为能力证据

用于验证 MuJoCo 第二仿真器管线（场景生成、ONNX 闭环、xvfb+glfw 渲染、ffmpeg 编码）是否跑通，**不是**策略能力演示。

已知问题：

1. 轮子是**显式力矩速度环**（`τ = kd·(v*−v)`，kd=2，轮惯量 0.595 kg），在 1.25 ms 步长下刚性不稳定 → 零速命令下轮子自激（`dq` 达 ±100 rad/s），机身乱窜
2. 轮轴方向与策略约定相反（已发现：vx=+0.3 实际后退 0.10 m，vx=−0.3 前进 0.99 m），需要取反或改轴
3. 从官方 URDF 生成的场景缺少正确的轮地接触与关节 armature/damping

修复方向：轮子改隐式 `<velocity>` 执行器、加 armature 0.02–0.05、接触参数对齐、重标定后重新验证站立/前进/弧线，再出正式视频。
