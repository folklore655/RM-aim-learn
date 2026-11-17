# RM 视觉组二轮考核 · ROS 2 自瞄

Aim Lab 仿真自瞄。输入 `/aim_sim/armors` 的装甲板四角坐标，输出云台 yaw/pitch 与开火指令。六个场景全部击毁 350 HP 靶车。

| 任务 | 场景 | 有效命中率 | 命中/发射 |
|---|---|---|---|
| 一 | 4 m 固定 | 94.7 % | 18/19 |
| 二 | 6 m 固定 | 90.0 % | 18/20 |
| 三 | 10 m 固定 | 85.7 % | 18/21 |
| 四 | 6 m 平移 | 78.3 % | 18/23 |
| 五 | 6 m 旋转 | 85.7 % | 18/21 |
| 六 | 6 m 平移旋转 | 85.7 % | 18/21 |

都是单轮成绩，界面时刻与截图见 `results/总成绩.md`。

## 环境

- WSL2 Ubuntu 24.04，ROS 2 Jazzy
- 仿真仓库 `rm_aim_sim_training`（自带 `rm_aim_sim_bridge` 消息包）
- 依赖：`rclcpp`、`rm_aim_sim_bridge`，见 `ros2_ws/src/aim_learn/package.xml`

## 编译

```bash
source /opt/ros/jazzy/setup.bash
source <仿真仓库>/ros2_ws/install/local_setup.bash   # 不 source 会找不到 rm_aim_sim_bridge
mkdir -p ~/aim_ws/src && cp -r ros2_ws/src/aim_learn ~/aim_ws/src/   # 在仓库根目录执行
cd ~/aim_ws && colcon build --packages-select aim_learn
source install/local_setup.bash
```

## 运行

```bash
# 终端1 节点
cd ~/aim_ws && source install/local_setup.bash && ros2 run aim_learn peek

# 终端2 仿真（cfg 用本仓库 scenarios/ 里的绝对路径）
cd <仿真仓库>
./run.sh --headless --config <本仓库>/scenarios/task02_6m.cfg
```

任务一用仿真自带的 `configs/default.cfg`；其余场景 cfg 在 `scenarios/`。要改场景直接在 GUI 里调完点「保存当前配置」，或手改 cfg 的 `[target]` 段。

读成绩：

```bash
python3 -c "import json;d=json.load(open('/dev/shm/rm_aim_sim_default/state.json'));s=d['stats'];print(s['score_hit_rate'],s['kill_time'],s['valid_hits'],s['fired'])"
```

`kill_time` 是仿真启动到击杀的绝对时间，含预热，跨场景不比。`score_hit_rate = valid_hits/fired`，击杀瞬间还在飞的子弹只进分母。

两条注意：同一 channel 只能起一套仿真，否则报 `本机 IPC 启动失败：22`；本机 ros2 CLI daemon 有 bug，`ros2 topic list/echo` 之类要加 `--no-daemon`。

## 结构

单文件 `ros2_ws/src/aim_learn/src/peek.cpp`，291 行，四块：

1. 常量与跨帧状态（速度、角速度、车心、时间戳）
2. 工具函数：板中心、板法向、反推车心、弹道解算、停火
3. `on_armors` 回调，六步完成一次瞄准
4. `main`：建节点、订阅 `armors`、发布 `command`

数据流：仿真 → `/aim_sim/armors`（约 50 Hz）→ 本节点（回调里算完立刻发布）→ `/aim_sim/command` → bridge → 仿真。

## 算法

1. 四角平均得板中心，上边×左边归一化得法向。
2. 反推车心 `C = p − r·n`。n 块板的法向符号可能翻，枚举 2ⁿ 种组合，取各板反推出的 C 最一致的那组；单板时取离上一帧最近的。
3. 差分求平移速度与角速度，位置和朝向用各自时间戳分别更新（桥接会用新时间戳重发同一帧）。
4. 预测 `lead = 0.015 + t_f` 后的板位姿：`c' = C + Rz(ω·lead)(p−C) + v·lead`，法向同法旋转。
5. 取 `face = n'·(o−c')/|o−c'|` 最大的板，要求 `face > 0.8`、与次优差 `> 0.15`，且不在换向减速段，才发 `fire=true`；否则只转向不开火。
6. 仰角取低伸解 `tan e = (V²−√(V⁴−g(gd²+2hV²)))/(gd)`，枪口偏移迭代 3 次。判别式 `< 0` 表示超射程，本帧不开火。

指令一律复制所观测的 `header` 与 `session_id`；观测超 250 ms、`armors` 为空时主动发 `enable=false, fire=false`。

## 参数

| 项 | 值 |
|---|---|
| 弹速 / 重力 | 23.0 m/s / 9.7833 |
| 枪口长度 | 0.22 m |
| 云台高 | 0.65 m（odom 原点在云台旋转中心，板中心 z 约 −0.39） |
| 装甲板 | 0.135 × 0.125 m，倾角 15° |
| 旋转半径 | 0.2480696 m |
| 射击 | 10 Hz，指令延迟 0.015 s |
| 提前量基数 | 0.015 s |
| 开火门限 | face 0.8，与次优差 0.15 |

## 仓库

`git@github.com:folklore655/RM-aim-learn.git`，按阶段提交。
