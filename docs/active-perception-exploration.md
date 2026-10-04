你的理解已经非常接近现在自主机器人研究里的核心问题了。你说的这种：

> “机器初始掌握的信息尽可能少，通过自身行动主动获得外界信息，然后逐步建立环境模型。”

主流名称最接近的是：

> **Active Perception**（主动感知）

如果重点是“主动探索未知空间并建图”，通常叫：

> **Active SLAM**

如果更强调“选择下一步往哪里走，才能获得最大信息量”，通常叫：

> **Informative Path Planning**

或：

> **Next-Best-View / Next-Best-Action**

而如果进一步把“不确定性”本身作为机器人内部状态建模，则会进入：

> **Belief-space Planning / POMDP**

这些实际上是同一条思想链上的不同层次。

---

# 一、先区分两类根本不同的问题

机器人运动大致可以分成：

> **Known Environment（已知环境）** 和 **Unknown Environment（未知环境）**

也就是：

> 已知世界里的运动 和 未知世界里的自主探索。

这两件事虽然最后都表现为“机器人走路”，内部算法完全不同。

---

# 1. 已知空间里：核心问题是“怎么安全到目标”

假设机器人已经有地图 `M`，而且知道自己的位置 `x_t`。例如：

```
已知地图

████████████
█          █
█   Robot  █
█      █   █
█      █ G █
████████████
```

目标 `G` 已知。

这时候问题就是：`x_t → x_g`，求一条满足约束的路径：

> `π* = arg min_π C(π)`

常见方法就是我们熟悉的：

- A*
- Dijkstra
- D*
- D* Lite
- Hybrid A*
- RRT / RRT*
- PRM
- MPC
- trajectory optimization

移动机器人还会分两层：

```
Global Planner
地图 → A* / D* → 全局路径

Local Planner
局部传感器 → 避障 → 短期轨迹
```

例如 Nav2 其实基本就是这样的体系。

---

# 2. 训练环境中的机器人又稍微不同

如果是在 Gazebo、Isaac Sim、Habitat、MuJoCo 这种已知模拟环境里训练，还可以使用 **Reinforcement Learning（强化学习）**。例如定义：

- `s_t` = state
- `a_t` = action
- `r_t` = reward

机器人学习策略 `π(a|s)`，使：

> `max E[ ∑_t γ^t · r_t ]`

这种情况下环境虽然对策略来说可能“不可见”，但训练者其实知道世界。比如奖励：

> `r = -0.1·distance - 10·collision + 100·goal`

然后让机器人训练几十万、几百万次。

所以这类本质是：

> **在人工准备的环境分布中学习行动策略。**

问题是它可能产生 **Simulation Bias**（仿真偏差），也就是到了真正新环境：

```
训练：平地、箱子、规则墙壁
现实：碎石、烟尘、塌方、斜坡、未知材质
```

性能可能突然下降。

---

# 二、真正困难的是未知环境

现在假设机器人被扔进一个从没见过的地方。比如：月球洞穴、地震废墟、地下隧道。

这时候机器人一开始可能只知道：

```
当前位置附近：

????????
???R????
????????
```

问号代表 **unknown**。它根本不知道：哪里有路、哪里有墙、哪里有坑、前面有没有出口、自己到底走了多远、有没有回到原位置。

所以不能直接 `A*(start, goal)`，因为连地图都不存在。

---

# 三、这时候机器人必须同时做三件事情

核心变成：

> **Localization + Mapping + Exploration**

即：

```
我在哪里？ + 这里是什么？ + 我下一步去哪？
```

这就是自主探索系统的核心。

SLAM 解决前两个：**Localization + Mapping**。但 SLAM 本身并不完全回答“下一步主动去哪里”，这就是 **Autonomous Exploration**。

---

# 四、最经典的方法：Frontier Exploration

这是非常重要的一种方法。地图被分成 `{free, occupied, unknown}`：

```
?????????????????

????....?????????
????....?????????
????.R..?????????
????....?????????
####....?????????
```

这里 `.` = free，`#` = obstacle，`?` = unknown。

已知区域和未知区域之间的边界叫 **Frontier**，机器人会寻找 `free | unknown` 的边界，然后 `Frontier → Goal` 走过去，到那里再扫描。于是 `unknown → sensor → known`，然后重新寻找 frontier，循环：

```
Sense → Map → Frontier → Plan → Move → Sense
```

直到几乎没有 frontier。

Frontier exploration 至今仍然是自主探索的重要基线方法。相关综述也把它描述为“不断驶向已知自由空间与未知空间之间的边界，从而扩展地图”。[Frontiers](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2021.642780/full)

---

# 五、但是你提出的思想比 Frontier 更进一步

你刚才实际上提出了一个更有意思的问题：

> 机器人为什么一定往最近的未知区域走？

假设有两个方向：

```
       A
       ↑
       │
Robot ─┼────→ B
```

A 很近，但只能看见一个小角落；B 稍远，但可以看到一个巨大空间。

那么最优行为可能不是去最近位置，而是选择 **信息收益最大**。

---

# 六、这就是 Information Gain

假设机器人当前地图的不确定性是 `H(M)`，走到位置 `x` 后观察 `z`，地图不确定性变成 `H(M|z)`。信息收益：

> `IG(x) = H(M) - E[ H(M | z_x) ]`

那么机器人可以选择：

> `x* = arg max_x IG(x)`

也就是：“去那个最可能让我知道更多东西的地方”。这就是 **Information-driven Exploration**。

现有研究通常把未知环境探索方法划分成 Frontier-based 和 Information-driven 两类。后者会使用概率地图，评估不同候选视点能够带来的期望信息增益。[Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.911974/full)

---

# 七、实际不会单纯最大化信息量

因为这样可能出问题。例如：

```
这里信息很多
      ↓
     悬崖
```

如果只优化 `IG`，机器人可能为了获取信息走向危险区域。

所以更真实的目标函数是：

> `U(x) = α·I(x) - β·C(x) - γ·R(x) - δ·E(x)`

其中 `I(x)` = 信息收益，`C(x)` = 运动成本，`R(x)` = 风险，`E(x)` = 能耗。于是 `x* = arg max_x U(x)`。这就已经非常接近星球探索机器人真正需要解决的问题了。

---

# 八、如果是星球、废墟、洞穴，这个问题会进一步升级

因为那里不是简单二维迷宫。你实际需要建立的是 `M(x, y, z)`，例如：

```
Voxel map / OctoMap / TSDF / ESDF / point cloud / mesh / Gaussian map
```

然后每一个三维区域可能包含 `v_i = [occupancy, terrain, slope, semantic, uncertainty]`。例如：

```
Voxel A: occupancy = 0.1, slope = 8°, roughness = 0.2, confidence = 0.91
Voxel B: occupancy = ?, slope = ?, confidence = 0.12
```

于是机器人对世界的理解本质变成：

> **带概率和不确定性的三维地图**

---

# 九、这时候非常重要的概念叫 Traversability

不是所有 `free` 空间都 `traversable`。比如：空中是 free 但轮式机器人不能走；30° 坡可能走；70° 坡不能走；碎石可能危险；泥地可能打滑。

所以机器人需要计算 `T(x, y, z)`，即可通行性（Traversability）。可能依据：

> `T = f(slope, roughness, step, soil, support, clearance)`

这在星球机器人和废墟机器人里极其重要。

NASA 当前的自主行星探索研发也正朝向更高自主性的 terrain exploration，强调 LiDAR、感知和车载计算，让未来 rover 在更少人工干预下进行高速自主地表移动。[NASA TechPort](https://techport.nasa.gov/projects/184632)

---

# 十、如果环境非常危险，还必须加入 Risk-aware Planning

假设：

```
路线 A：10 m，但是穿过未知区域
路线 B：18 m，全部已知安全
```

机器人不能简单 `min distance`，而应该 `min( distance + λ·risk )`，甚至 `risk = P(failure) × Cost(failure)`。例如 `P(fall) = 0.1`，如果掉落意味着机器人任务完全失败，那风险非常高。

---

# 十一、NASA JPL 有一个特别符合你描述的系统

叫 **NeBula**，全称 **Networked Belief-aware Perceptual Autonomy**。

这是 JPL 针对未知极端环境自主机器人开发的一套系统。它重点考虑：传感器不确定性、环境不确定性、运动不确定性、通信不确定性、系统健康状态、风险。并包含：resilient navigation、3D SLAM、dense 3D mapping、traversability、risk-aware decision making、multi-robot exploration。

JPL 明确把它定位为未知环境和极端地形中的自主机器人系统，并报告了火星类比洞穴的全自主探索实验。[JPL Robotics](https://robotics.jpl.nasa.gov/how-we-do-it/systems/nebula-autonomy-suite/)

这个案例其实与你刚才设想的架构非常接近。

---

# 十二、DARPA SubT 也是非常典型的现实验证

DARPA 的 Subterranean Challenge 几乎就是：

> “把机器人扔进人不应该进去的未知环境会怎样？”

环境包括：洞穴、地下隧道、塌方区域、黑暗、烟雾、灰尘、台阶、陡坡、大坑、GPS-denied。

而且要求 **minimal human intervention**，机器人自己：

```
探索 → 建图 → 定位 → 识别目标 → 决定下一目标 → 继续探索
```

DARPA 总结的四个核心技术方向就是：

> **Autonomy + Perception + Networking + Mobility**

而且它明确考虑传感器退化、严重通信受限、动态地形和复杂障碍，这正是灾害废墟这类环境的实际困难。

---

# 十三、你提出来的“感知函数”其实可以正式写成数学形式

假设真实世界 `W`，机器人不能直接看到 `W`，只能通过传感器：

> `z_t = h(W, x_t) + ε`

其中 `h` 就是你说的“感知函数”。机器人只能看到 `z_t` 而不是 `W`。例如真实世界有整个洞穴，但 LiDAR 一次只能看到半径 20m 且无遮挡的区域。所以 `W ≠ z_t`，机器人内部只能维护 `Ŵ_t`，即“当前世界估计”。

---

# 十四、于是循环变成你想的这种形式

```
W →(sensor)→ z_t
(z_1, …, z_t) → Ŵ_t
机器人判断：我最缺什么信息？
选择动作 a_t → 得到新观测 z_{t+1}
```

于是：

> **Perceive → Estimate → Find uncertainty → Act → Perceive**

这是非常关键的闭环。

---

# 十五、这就是 Active Perception

普通感知：机器人在那里不动，Camera → 识别，叫 **Passive Perception**。

主动感知：这个东西看不清 → 机器人换位置 → 再次观察 → 获得更多信息，叫 **Active Perception**。

主动视觉研究里会主动改变相机的位置或视角，让感知结果更好；常见策略是选择 **Next Best View**。[Frontiers](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2021.642780/full)

---

# 十六、Next Best View 正好就是你描述的方法

假设机器人现在：

```
       Object
       ███?
Robot → ███?
       ????
```

右侧没看见。机器人计算多个候选位置 `x_1, x_2, x_3`，分别预测 `IG(x_1)`、`IG(x_2)`、`IG(x_3)`，选择 `x* = arg max IG(x)`，于是机器人主动走到那里。这个叫 **Next Best View**（NBV）。

---

# 十七、如果不是一个视角，而是一整条路径

那么 `π = {x_1, x_2, …, x_n}`，优化：

> `π* = arg max_π [ IG(π) - λ·Cost(π) ]`

就叫 **Informative Path Planning**，也就是“信息驱动路径规划”。这是很成熟的研究领域。

---

# 十八、再往理论上走，就是 Belief

这是你这个想法中特别值得研究的地方。

机器人并不应该说“这个位置是空的”，而是 `P(free) = 0.92` 或 `P(occupied) = 0.08`。于是世界状态不是 `s`，而是概率分布 `b(s)`，叫 **Belief**。

机器人维护：

> `b_t(s) = P(s | z_{1:t}, a_{1:t})`

然后规划不是 state space，而是 **belief space**，这叫 **Belief Space Planning**。它特别适合 GPS denied、传感器有噪声、地图不确定、环境未知这样的机器人。

---

# 十九、更标准的理论形式其实是 POMDP

完整模型是 **POMDP**（Partially Observable Markov Decision Process）。

定义 `(S, A, T, R, O, Z)`：机器人看不到真正的 `s_t`，只能得到 `o_t`，所以维护 `b_t(s)`，然后选择 `a_t` 使长期收益：

> `E[ ∑_{t=0}^{∞} γ^t · R_t ]`

最大。

这其实就是：

> “我不知道真实世界是什么，但我有一个概率信念，然后根据这个信念行动。”

---

# 二十、你说的“初始信息最少”还有一个更深的问题

你其实提出了 **Exploration vs Exploitation**。机器人可以：

**Exploitation**（利用）：利用已经知道的知识完成任务。例如“已知出口 → 直接走”。

**Exploration**（探索）：为了减少未知而行动。例如“不知道右边是什么 → 先过去看”。

理想机器人不断权衡：

> **Task Reward + Information Reward**

例如：

> `R = R_task + λ·R_information`

这是强化学习、主动感知和机器人探索共同出现的问题。

---

# 二十一、如果让我设计一个真正“无人干预”的三维探索机器人

比如火星洞穴机器人。我不会设计成“一个神经网络，Camera → action”，而是分层：

```
LiDAR ─┐
Camera ├─→ Sensor Fusion
IMU ───┤
Odom ──┘
          ↓
        3D SLAM
          ↓
     Probabilistic Map
          ↓
    Traversability Map
          ↓
 Uncertainty / Frontier Map
          ↓
Active Exploration Planner
          ↓
 Risk-aware Path Planner
          ↓
 Local Motion Planner
          ↓
       Controller
          ↓
        Robot
```

同时维护：World Model + Robot State + Mission State + Health State。

---

# 二十二、而未来更先进的系统会再多一层

现在越来越可能出现 Geometry + Semantics + Physics + Uncertainty。例如：

```
岩石A：位置 = …，体积 = …，不可通行，置信度 = 0.96
地面B：坡度 = 14°，松散，打滑概率 = 0.23
洞穴C：unknown，信息价值 = 0.82
```

机器人内部世界模型就可能变成 `M = (M_g, M_s, M_p, M_u)`，分别是 geometry、semantics、physics、uncertainty。这样才真正有能力进行长期自主探索。

---

# 你刚才的想法可以浓缩成一个非常标准的研究问题

你其实问的是：

> **一个具有有限感知范围的自主智能体，应当如何选择自己的运动，使其在最小运动代价和风险下，最大限度降低对未知三维环境的不确定性，并同时完成任务？**

写成数学形式：

> `π* = arg max_π [ I(M; Z_π)  信息获取  +  α·R_task(π)  任务收益  -  β·C(π)  运动代价  -  γ·Risk(π)  风险 ]`

这个已经不是普通“寻路”问题。它实际上同时属于：

> **Active SLAM + Informative Path Planning + Active Perception + Belief Space Planning**

所以你最后问“这种方法主流叫什么”，如果只能选一个最贴近你思想的词，我会选：

> **Active Perception（主动感知）**

如果专门研究“机器人自己走到哪里才能把未知三维世界建出来”，我会更准确地称为：

> **Autonomous Exploration / Active SLAM**

这其实是一个相当好的研究方向，而且跟我们现在 ROS 项目下一步能自然衔接：**2D SLAM → frontier exploration → 3D occupancy → 3D frontier/NBV → information-gain planner → risk-aware autonomous exploration**。这条路线最终就能够从一个普通 Nav2 小车逐步走到你说的“扔进陌生环境自己理解世界”的系统。
