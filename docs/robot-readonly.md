# 运行模型与机器人的只读状态链路

链路为 **OpenAI Responses API → 本地函数分发 → MCP stdio → 状态读取 → ROS 订阅桥**。使用 `.env` 中配置的模型，不自动替换模型。

ROS 桥在仿真容器中常驻，仅订阅状态并原子写入 `workspace/log/robot-status.json`；MCP 服务由客户端按需启动，通过标准输入输出通信，不开放 HTTP 端口。API 密钥仅由本地模型客户端读取，不传给 ROS 桥或 MCP 子进程。

当前工具只有 `get_robot_status`，可读取地图坐标位姿、里程计、雷达最近距离、地图摘要、导航状态消息、guard 输出及节点发现情况。没有运动工具，模型请求不能转成任意 shell、ROS 发布或导航命令。

上面的限制适用于本页的只读入口。另有显式启动的[受限运动试验入口](model-motion-trial.md)，允许模型请求一次固定的短距离目标。

## 使用

仿真、SLAM、Nav2 与只读桥已在本次验证中启动。此时可以直接运行：

```bash
cd /mnt/robot_disk/ros_sim

# 通过真实 MCP 会话读取状态，不调用模型
./scripts/robot-status.sh

# 使用 .env 中的模型完成一次“请求工具 → 读取状态 → 中文解释”
./scripts/robot-status.sh --model --output logs/robot-readonly-model.json
```

`--model` 每次最多调用两次 Responses API，不循环调度、不自动重试，会产生正常 API 用量。两次请求均设置 `store=False`。仅状态摘要发送到 OpenAI，不发送完整地图栅格、原始扫描或密钥。第二次请求禁止继续调用工具。函数名和参数必须通过本地允许列表校验。

这是每次运行一次的状态查询，后台没有持续消耗 API 的模型循环。已返回的模型解释描述的是采集时刻；需要新状态时重新运行命令。

## 从关闭状态启动

环境依赖见 [OpenAI 环境说明](openai-environment.md)。仿真启动需要当前桌面的 DISPLAY 和 NVIDIA 运行环境。下面前三个长驻命令分别放在不同终端中运行；等待 SLAM 有地图后再启动 Nav2。

```bash
cd /mnt/robot_disk/ros_sim
./docker/prepare-display.sh
./docker/start-navigation-base.sh mapping

# 第二个终端
./docker/start-slam.sh

# 第三个终端
./docker/start-nav2.sh

# 第四个终端
./scripts/start-robot-readonly.sh
./scripts/robot-status.sh --model
```

`start-robot-readonly.sh` 可重复运行，会检查采集进程和新鲜快照。停止本轮启动的机器人服务：

```bash
docker stop robot-nav2 robot-sim-gui
```

MCP 子进程随每次查询退出；ROS 桥随仿真容器退出。旧快照超时后返回不可用，不会继续当成实时状态。

## 数据语义和边界

- 快照超过 3 秒或时间异常则整体不可用；字段同时带接收墙钟年龄、消息仿真时间年龄及 `fresh`。雷达、里程计、TF 等默认 1.5 秒，地图 30 秒；这些只是只读展示标准，不用于放行运动。
- 地图位姿来自 `map → vehicle/base_link` TF；里程计位姿保留自己的 frame。缺少 TF 时不猜测地图位置。
- 地图已知面积是当前栅格中非 unknown 面积，不是整屋覆盖率。
- Nav2 的服务发现只证明接口可见，不能代替生命周期状态；尚未收到目标状态消息时不推断“无活动任务”。
- guard 节点、输出速度与输入推算原因分别报告。推算原因不是 guard 的权威确认；无命令时 `command timeout` 是正常的保护保持零输出情形。保护阈值仍为 1.9 m。
- JSON 顶层 `passed` 表示本次模型/MCP 链路完成，不代表可以安全移动；必须查看各字段的新鲜度。安全动作使用独立的[Robot Skills 六工具入口](robot-skills-mcp.md)，不会从本页的只读入口触发。

模型工具流程依据 [OpenAI 官方 Function calling 文档](https://developers.openai.com/api/docs/guides/function-calling)。本地测试：

```bash
.venv-agent/bin/python -m unittest discover -s workspace/robot_agent -p 'test_*.py' -v
```
