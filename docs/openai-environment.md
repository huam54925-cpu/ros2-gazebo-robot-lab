# OpenAI API 与 MCP 本地环境

2026-10-04 已在 `/mnt/robot_disk/ros_sim` 安装独立 Python 环境，供后续按 **API 连通性 → 只读机器人状态 → 安全动作接口** 接入。

## 当前状态

- Python 3.14.4，环境目录 `.venv-agent/`，不使用 ROS 的系统依赖。
- OpenAI SDK 3.24.0、MCP SDK 2.3.0、python-dotenv 1.2.4、httpx2 2.13.1；直接与间接依赖共 32 个包已锁定版本和下载哈希。
- 项目内 uv 0.12.23 位于 `.tools/uv`，下载缓存也在项目磁盘；没有安装宿主机 pip 或修改 shell 启动文件。
- 初次无密钥 HTTPS 探测返回 401；用户本地配置密钥后，模型列表认证已通过，返回 133 个模型条目。
- `.env` 权限 600，已配置密钥和模型；空模板 `.env.example` 仍不含密钥。已使用用户配置的 `gpt-6.1-sol` 完成一次真实工具调用和中文状态解释。
- [机器人只读状态链路](robot-readonly.md)和[一次受限运动试验](model-motion-trial.md)已运行验证；另有[六工具安全 Frontier MCP](robot-skills-mcp.md)，普通状态查询仍只读，任意坐标导航尚未开放。

## 配置和检查

用本地编辑器打开 `/mnt/robot_disk/ros_sim/.env`，填写 `OPENAI_API_KEY`。密钥应从自己的 OpenAI API 项目获取，不要发到聊天中。配置方法依据 [OpenAI 官方快速入门](https://developers.openai.com/api/docs/quickstart)。

```dotenv
OPENAI_API_KEY=
OPENAI_BASE_URL=https://api.openai.com/v1
OPENAI_MODEL=
```

`OPENAI_MODEL` 留待模型调用阶段选择；当前只读认证检查不需要它。进程中的同名环境变量优先于 `.env`，包括显式空值。检查脚本只读取以上三个配置项，不执行 `.env` 内的 shell 文本。

```bash
cd /mnt/robot_disk/ros_sim

# 仅检查本地依赖和配置；不会发送 API 请求
./scripts/check-agent-env.sh

# 填好密钥后：调用 GET /v1/models 验证认证
./scripts/check-agent-env.sh --api
```

检查结果是 JSON。只有 `api.status` 为 `passed` 且 `authenticated` 为 `true` 才算本次模型列表认证通过；这不证明某个模型有推理权限或可用额度。脚本不打印密钥、模型列表内容或服务端错误正文；不发起生成请求，不传机器人数据，不执行机器人动作。网络请求超时 20 秒、不重试、不跟随重定向，并限制官方 API 地址。

退出码：`0` 为所选检查通过（默认仅本地检查）；`2` 为配置不正确或 `--api` 缺密钥；`3` 为 API/网络错误。

## 重装与验证

安装脚本适用于 Linux x86_64，当前锁文件在宿主机 Python 3.14 上验证。它依赖已有 `python3` 的标准库与 venv；首次安装需要访问 PyPI 包下载服务。重复执行会保留已有 `.env`，并同步**专用**虚拟环境至锁定依赖。

```bash
cd /mnt/robot_disk/ros_sim
./scripts/setup-agent-env.sh
.venv-agent/bin/python -m unittest discover -s workspace/robot_agent -p 'test_*.py' -v
UV_CACHE_DIR="$PWD/.tools/uv-cache" .tools/uv pip check --python .venv-agent/bin/python
```

`.env`、`.venv-agent/`、`.tools/` 均由 Git 忽略，仅提交空模板 `.env.example`。依赖输入与锁文件在 `workspace/robot_agent/`。升级依赖时显式更新输入，再使用项目内 uv 重新生成哈希锁文件并验证；日常安装不会自动升级。

只读状态由仿真容器中的 ROS 订阅桥输出，再供本地 MCP 和模型读取，见[运行说明](robot-readonly.md)。本虚拟环境不包含 `rclpy`。受限运动试验已复用独立 guard 与整条路径检查；安全 Frontier ID 选择已接入统一 Robot Skills；任意坐标导航仍需另行实现和验收。
