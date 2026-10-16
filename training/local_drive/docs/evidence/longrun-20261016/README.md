# 验收记录

最终交付镜像ID见 images.json，逐文件源码哈希见 release-manifest.json。

- `container-tests-final.txt`：修复配置隔离后的43项完整测试。
- `preflight-final.json`：最终镜像的本机CUDA预检。
- `host-tests.txt`：宿主12项调度器测试，最终修改未涉及调度器。
- 没有final后缀的早期测试、preflight和vehicle-training记录来自第一次构建，不能当作最终镜像实测。
- `vehicle-acceptance.txt`：首次初始化失败；`vehicle-acceptance-retry.txt`：早期镜像全新回合通过。
- `pre-fix-*`：跨进程续跑暴露Configuration mismatch的原始失败证据。

测试只证明记录范围内的行为，不证明三小时稳定性、异机可用性或导航性能改善。测试用临时模型与Replay Buffer在提取清单和日志后清理，不作为训练成果交付。

`scheduler-smoke.json`、`scheduler-events.jsonl`和cold-stop/resume-sigterm记录来自最终镜像，实际覆盖0→8步STOP、8→16步跨进程续跑与SIGTERM、零残留容器。

`vehicle-training-final.json`和`vehicle-acceptance-final.txt`来自最终镜像：CUDA采样64步、48次梯度更新，保存重载后再采样8步，共72条Replay数据，通过。
