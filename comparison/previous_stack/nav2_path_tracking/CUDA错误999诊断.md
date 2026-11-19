# CUDA 错误 999 诊断（2026-11-12）

## 重启后复检（已恢复）

2026-11-12用户重启后：GPU Recovery Action=None，cuInit=CUDA_SUCCESS，Isaac内置PyTorch GPU实际运算通过。随后真实G1短程导航、取消停车、超时归零及反馈中断保护均已通过，见 `results/isaac_summary.json`。驱动版本保持595.99.02，未重载模块或更换驱动。

以下为重启前的故障记录，保留用于再次出现时对照。

## 重启前已确定的直接原因

重启前 GPU 的 UVM 驱动处于全局致命错误状态，驱动要求重启操作系统。

重启前实测：

```text
GPU: NVIDIA GeForce RTX 5070 Laptop GPU
驱动: 595.99.02
nvidia-smi gpu_recovery_action: Reboot
cuInit(0): 999 / CUDA_ERROR_UNKNOWN
/dev/nvidia0: 可打开
/dev/nvidiactl: 可打开
/dev/nvidia-uvm: EIO / Input/output error
```

用户态 libcuda、nvidia 与 nvidia_uvm 内核模块版本均为595.99.02。独立系统Python直接调用libcuda也失败，因此当前阻塞不局限于PyTorch、Isaac、ONNX或导航代码；设备访问失败是EIO，不能按普通Unix权限不足处理。

## 本机内核证据

时间均为北京时间。2026-11-11 04:33:03记录休眠恢复；随后发生MMU访问故障，并在04:33:10记录：

```text
uvm encountered global fatal error 0x60, requiring os reboot to recover.
Xid ... 154, GPU recovery action changed ... (Node Reboot Required)
```

05:12再次恢复休眠后，同样记录UVM全局致命错误。当时仍是11月8日启动后的同一次系统会话，故障尚未通过重启清除。

Xid154是恢复动作变化的通知，不能单凭它判定最初故障。这里的结论同时依据UVM fatal原文、设备EIO、最小cuInit失败和当前NVML恢复动作Reboot。

可以确定故障与休眠恢复在日志中紧接出现；至于最初MMU故障究竟由驱动、电源管理、应用交互还是其他因素触发，目前没有足够证据下结论，也不能由此判定显卡硬件损坏。

## 同类故障再次出现时的处理顺序

1. 保存正在编辑的内容，正常重启电脑。不要仅重启Isaac、终端或Python。
2. 重启后先运行 `check_cuda.sh`，核对恢复动作、最小驱动初始化及Isaac内置PyTorch的实际GPU计算。
3. 前两层通过后，运行 `test_tracking.sh --backend isaac`，继续第三阶段G1策略/物理闭环验收。
4. 在完成首次验收前，避免在CUDA任务运行中让电脑进入休眠；这是临时测试控制条件，不是已证明的永久修复。
5. 若重启后CUDA立即失败，或休眠恢复后再次失败，再对比本次启动的内核日志和恢复动作，研究电源管理配置。当前不盲目修改休眠配置或更换驱动。

之前提出的“重载UVM”是尚未取得决定性日志时的候选方案；现在已确认Reboot恢复动作，不把模块重载或GPU复位作为当前首选，也没有执行它们。

## 官方依据

NVIDIA nvidia-smi 文档的 GPU Recovery Action → Reboot 明确说明：UVM fatal可能使OS状态不一致，需要重启操作系统，应用不能仅靠重新启动恢复，普通系统重启足够，无需切断电源。

https://docs.nvidia.com/deploy/nvidia-smi/#gpu-recovery-action

## 对项目状态的影响

重启前，CUDA故障阻塞了真实G1验收。重启后已重新运行真实策略与物理测试并通过短程闭环；这一结论来自实际测试，而非仅根据CUDA恢复推断。
