#!/usr/bin/env bash
# 只做只读状态查询和小规模GPU计算，不重载模块、不改驱动、不重启电脑。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="${ISAAC_SIM_ROOT:-/home/ubuntu/isaacsim-6.2.0}"
nvidia-smi --query-gpu=name,driver_version,gpu_recovery_action --format=csv,noheader
/usr/bin/python3 "$ROOT/cuda_driver_probe.py"
"$SIM_ROOT/python.sh" --no-ros-env -c 'import torch; torch.cuda.init(); x=torch.arange(16,device="cuda:0",dtype=torch.float32); result=(x*x).sum(); torch.cuda.synchronize(); assert result.item()==1240.; print("Isaac PyTorch CUDA compute OK:",torch.cuda.get_device_name(0))'
