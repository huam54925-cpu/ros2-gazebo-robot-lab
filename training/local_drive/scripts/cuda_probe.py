"""Verify CUDA kernels and autograd; run inside the trainer with --gpus all."""
import json
import subprocess
import torch

assert torch.cuda.is_available(), 'CUDA is unavailable in this container'
x = torch.ones((32, 32), device='cuda', requires_grad=True)
y = x @ torch.ones_like(x)
y.sum().backward()
torch.cuda.synchronize()
assert torch.all(y == 32).item()
assert torch.all(x.grad == 32).item()
report = {
    'status': 'passed',
    'torch': torch.__version__,
    'cuda_runtime': torch.version.cuda,
    'gpu_name': torch.cuda.get_device_name(0),
    'compute_capability': list(torch.cuda.get_device_capability(0)),
    'compiled_architectures': torch.cuda.get_arch_list(),
    'tensor_device': str(y.device),
    'matrix_multiply_and_backward': 'passed',
    'driver': subprocess.check_output([
        'nvidia-smi', '--query-gpu=driver_version', '--format=csv,noheader'
    ], text=True).strip(),
}
print(json.dumps(report, indent=2))
