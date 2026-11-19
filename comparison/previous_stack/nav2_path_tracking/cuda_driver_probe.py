"""独立系统 Python 的 CUDA 初始化探针，不依赖 PyTorch 或 Isaac。"""
import ctypes
import sys

cuda = ctypes.CDLL('libcuda.so.1')
result = cuda.cuInit(0)
name = ctypes.c_char_p()
cuda.cuGetErrorName(result, ctypes.byref(name))
print('cuInit:', result, name.value.decode() if name.value else 'unknown')
sys.exit(0 if result == 0 else 1)
