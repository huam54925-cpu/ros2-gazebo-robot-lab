"""Select training compute explicitly; a requested GPU must never silently fall back."""
def resolve_device(requested: str, cuda_available: bool) -> str:
    if requested not in ('cpu', 'cuda', 'auto'):
        raise ValueError('Device must be cpu, cuda or auto')
    if requested == 'cuda' and not cuda_available:
        raise ValueError('CUDA requested but unavailable: use the CUDA trainer image with --gpus all and a compatible NVIDIA driver')
    return 'cuda' if requested == 'cuda' or requested == 'auto' and cuda_available else 'cpu'
