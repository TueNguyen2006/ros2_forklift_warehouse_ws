import numpy as np


try:
    from gymnasium import spaces
except Exception:  # pragma: no cover - optional local dependency.
    class _Box:
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.low = np.array(low if shape is None else np.full(shape, low), dtype=dtype)
            self.high = np.array(high if shape is None else np.full(shape, high), dtype=dtype)
            self.shape = self.low.shape
            self.dtype = dtype

        def sample(self):
            return np.random.uniform(self.low, self.high).astype(self.dtype)

    class _Spaces:
        Box = _Box

    spaces = _Spaces()
