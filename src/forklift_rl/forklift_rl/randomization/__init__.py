import random


def sample_uniform_ranges(config: dict, seed: int | None = None) -> dict:
    rng = random.Random(seed)
    result = {}
    for name, value in config.items():
        if isinstance(value, list) and len(value) == 2:
            result[name] = rng.uniform(float(value[0]), float(value[1]))
    return result
