def expected_slip_scale(friction_mu: float, reference_mu: float = 0.9) -> float:
    if friction_mu <= 0.0:
        raise ValueError("friction_mu must be positive")
    if reference_mu <= 0.0:
        raise ValueError("reference_mu must be positive")
    return reference_mu / friction_mu
