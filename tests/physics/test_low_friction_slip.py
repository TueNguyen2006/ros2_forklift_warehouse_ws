from pathlib import Path

import yaml

from forklift_control.contact_model import expected_slip_scale


def test_low_friction_floor_has_higher_expected_slip():
    config = yaml.safe_load(Path("src/forklift_simulation/config/contact_physics.yaml").read_text())
    floors = config["floors"]

    normal_mu = floors["normal_floor"]["longitudinal_friction"]
    low_mu = floors["low_friction_floor"]["longitudinal_friction"]

    assert low_mu < normal_mu
    assert expected_slip_scale(low_mu, normal_mu) > expected_slip_scale(normal_mu, normal_mu)
