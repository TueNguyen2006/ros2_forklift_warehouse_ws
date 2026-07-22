from warehouse_visual_localization.core.human_ssm import HumanSSMConfig, evaluate_human_ssm


def test_ssm_far_human_is_clear_and_near_human_limits_speed():
    cfg = HumanSSMConfig()
    far = evaluate_human_ssm(5.0, 0.3, cfg)
    near = evaluate_human_ssm(0.65, 0.3, cfg)
    assert far.safety_state == "clear"
    assert near.allowed_speed < far.allowed_speed


def test_ssm_margin_and_invalid_braking_fail_safe():
    assert evaluate_human_ssm(0.5, 0.2).safety_state == "stop"
    invalid = evaluate_human_ssm(2.0, 0.2, HumanSSMConfig(braking_deceleration=0.0))
    assert invalid.safety_state == "invalid"
    assert invalid.allowed_speed == 0.0
