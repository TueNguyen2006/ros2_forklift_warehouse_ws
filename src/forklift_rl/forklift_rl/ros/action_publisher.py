from std_msgs.msg import Float64MultiArray


def action_to_target_command(action) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = [float(action[0]), float(action[1])]
    return msg
