set -e
pkill -f rear_steer_sim_only.launch.py || true
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
sleep 2
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /opt/ros/humble/setup.bash
source /home/tuenguyen/ros2_forklift_warehouse_artifacts/install/setup.bash
LOG=/tmp/rear_sim_arc_joint_probe.log
ros2 launch warehouse_visual_localization rear_steer_sim_only.launch.py gui:=false rviz:=false headless:=true > "$LOG" 2>&1 &
LPID=$!
sleep 18
python3 - <<'PY'
import math
import time
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class Probe(Node):
    def __init__(self):
        super().__init__('arc_joint_probe')
        self.pub = self.create_publisher(Twist, '/realistic_nav/cmd_vel_request', 10)
        self.samples = []
        self.traction_cmds = []
        self.steering_cmds = []
        self.create_subscription(JointState, '/joint_states', self.cb, 50)
        self.create_subscription(
            Float64MultiArray,
            '/traction_velocity_controller/commands',
            self.traction_cb,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            '/rear_steering_position_controller/commands',
            self.steering_cb,
            20,
        )

    def cb(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        required = [
            'rear_left_wheel_spin_joint',
            'rear_right_wheel_spin_joint',
            'rear_left_steering_joint',
            'rear_right_steering_joint',
        ]
        if not all(name in positions or name in velocities for name in required):
            return
        if required[0] not in velocities or required[1] not in velocities:
            return
        if required[2] not in positions or required[3] not in positions:
            return
        self.samples.append(
            (
                time.time(),
                velocities[required[0]],
                velocities[required[1]],
                positions[required[2]],
                positions[required[3]],
            )
        )

    def traction_cb(self, msg: Float64MultiArray):
        self.traction_cmds.append((time.time(), *list(msg.data)))

    def steering_cb(self, msg: Float64MultiArray):
        self.steering_cmds.append((time.time(), *list(msg.data)))

    def cmd(self, vx: float, wz: float, sec: float):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        end_time = time.time() + sec
        while time.time() < end_time:
            self.pub.publish(msg)
            time.sleep(0.05)
        self.pub.publish(Twist())


rclpy.init()
node = Probe()
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()
time.sleep(3.0)
node.cmd(0.05, 0.017, 3.5)
time.sleep(1.0)
tail = node.samples[-20:]
print('TAIL', tail)
if node.samples:
    print(
        'MAX_WHEEL_ALL',
        max(abs(sample[1]) for sample in node.samples),
        max(abs(sample[2]) for sample in node.samples),
    )
    print(
        'MAX_STEER_ALL',
        max(abs(sample[3]) for sample in node.samples),
        max(abs(sample[4]) for sample in node.samples),
    )
if node.traction_cmds:
    print('TRACTION_CMD_MAX', max(abs(s[1]) for s in node.traction_cmds), max(abs(s[2]) for s in node.traction_cmds))
    print('TRACTION_CMD_TAIL', node.traction_cmds[-10:])
if node.steering_cmds:
    print('STEERING_CMD_MAX', max(abs(s[1]) for s in node.steering_cmds), max(abs(s[2]) for s in node.steering_cmds))
    print('STEERING_CMD_TAIL', node.steering_cmds[-10:])
node.destroy_node()
rclpy.shutdown()
PY
kill $LPID || true
wait $LPID || true
