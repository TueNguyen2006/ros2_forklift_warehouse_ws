set -e
pkill -f rear_steer_sim_only.launch.py || true
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
sleep 2
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /opt/ros/humble/setup.bash
source /home/tuenguyen/ros2_forklift_warehouse_artifacts/install/setup.bash
LOG=/tmp/rear_sim_arc_recheck.log
ros2 launch warehouse_visual_localization rear_steer_sim_only.launch.py gui:=false rviz:=false headless:=true > "$LOG" 2>&1 &
LPID=$!
sleep 18
python3 - <<'PY'
import math, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState

class Probe(Node):
    def __init__(self):
        super().__init__('arc_probe')
        self.pub = self.create_publisher(Twist, '/realistic_nav/cmd_vel_request', 10)
        self.model = None
        self.bad = 0
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_cb, 20)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)
    def model_cb(self, msg):
        if 'forklift_baseline' in msg.name:
            i = msg.name.index('forklift_baseline')
            p = msg.pose[i].position
            self.model = (p.x, p.y, p.z)
    def joint_cb(self, msg):
        for v in msg.velocity:
            if not math.isfinite(v):
                self.bad += 1
    def cmd(self, vx, wz, sec):
        msg = Twist(); msg.linear.x = vx; msg.angular.z = wz
        t = time.time() + sec
        while time.time() < t:
            self.pub.publish(msg)
            time.sleep(0.05)
        self.pub.publish(Twist())

rclpy.init(); node = Probe(); th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True); th.start(); time.sleep(3)
start = node.model
node.cmd(0.05, 0.017, 3.5)
time.sleep(1)
end = node.model
print('START', start)
print('END', end)
print('BAD', node.bad)
node.destroy_node(); rclpy.shutdown()
PY
kill $LPID || true
wait $LPID || true
