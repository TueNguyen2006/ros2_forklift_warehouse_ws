set -e
pkill -f rear_steer_sim_only.launch.py || true
pkill -x gzserver || true
pkill -x gzclient || true
sleep 2
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /opt/ros/humble/setup.bash
source /home/tuenguyen/ros2_forklift_warehouse_artifacts/install/setup.bash
LOG=/tmp/rear_sim_straight_speed.log
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
        super().__init__('straight_speed_probe')
        self.pub = self.create_publisher(Twist, '/realistic_nav/cmd_vel_request', 10)
        self.model = []
        self.left = []
        self.right = []
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_cb, 20)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)
    def model_cb(self, msg):
        if 'forklift_baseline' in msg.name:
            i = msg.name.index('forklift_baseline')
            p = msg.pose[i].position
            self.model.append((time.time(), p.x, p.y, p.z))
    def joint_cb(self, msg):
        data = dict(zip(msg.name, msg.velocity))
        if 'rear_left_wheel_spin_joint' in data:
            self.left.append((time.time(), data['rear_left_wheel_spin_joint']))
        if 'rear_right_wheel_spin_joint' in data:
            self.right.append((time.time(), data['rear_right_wheel_spin_joint']))
    def cmd(self, vx, wz, sec):
        m = Twist(); m.linear.x=vx; m.angular.z=wz
        end = time.time()+sec
        while time.time()<end:
            self.pub.publish(m); time.sleep(0.05)
        self.pub.publish(Twist())

def delta(samples):
    a,b=samples[0],samples[-1]
    return round(b[1]-a[1],4), round(b[2]-a[2],4)

rclpy.init(); n=Probe(); th=threading.Thread(target=rclpy.spin,args=(n,),daemon=True); th.start(); time.sleep(3)
start=len(n.model)
n.cmd(0.10,0.0,4.0)
time.sleep(1.0)
seg=n.model[start:]
if len(seg)>=2:
    dx,dy=delta(seg)
    print('MODEL_DELTA', dx, dy, round((dx*dx+dy*dy)**0.5,4))
print('LEFT_VEL_LAST10', [round(v,3) for _,v in n.left[-10:]])
print('RIGHT_VEL_LAST10', [round(v,3) for _,v in n.right[-10:]])
n.destroy_node(); rclpy.shutdown()
PY
kill $LPID || true
wait $LPID || true
