set -e
pkill -f rear_steer_truth_mode.launch.py || true
pkill -f rear_steer_sim_only.launch.py || true
pkill -f warehouse_nav_realistic.launch.py || true
pkill -f warehouse_nav_realistic_truth.launch.py || true
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
sleep 2
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /opt/ros/humble/setup.bash
source /home/tuenguyen/ros2_forklift_warehouse_artifacts/install/setup.bash
LOG=/tmp/rear_truth_disappear_probe.log
ros2 launch warehouse_visual_localization rear_steer_truth_mode.launch.py gui:=false rviz:=false headless:=true use_stability_guard:=false use_collision_monitor:=false > "$LOG" 2>&1 &
LPID=$!
sleep 52
python3 - <<'PY'
import time, threading, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose

class Probe(Node):
    def __init__(self):
        super().__init__('disappear_probe')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.model = []
        self.odom = []
        self.cmds = []
        self.plan = []
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_cb, 20)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(Twist, '/realistic_nav/cmd_vel_request', self.cmd_cb, 20)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
    def model_cb(self, msg):
        if 'forklift_baseline' in msg.name:
            i = msg.name.index('forklift_baseline')
            p = msg.pose[i].position
            q = msg.pose[i].orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            self.model.append((time.time(), p.x, p.y, p.z, yaw))
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom.append((time.time(), p.x, p.y))
    def cmd_cb(self, msg):
        self.cmds.append((time.time(), msg.linear.x, msg.angular.z))
    def plan_cb(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

rclpy.init(); node = Probe(); th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True); th.start()
ready = node.client.wait_for_server(timeout_sec=20.0)
print('READY', ready)
if ready:
    goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id='map'; goal.pose.header.stamp=node.get_clock().now().to_msg(); goal.pose.pose.position.x=-4.0; goal.pose.pose.position.y=-1.0; goal.pose.pose.orientation.w=1.0
    fut = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    gh = fut.result(); print('ACCEPTED', bool(gh and gh.accepted))
    time.sleep(12.0)
print('PLAN_LEN', len(node.plan))
print('PLAN_HEAD', [tuple(round(v,3) for v in pt) for pt in node.plan[:8]])
print('CMDS_NONZERO', [(round(vx,3), round(wz,3)) for _,vx,wz in node.cmds if abs(vx)>1e-4 or abs(wz)>1e-4][:20])
print('MODEL_LAST', node.model[-5:])
print('ODOM_LAST', node.odom[-5:])
if node.model:
    zeros = sum(1 for _,x,y,z,_ in node.model if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9)
    print('MODEL_ZERO_SAMPLES', zeros)
node.destroy_node(); rclpy.shutdown()
PY
kill $LPID || true
wait $LPID || true
printf '\n===== ERRORS =====\n'
grep -nE 'Registration failed|VWDictionary|\[ERROR\]|\[Err\]|nan|NaN|Segmentation|EXCEPTION|out of bounds' "$LOG" || true
