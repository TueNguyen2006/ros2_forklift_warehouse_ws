set -e
pkill -f rear_steer_truth_mode.launch.py || true
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
sleep 2
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /opt/ros/humble/setup.bash
source /home/tuenguyen/ros2_forklift_warehouse_artifacts/install/setup.bash
LOG=/tmp/rear_truth_goal_recheck.log
ros2 launch warehouse_visual_localization rear_steer_truth_mode.launch.py gui:=false rviz:=false headless:=true use_stability_guard:=false use_collision_monitor:=false > "$LOG" 2>&1 &
LPID=$!
sleep 52
python3 - <<'PY'
import time, threading, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import NavigateToPose

class Probe(Node):
    def __init__(self):
        super().__init__('truth_goal_probe')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cmds = []
        self.plan = None
        self.odom = []
        self.create_subscription(Twist, '/realistic_nav/cmd_vel_request', self.cmd_cb, 20)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
    def cmd_cb(self, msg):
        self.cmds.append((time.time(), msg.linear.x, msg.angular.z))
    def plan_cb(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    def odom_cb(self, msg):
        self.odom.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

rclpy.init(); node = Probe(); th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True); th.start()
ready = node.client.wait_for_server(timeout_sec=20.0)
print('READY', ready)
if ready:
    goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id='map'; goal.pose.header.stamp=node.get_clock().now().to_msg(); goal.pose.pose.position.x=-4.0; goal.pose.pose.position.y=-1.0; goal.pose.pose.orientation.w=1.0
    fut = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    gh = fut.result(); print('ACCEPTED', bool(gh and gh.accepted))
    time.sleep(10.0)
nonzero = [(round(vx,3), round(wz,3)) for _,vx,wz in node.cmds if abs(vx)>1e-4 or abs(wz)>1e-4]
print('CMDS', len(nonzero), nonzero[:12])
if node.plan:
    print('PLAN_LEN', len(node.plan))
    print('PLAN_HEAD', [tuple(round(v,3) for v in pt) for pt in node.plan[:8]])
    print('PLAN_TAIL', [tuple(round(v,3) for v in pt) for pt in node.plan[-8:]])
else:
    print('PLAN_LEN', 0)
if len(node.odom) >= 2:
    sx,sy = node.odom[0]; ex,ey = node.odom[-1]; print('ODOM_DELTA', round(ex-sx,3), round(ey-sy,3), round(math.hypot(ex-sx, ey-sy),3))
node.destroy_node(); rclpy.shutdown()
PY
kill $LPID || true
wait $LPID || true
