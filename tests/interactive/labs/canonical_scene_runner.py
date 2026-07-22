"""Scene primitives shared by dedicated canonical forklift labs."""
from __future__ import annotations
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT / "src" / "warehouse_visual_localization")); sys.path.insert(0,str(ROOT / "tests" / "interactive"))
from common import button, draw_forklift, parser, setup_figure, show_or_smoke
from warehouse_visual_localization.core import ControlCommand, FourWheelForkliftModel, Pose2D, VehicleState
from warehouse_visual_localization.core.cbf_filter import filter_linear_cbf_command
from warehouse_visual_localization.core.corridor_geometry import CorridorBoundary, evaluate_corridor
from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig, curvature_speed_limit
from warehouse_visual_localization.core.docking_geometry import DockingConfig, evaluate_docking
from warehouse_visual_localization.core.human_ssm import evaluate_human_ssm
from warehouse_visual_localization.core.rollover_safety import RolloverConfig, evaluate_rollover
from warehouse_visual_localization.core.visual_servo import planar_pbvs_command


def _motion_scene(title, step_fn, draw_fn):
    args=parser(title).parse_args(); import matplotlib.pyplot as plt
    state={"robot":VehicleState(Pose2D(-1.5,0,0)),"trail":[],"play":False}; model=FourWheelForkliftModel(); fig,ax=setup_figure(title)
    play,step,reset=button(fig,0,"Play"),button(fig,1,"Step"),button(fig,2,"Reset")
    def redraw():
        ax.clear(); ax.grid(True); ax.set_aspect("equal"); ax.set_xlim(-2,3); ax.set_ylim(-2,2)
        draw_fn(ax,state,model); fig.canvas.draw_idle()
    def advance(_=None):
        step_fn(state,model); state["trail"].append((state["robot"].pose.x,state["robot"].pose.y)); redraw()
    def toggle(_): state["play"]=not state["play"]; play.label.set_text("Pause" if state["play"] else "Play")
    def restart(_): state.update(robot=VehicleState(Pose2D(-1.5,0,0)),trail=[],play=False); play.label.set_text("Play"); redraw()
    timer=fig.canvas.new_timer(interval=80); timer.add_callback(lambda: advance() if state["play"] else None); timer.start()
    play.on_clicked(toggle); step.on_clicked(advance); reset.on_clicked(restart); redraw()
    if args.smoke: advance()
    show_or_smoke(fig,args.smoke)


def curvature_speed():
    def step(s,m):
        x=s["robot"].pose.x; k=0 if x<-.2 else 1.3; v=min(.32,curvature_speed_limit([k],CurvatureSpeedConfig()).speed_limit[0]); s["robot"]=m.step(s["robot"],ControlCommand(v=v,steer=-math.atan(1.25*k)),.08)
    def draw(ax,s,m):
        ax.plot([-2,-.2,.5,1.2,2.5],[0,0,.15,1,1],"k--",label="planned path"); draw_forklift(ax,s["robot"]); ax.plot(*zip(*s["trail"]),color="tab:blue") if s["trail"] else None; ax.axvspan(-.2,.6,color="tab:orange",alpha=.15); ax.set_title("Curve speed scheduling: orange zone reduces speed before the turn"); ax.legend()
    _motion_scene("Curvature-Speed Forklift Lab",step,draw)


def corridor():
    def step(s,m): s["robot"]=m.step(s["robot"],ControlCommand(v=.22,steer=math.radians(18)),.08)
    def draw(ax,s,m):
        r=evaluate_corridor([(0.35,.5),(.35,-.5),(-1.1,-.5),(-1.1,.5)],s["robot"].pose,CorridorBoundary((0,1),1),CorridorBoundary((0,1),-1)); ax.axhline(1,color="black",lw=2);ax.axhline(-1,color="black",lw=2); draw_forklift(ax,s["robot"],"tab:green" if r.feasible else "tab:red"); ax.scatter([s["robot"].pose.x],[s["robot"].pose.y],color="blue",label="base center"); ax.set_title(f"Full footprint margin={r.minimum_margin:.2f} m (red means a corner crossed)");ax.legend()
    _motion_scene("Full-Footprint Corridor Lab",step,draw)


def rollover():
    def step(s,m): s["robot"]=m.step(s["robot"],ControlCommand(v=.32,steer=math.radians(-30)),.08)
    def draw(ax,s,m):
        r=evaluate_rollover(s["robot"].v,1.3,0,[(-.6,-.5),(.6,-.5),(.6,.5),(-.6,.5)],RolloverConfig(vehicle_cog_height=.85,safety_factor=.5)); draw_forklift(ax,s["robot"]); ax.arrow(s["robot"].pose.x,s["robot"].pose.y,0,r.lateral_acceleration*.2,color="tab:red",width=.02,label="lateral acceleration");ax.set_title(f"High-CoG turn: rollover utilization={r.utilization_ratio:.0%}");ax.legend()
    _motion_scene("Quasi-static Rollover Lab",step,draw)


def human_ssm():
    def step(s,m):
        d=max(.35,1.7-s["robot"].pose.x); allowed=evaluate_human_ssm(d,s["robot"].v).allowed_speed;s["robot"]=m.step(s["robot"],ControlCommand(v=min(.32,allowed)),.08)
    def draw(ax,s,m):
        human_x=1.7;d=human_x-s["robot"].pose.x;result=evaluate_human_ssm(max(0,d),s["robot"].v); ax.axvline(human_x,color="tab:red",lw=3,label="human crossing");ax.axvspan(human_x-result.stopping_distance,human_x,color="tab:red",alpha=.15,label="stopping zone");draw_forklift(ax,s["robot"]);ax.set_title(f"SSM: distance={d:.2f}m, allowed speed={result.allowed_speed:.2f}m/s");ax.legend()
    _motion_scene("Human SSM Forklift Lab",step,draw)


def cbf():
    def step(s,m):
        safe=filter_linear_cbf_command([.32,0],[[1,0],[-1,0]],[.12,.12],[-.35,-.5],[.35,.5]).safe_command[0];s["robot"]=m.step(s["robot"],ControlCommand(v=safe),.08)
    def draw(ax,s,m):
        ax.axvspan(.2,3,color="tab:red",alpha=.12,label="unsafe raw-command region");ax.plot([-1.5,1.5],[0,0],"--",color="tab:orange",label="raw v=0.32 path");draw_forklift(ax,s["robot"]);ax.set_title("CBF: solid forklift follows filtered v=0.12, not raw v=0.32");ax.legend()
    _motion_scene("CBF Command Filter Lab",step,draw)


def docking():
    def step(s,m): s["robot"]=m.step(s["robot"],ControlCommand(v=.12,steer=0),.08)
    def draw(ax,s,m):
        from matplotlib.patches import Rectangle
        r=evaluate_docking(s["robot"].pose.y,-s["robot"].pose.yaw,DockingConfig(insertion_length=1,pocket_width=.08,clearance_margin=.01));ax.add_patch(Rectangle((1,-.04),.6,.08,color="saddlebrown",alpha=.7));ax.axvline(1,color="tab:green",ls="--",label="insertion line");draw_forklift(ax,s["robot"],"tab:green" if r.docking_feasible else "tab:red");ax.set_title(f"Fork-tip docking: {'PASS' if r.docking_feasible else 'FAIL'} exact tip error={r.exact_tip_error:.3f}m");ax.legend()
    _motion_scene("Fork-Tip Docking Lab",step,draw)


def visual_servo():
    def step(s,m):
        u=planar_pbvs_command([s["robot"].pose.x,s["robot"].pose.y,s["robot"].pose.yaw],[1,.25,0],[.3,.8,1]);s["robot"]=m.step(s["robot"],ControlCommand(v=max(0,min(.2,u.command[0])),steer=max(-.4,min(.4,u.command[2]))),.08)
    def draw(ax,s,m):
        target=(1,.25);ax.scatter(*target,marker="*",s=250,color="tab:green",label="camera feature target");draw_forklift(ax,s["robot"]);ax.arrow(s["robot"].pose.x,s["robot"].pose.y,target[0]-s["robot"].pose.x,target[1]-s["robot"].pose.y,color="tab:purple",width=.01,label="feature error");ax.set_title("Planar visual servo: correction drives forklift feature to target");ax.legend()
    _motion_scene("Planar Visual-Servo Docking Lab",step,draw)
