#!/usr/bin/env python3
"""Paired A/B: alternate the shipped Objectives' seed configuration start by start.

The moveit_pro agent re-reads an Objective's XML on every run (verified: changing the file
between two runs changes the covariance that reaches /initialpose with no restart), so the
two arms can be interleaved inside ONE continuous session instead of run as separate
sessions. Host conditions drift over an hour of driving; interleaving makes that drift
common to both arms rather than a difference between them.

Each start's arm is recorded by the recorder itself -- /initialpose carries the covariance
that was in the file -- so the arm labels in the analysis come from the measurement, not
from this script's bookkeeping.
"""
import argparse, json, math, os, re, sys, threading, time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, ReliabilityPolicy, HistoryPolicy)
from nav_msgs.msg import Odometry
from moveit_studio_agent_msgs.msg import Json
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros

LATCHED = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=5)

OBJ_DIR = os.environ.get("OBJ_DIR", "/home/breelynk/user_ws/src/hangar_sim/objectives")
LOADSTEP_FILE = os.environ.get("LOADSTEP_FILE", "/home/breelynk/user_ws/log/.loadstep")
FILES = ["navigate_to_clicked_point.xml", "navigate_to_clicked_point_with_replanning.xml"]
SHIPPED = '<Action ID="SetInitialPose" robot_frame_id="ridgeback_base_link" />'

WAYPOINTS = {
    "A": (3.17, 10.27, math.radians(13.9)),
    "B": (-2.64, 4.91, math.radians(-25.0)),
    "C": (-8.37, 10.61, math.radians(18.4)),
}
CYCLE = ["B", "C", "B", "A"]
RESCUE_XY, RESCUE_YAW = 0.0100, 0.0009


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_arm(arm, xy, yawv):
    """Rewrite both Objectives for this arm. Idempotent: always rebuilt from the shipped form."""
    if arm == "baseline":
        block = SHIPPED
    elif arm == "tight":
        block = ('<Action\n        ID="SetInitialPose"\n'
                 '        robot_frame_id="ridgeback_base_link"\n'
                 f'        xy_variance="{xy}"\n        yaw_variance="{yawv}"\n      />')
    elif arm == "noseed":
        block = None
    else:
        raise SystemExit(f"unknown arm {arm}")
    anchor = '<Control ID="Sequence" name="TopLevelSequence">\n'
    for f in FILES:
        p = os.path.join(OBJ_DIR, f)
        s = open(p).read()
        # Normalise to the shipped single-line form first, re-inserting it if a previous
        # 'noseed' start removed it, so every arm is reached from the same starting text.
        s = re.sub(r'<Action\s+ID="SetInitialPose"[^>]*?/>', SHIPPED, s, flags=re.S)
        if SHIPPED not in s:
            s = s.replace(anchor, anchor + "      " + SHIPPED + "\n", 1)
        if block is None:
            s = s.replace("      " + SHIPPED + "\n", "", 1)
        else:
            s = s.replace(SHIPPED, block, 1)
        open(p, "w").write(s)


class NavAB(Node):
    def __init__(self, frame, click_delay):
        super().__init__("navloop_ab")
        cb = ReentrantCallbackGroup()
        self.frame, self.click_delay = frame, click_delay
        self.goal = None
        self.answered = set()
        self.truth = {}
        self.pubs = {}
        for name, handler in (("get_pose_from_user", self.answer_pose),
                              ("wait_for_user_path_approval", self.answer_approval)):
            base = f"/moveit_pro_ui/{name}"
            self.pubs[name] = self.create_publisher(Json, base + "/response", 10)
            self.create_subscription(
                Json, base + "/request",
                (lambda m, n=name, h=handler: self.on_request(n, h, m)), LATCHED,
                callback_group=cb)
        self.create_subscription(Odometry, "/odom", self.on_truth, SENSOR, callback_group=cb)
        self.ac = ActionClient(self, DoObjectiveSequence, "/do_objective", callback_group=cb)
        # The costmaps keep obstacles stamped from a wrong pose (a separately filed defect),
        # and after enough starts nav2 can no longer plan at all. Clearing them before every
        # start -- identically in every arm -- keeps that defect from ending the session
        # without touching the localization question being measured.
        self.ip = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=20))
        tf2_ros.TransformListener(self.buf, self, spin_thread=False)
        self.clr = [self.create_client(ClearEntireCostmap,
                                       "/global_costmap/clear_entirely_global_costmap",
                                       callback_group=cb),
                    self.create_client(ClearEntireCostmap,
                                       "/local_costmap/clear_entirely_local_costmap",
                                       callback_group=cb)]

    def localization_error(self):
        """How far the filter's estimate is from truth right now, or None."""
        try:
            tr = self.buf.lookup_transform("map", "ridgeback_base_link", rclpy.time.Time(),
                                           rclpy.duration.Duration(seconds=1.0))
        except Exception:
            return None
        if not self.truth:
            return None
        return math.hypot(tr.transform.translation.x - self.truth["x"],
                          tr.transform.translation.y - self.truth["y"])

    def rescue(self):
        """Put the filter back on truth when it is hopelessly lost.

        A divergence nothing recovers from turns every later start into a nav failure rather
        than a trial, and an operator's real answer to a lost robot is a fresh pose estimate.
        Applied identically in every arm."""
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = self.truth["x"]
        m.pose.pose.position.y = self.truth["y"]
        m.pose.pose.orientation.z = math.sin(self.truth["yaw"] / 2.0)
        m.pose.pose.orientation.w = math.cos(self.truth["yaw"] / 2.0)
        # A covariance no arm uses, so the analysis can tell a rescue seed from an
        # Objective's own SetInitialPose and never count one as the other.
        m.pose.covariance[0] = m.pose.covariance[7] = RESCUE_XY
        m.pose.covariance[35] = RESCUE_YAW
        for _ in range(3):
            self.ip.publish(m)
            time.sleep(0.3)

    def clear_costmaps(self):
        for c in self.clr:
            if c.wait_for_service(timeout_sec=3.0):
                f = c.call_async(ClearEntireCostmap.Request())
                t = time.time()
                while not f.done() and time.time() - t < 5.0:
                    time.sleep(0.05)

    def on_truth(self, m):
        self.truth = dict(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                          yaw=yaw_of(m.pose.pose.orientation))

    def on_request(self, name, handler, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        rid = payload.get("request_id")
        if not rid or payload.get("cleared") or "request" not in payload or rid in self.answered:
            return
        self.answered.add(rid)
        time.sleep(self.click_delay)
        out = Json()
        out.data = json.dumps({"request_id": rid, "response": handler(payload["request"])})
        self.pubs[name].publish(out)

    def answer_pose(self, request):
        x, y, yaw = self.goal
        now = self.get_clock().now().to_msg()
        return {"status": {"success": True, "error_message": ""},
                "pose": {"header": {"frame_id": self.frame,
                                    "stamp": {"sec": int(now.sec), "nanosec": int(now.nanosec)}},
                         "pose": {"position": {"x": x, "y": y, "z": 0.0},
                                  "orientation": {"x": 0.0, "y": 0.0,
                                                  "z": math.sin(yaw / 2.0),
                                                  "w": math.cos(yaw / 2.0)}}}}

    def answer_approval(self, request):
        return {"success": True}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objective", default="Navigate to Clicked Point with Replanning")
    ap.add_argument("--starts", type=int, default=60)
    ap.add_argument("--arms", default="baseline,tight")
    ap.add_argument("--xy", default="0.0052")
    ap.add_argument("--yaw", default="0.00085")
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", default="map")
    ap.add_argument("--click-delay", type=float, default=4.0)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--settle", type=float, default=6.0)
    ap.add_argument("--clear-costmaps", type=int, default=1)
    # Signal the host to step its CPU load before each start, and give the simulator time to
    # settle to its new real-time factor before the robot moves.
    ap.add_argument("--load-step", type=int, default=0)
    ap.add_argument("--load-settle", type=float, default=20.0)
    ap.add_argument("--loadstep-file", default=LOADSTEP_FILE)
    ap.add_argument("--rescue-m", type=float, default=2.0)
    a = ap.parse_args()
    arms = a.arms.split(",")

    rclpy.init()
    n = NavAB(a.frame, a.click_delay)
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    if not n.ac.wait_for_server(timeout_sec=90.0):
        print("no /do_objective"); return 2
    t0 = time.time()
    while not n.truth and time.time() - t0 < 30:
        time.sleep(0.1)

    fh = open(a.out, "w")
    for i in range(a.starts):
        arm = arms[i % len(arms)]
        lost = n.localization_error()
        rescued = False
        if a.rescue_m > 0 and lost is not None and lost > a.rescue_m:
            print(f"[{i}] lost by {lost:.1f} m -> rescue", flush=True)
            n.rescue(); time.sleep(6.0); rescued = True
        if a.load_step:
            try:
                with open(a.loadstep_file, "w") as fh2:
                    fh2.write(str(i))
            except OSError as e:
                print(f"load-step signal failed: {e}", flush=True)
            time.sleep(a.load_settle)
        if a.clear_costmaps:
            n.clear_costmaps()
        set_arm(arm, a.xy, a.yaw)
        time.sleep(0.5)
        wp = CYCLE[i % len(CYCLE)]
        n.goal = WAYPOINTS[wp]
        n.answered.clear()
        rec = dict(i=i, arm=arm, wp=wp, goal=list(n.goal), start=dict(n.truth),
                   lost_before=lost, rescued=rescued, t_send=time.time())
        g = DoObjectiveSequence.Goal()
        g.objective_name = a.objective
        g.objective_xml_string = ""
        f = n.ac.send_goal_async(g)
        ts = time.time()
        while not f.done() and time.time() - ts < 30:
            time.sleep(0.05)
        gh = f.result()
        if gh is None or not gh.accepted:
            rec.update(accepted=False, result="rejected", t_end=time.time())
            print(f"[{i}] {arm} REJECTED", flush=True)
            fh.write(json.dumps(rec) + "\n"); fh.flush(); time.sleep(a.settle); continue
        rec["accepted"] = True
        rec["t_accept"] = time.time()
        rf = gh.get_result_async()
        dl = time.time() + a.timeout
        while not rf.done() and time.time() < dl:
            time.sleep(0.05)
        if not rf.done():
            gh.cancel_goal_async(); time.sleep(4); rec["result"] = "timeout"
            print(f"[{i}] {arm} {wp} TIMEOUT", flush=True)
        else:
            r = rf.result()
            rec["result"] = "status=%d" % r.status
            print(f"[{i}] {arm} {wp} status={r.status} "
                  f"dt={time.time()-rec['t_accept']:.0f}s", flush=True)
        rec["t_end"] = time.time()
        rec["end"] = dict(n.truth)
        fh.write(json.dumps(rec) + "\n"); fh.flush()
        time.sleep(a.settle)
    fh.close()
    set_arm("baseline", a.xy, a.yaw)
    print(f"DONE {a.starts} starts -> {a.out}", flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
