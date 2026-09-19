#!/usr/bin/env python3
"""recorder2.py + YAW and particle-DIVERSITY instrumentation.

Everything recorder2.py recorded is still recorded (error at the robot at matched stamps,
cloud spread, AMCL covariance, /initialpose events, costmap lethal counts). Added here:

  err_yaw         estimated yaw minus truth yaw, at the estimate's OWN stamp, truth
                  interpolated to it.  (recorder2 already computed this; nothing above
                  anal2.py's one summary column ever looked at it.)
  cloud.ess_w     effective sample size from the published particle weights, N/(sum w^2)
                  with w normalised -- the quantity selective_resampling gates on.
  cloud.ess_dup   effective sample size from duplicate multiplicities, n^2/sum(c_i^2).
                  After a resample the surviving particles are copies; this counts them.
  cloud.yaw_r95   circular 95th-percentile |yaw - cloud mean yaw|: the cloud's HEADING
                  spread, which is what a spin depletes and what position r95 hides.
  amcl_upd        a monotone counter of distinct map->odom updates, so update (and hence
                  resample) rate is readable per run.
"""
import argparse, json, math, signal, sys, time, threading
from collections import deque, Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
from geometry_msgs.msg import PoseArray, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid

try:
    from nav2_msgs.msg import ParticleCloud
except Exception:
    ParticleCloud = None

MAP, BASE = "map", "ridgeback_base_link"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Rec(Node):
    def __init__(self):
        super().__init__("nav_recorder4")
        cb = ReentrantCallbackGroup()
        self.truth_hist = deque(maxlen=12000)
        self.cloud = None          # list of (x, y, yaw, weight-or-None)
        self.cloud_src = None
        self.cmd = (0.0, 0.0, 0.0)
        self.gobs = self.lobs = None
        self.rows = []
        self.events = []
        self.apose = None
        self.wodom = self.wodom0 = self.fodom = None
        self.upd = 0
        self._last_mo = None
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, "/odom", self.on_truth, sensor, callback_group=cb)
        # Subscribe the cloud under both possible types; only the matching one ever fires.
        self.create_subscription(PoseArray, "/particle_cloud", self.on_cloud_pa, sensor, callback_group=cb)
        if ParticleCloud is not None:
            self.create_subscription(ParticleCloud, "/particle_cloud", self.on_cloud_pc, sensor, callback_group=cb)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_cmd, 10, callback_group=cb)
        self.create_subscription(OccupancyGrid, "/global_costmap/obstacle_layer", self.on_g, latched, callback_group=cb)
        self.create_subscription(OccupancyGrid, "/local_costmap/obstacle_layer", self.on_l, latched, callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10, callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_apose, sensor, callback_group=cb)
        # The odom side of the chain. AMCL's motion model is driven by whatever yaw these
        # report, so an odometry that over- or under-reports rotation is a bias the filter
        # has to fight with its measurement -- exactly what a spin would expose.
        self.create_subscription(Odometry, "/platform_velocity_controller_nav2/odom",
                                 lambda m: self.__setattr__("wodom", m), sensor, callback_group=cb)
        self.create_subscription(Odometry, "/platform_velocity_controller/odom",
                                 lambda m: self.__setattr__("wodom0", m), sensor, callback_group=cb)
        self.create_subscription(Odometry, "/odom_filtered",
                                 lambda m: self.__setattr__("fodom", m), sensor, callback_group=cb)
        self.buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.buf, self, spin_thread=False)

    def on_truth(self, m):
        t = rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.truth_hist.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                                yaw_of(m.pose.pose.orientation)))

    def on_cloud_pa(self, m):
        self.cloud_src = "PoseArray"
        self.cloud = [(p.position.x, p.position.y, yaw_of(p.orientation), None) for p in m.poses]

    def on_cloud_pc(self, m):
        self.cloud_src = "ParticleCloud"
        self.cloud = [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation), p.weight)
                      for p in m.particles]

    def on_apose(self, m): self.apose = m

    def on_initialpose(self, m):
        ev = dict(ev="initialpose", wall=time.time(),
                  stamp=rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9,
                  frame=m.header.frame_id,
                  x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                  yaw=yaw_of(m.pose.pose.orientation),
                  cov_xx=m.pose.covariance[0], cov_yy=m.pose.covariance[7],
                  cov_aa=m.pose.covariance[35], n_rows=len(self.rows))
        g = self.truth_at(ev["stamp"]) or (self.truth_hist[-1][1:] if self.truth_hist else None)
        if g:
            ev.update(tx=g[0], ty=g[1], tyaw=g[2],
                      seed_err_d=math.hypot(ev["x"] - g[0], ev["y"] - g[1]),
                      seed_err_yaw=wrap(ev["yaw"] - g[2]))
        self.events.append(ev)

    def on_cmd(self, m): self.cmd = (m.linear.x, m.linear.y, m.angular.z)
    def on_g(self, m): self.gobs = m
    def on_l(self, m): self.lobs = m

    def truth_at(self, when):
        h = list(self.truth_hist)
        if len(h) < 2 or when < h[0][0] or when > h[-1][0]:
            return None
        lo = hi = None
        for s in h:
            if s[0] <= when:
                lo = s
            else:
                hi = s
                break
        if lo is None or hi is None:
            return None
        span = hi[0] - lo[0]
        f = 0.0 if span <= 0 else (when - lo[0]) / span
        return (lo[1] + f * (hi[1] - lo[1]), lo[2] + f * (hi[2] - lo[2]),
                lo[3] + f * wrap(hi[3] - lo[3]))

    def cloud_stats(self):
        c = self.cloud
        if not c:
            return None
        n = len(c)
        xs = [p[0] for p in c]; ys = [p[1] for p in c]; yaws = [p[2] for p in c]
        mx = sum(xs)/n; my = sum(ys)/n
        cs = sum(math.cos(a) for a in yaws)/n; sn = sum(math.sin(a) for a in yaws)/n
        R = math.hypot(cs, sn)
        myaw = math.atan2(sn, cs)
        d = sorted(math.hypot(x-mx, y-my) for x, y in zip(xs, ys))
        # circular spread about the cloud's own mean heading -- the quantity a spin depletes
        dy = sorted(abs(wrap(a - myaw)) for a in yaws)
        mult = Counter((round(x, 6), round(y, 6), round(a, 6)) for x, y, a in zip(xs, ys, yaws))
        ess_dup = (n * n) / sum(v * v for v in mult.values())
        out = dict(n=n, r95=d[int(0.95*(n-1))], rmax=d[-1], mx=mx, my=my, myaw=myaw,
                   yaw_std=(math.sqrt(-2.0*math.log(R)) if R > 1e-12 else math.pi),
                   yaw_r95=dy[int(0.95*(n-1))], yaw_max=dy[-1],
                   uniq_frac=len(mult)/n, ess_dup=ess_dup)
        ws = [p[3] for p in c if p[3] is not None]
        if len(ws) == n and sum(ws) > 0:
            s = sum(ws)
            out["ess_w"] = (s * s) / sum(w * w for w in ws)
        return out

    def sample(self):
        if not self.truth_hist:
            return
        row = dict(t=self.truth_hist[-1][0], wall=time.time(),
                   vx=self.cmd[0], vy=self.cmd[1], wz=self.cmd[2])
        try:
            tr = self.buf.lookup_transform(MAP, BASE, rclpy.time.Time(),
                                           rclpy.duration.Duration(seconds=0.05))
            st = rclpy.time.Time.from_msg(tr.header.stamp).nanoseconds * 1e-9
            ex, ey = tr.transform.translation.x, tr.transform.translation.y
            eyaw = yaw_of(tr.transform.rotation)
            g = self.truth_at(st)
            row["est_stamp"] = st
            if g:
                row.update(tx=g[0], ty=g[1], tyaw=g[2], ex=ex, ey=ey, eyaw=eyaw,
                           err_d=math.hypot(ex-g[0], ey-g[1]), err_yaw=wrap(eyaw-g[2]))
        except Exception:
            pass
        try:
            mo = self.buf.lookup_transform(MAP, "odom", rclpy.time.Time(),
                                           rclpy.duration.Duration(seconds=0.05))
            mod = (round(mo.transform.translation.x, 9), round(mo.transform.translation.y, 9),
                   round(yaw_of(mo.transform.rotation), 9))
            if self._last_mo is not None and mod != self._last_mo:
                self.upd += 1
            self._last_mo = mod
            row["mo_x"] = mo.transform.translation.x
            row["mo_y"] = mo.transform.translation.y
            row["mo_d"] = math.hypot(mo.transform.translation.x, mo.transform.translation.y)
            row["mo_yaw"] = yaw_of(mo.transform.rotation)
            row["mo_stamp"] = rclpy.time.Time.from_msg(mo.header.stamp).nanoseconds * 1e-9
        except Exception:
            pass
        row["amcl_upd"] = self.upd
        for nm, gg in (("gobs", self.gobs), ("lobs", self.lobs)):
            if gg is not None:
                row[nm+"_lethal"] = sum(1 for v in gg.data if v >= 90)
        cs = self.cloud_stats()
        if cs:
            row["cloud"] = cs
        if self.apose is not None:
            a = self.apose
            ast = rclpy.time.Time.from_msg(a.header.stamp).nanoseconds * 1e-9
            d = dict(x=a.pose.pose.position.x, y=a.pose.pose.position.y,
                     yaw=yaw_of(a.pose.pose.orientation), stamp=ast,
                     cxx=a.pose.covariance[0], cyy=a.pose.covariance[7],
                     caa=a.pose.covariance[35])
            g = self.truth_at(ast)
            if g:
                d["err_d"] = math.hypot(d["x"]-g[0], d["y"]-g[1])
                d["err_yaw"] = wrap(d["yaw"]-g[2])
            row["ap"] = d
        for nm, o in (("wo", self.wodom), ("wo0", self.wodom0), ("fo", self.fodom)):
            if o is not None:
                row[nm] = dict(x=o.pose.pose.position.x, y=o.pose.pose.position.y,
                               yaw=yaw_of(o.pose.pose.orientation),
                               stamp=rclpy.time.Time.from_msg(o.header.stamp).nanoseconds * 1e-9)
        row["n_ip"] = len(self.events)
        self.rows.append(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--label", default="rec")
    a = ap.parse_args()
    rclpy.init()
    n = Rec()
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    stop = {"v": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
    fh = open(a.out + "l", "w")
    end = time.time() + a.duration
    seen = seen_ev = 0
    while time.time() < end and not stop["v"]:
        n.sample()
        while seen_ev < len(n.events):
            fh.write(json.dumps(n.events[seen_ev]) + "\n"); seen_ev += 1
        while seen < len(n.rows):
            fh.write(json.dumps(n.rows[seen]) + "\n"); seen += 1
        fh.flush()
        time.sleep(0.05)
    grids = {}
    for nm, gg in (("gobs", n.gobs), ("lobs", n.lobs)):
        if gg is not None:
            grids[nm] = dict(w=gg.info.width, h=gg.info.height, res=gg.info.resolution,
                             ox=gg.info.origin.position.x, oy=gg.info.origin.position.y,
                             data=list(gg.data))
    fh.write(json.dumps(dict(ev="grids", cloud_src=n.cloud_src, grids=grids)) + "\n")
    fh.close()
    print(f"WROTE {a.out}l rows={len(n.rows)} cloud_src={n.cloud_src} amcl_upd={n.upd}")


if __name__ == "__main__":
    sys.exit(main())
