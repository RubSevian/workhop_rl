#!/usr/bin/env python3
"""Manual Stage-4C evaluator and fixed-world route service.

Navigation uses only Point-LIO /state_estimation.  /sim/ground_truth_odom is
subscribed for diagnostics and never feeds the path transform.
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def finite(msg):
    vals = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    return all(math.isfinite(float(v)) for v in vals)


class Stage4CEvaluator(Node):
    def __init__(self, args):
        super().__init__("stage4c_pointlio_evaluator")
        self.args = args
        self.lio = None
        self.gt = None
        self.align = None
        self.goal = None
        self.active = False
        self.finished = False
        self.started = time.monotonic()
        self.route_started = None
        self.arrival = None
        self.samples = []
        self.collision = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.active_pub = self.create_publisher(Bool, "/navigation_active", qos)
        self.goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self.path_pub = self.create_publisher(NavPath, "/path", 10)
        self.lio_sub = self.create_subscription(Odometry, "/aft_mapped_to_init", self.lio_cb, 20)
        self.gt_sub = self.create_subscription(Odometry, "/sim/ground_truth_odom", self.gt_cb, 20)
        self.cmd_sub = self.create_subscription(TwistStamped, "/cmd_vel", self.cmd_cb, 20)
        self.collision_sub = self.create_subscription(String, "/mujoco/collision_diagnostics", self.collision_cb, qos)
        self.ready_sub = self.create_subscription(Bool, "/pointlio_ready", self.ready_cb, qos)
        self.start_srv = self.create_service(Trigger, "/stage4c/start_navigation", self.start_navigation)
        self.timer = self.create_timer(0.1, self.tick)
        self.latest_cmd = None
        self.pointlio_ready = False
        self.auto_start = bool(args.auto_start)
        self.start_delay = float(args.start_delay)

    def ready_cb(self, msg):
        self.pointlio_ready = bool(msg.data)

    def lio_cb(self, msg):
        if finite(msg):
            self.lio = msg
            self.record()

    def gt_cb(self, msg):
        if finite(msg):
            self.gt = msg
            self.record()

    def cmd_cb(self, msg):
        vals = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if all(math.isfinite(float(v)) for v in vals):
            self.latest_cmd = vals

    def collision_cb(self, msg):
        try:
            self.collision = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            pass

    def record(self):
        if self.lio is None or self.gt is None:
            return
        lx, ly, lyaw = self.lio.pose.pose.position.x, self.lio.pose.pose.position.y, yaw(self.lio.pose.pose.orientation)
        gx, gy, gyaw = self.gt.pose.pose.position.x, self.gt.pose.pose.position.y, yaw(self.gt.pose.pose.orientation)
        if self.align is None:
            da = wrap(gyaw - lyaw)
            self.align = (gx - math.cos(da) * lx + math.sin(da) * ly,
                          gy - math.sin(da) * lx - math.cos(da) * ly, da)
        ax, ay, ayaw = self.align_pose(lx, ly, lyaw)
        sample = {"t": time.monotonic() - self.started, "lio_x": lx, "lio_y": ly,
                  "gt_x": gx, "gt_y": gy, "aligned_lio_x": ax, "aligned_lio_y": ay,
                  "xy_error_m": math.hypot(ax - gx, ay - gy),
                  "yaw_error_rad": wrap(ayaw - gyaw), "cmd": self.latest_cmd}
        if self.goal:
            sample["goal_error_est_m"] = math.hypot(ax - self.goal[0], ay - self.goal[1])
            sample["goal_error_gt_m"] = math.hypot(gx - self.goal[0], gy - self.goal[1])
        self.samples.append(sample)
        if self.active and self.arrival is None and self.goal and sample["goal_error_gt_m"] <= self.args.stop_distance:
            self.arrival = time.monotonic()
            self.active = False
            self.active_pub.publish(Bool(data=False))

    def align_pose(self, x, y, a):
        tx, ty, da = self.align
        return tx + math.cos(da) * x - math.sin(da) * y, ty + math.sin(da) * x + math.cos(da) * y, wrap(a + da)

    def start_route(self):
        if time.monotonic() - self.started < self.start_delay:
            return False
        if self.gt is None or self.lio is None or not self.pointlio_ready:
            return False
        gx, gy, a = self.gt.pose.pose.position.x, self.gt.pose.pose.position.y, yaw(self.gt.pose.pose.orientation)
        self.goal = (gx + math.cos(a) * self.args.goal_x - math.sin(a) * self.args.goal_y,
                     gy + math.sin(a) * self.args.goal_x + math.cos(a) * self.args.goal_y)
        self.route_started = time.monotonic()
        self.active = True
        self.active_pub.publish(Bool(data=True))
        self.get_logger().info(f"Stage-4C route started: goal=({self.goal[0]:.3f}, {self.goal[1]:.3f})")
        return True

    def start_navigation(self, _, response):
        if time.monotonic() - self.started < self.start_delay:
            response.success = False
            response.message = f"Stand-up hold active; retry after {self.start_delay:.1f} s"
            return response
        if not self.start_route():
            response.success = False
            response.message = "Point-LIO/GT odometry is not ready"
            return response
        response.success = True
        response.message = f"Started fixed-world goal ({self.goal[0]:.3f}, {self.goal[1]:.3f})"
        return response

    def publish_route(self):
        if not self.active or self.goal is None or self.lio is None or self.align is None:
            return
        lx, ly, la = self.lio.pose.pose.position.x, self.lio.pose.pose.position.y, yaw(self.lio.pose.pose.orientation)
        tx, ty, da = self.align
        # Convert fixed GT-world goal back into the raw Point-LIO frame.
        dx, dy = self.goal[0] - tx, self.goal[1] - ty
        goal_lx = math.cos(da) * dx + math.sin(da) * dy
        goal_ly = -math.sin(da) * dx + math.cos(da) * dy
        dx, dy = goal_lx - lx, goal_ly - ly
        vx = math.cos(la) * dx + math.sin(la) * dy
        vy = -math.sin(la) * dx + math.cos(la) * dy
        stamp = self.get_clock().now().to_msg()
        route = NavPath(); route.header.stamp = stamp; route.header.frame_id = "vehicle"
        for i in range(21):
            u = i / 20.0; pose = PoseStamped(); pose.header = route.header
            pose.pose.position.x = vx * u; pose.pose.position.y = vy * u; pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        self.path_pub.publish(route)
        goal = PointStamped(); goal.header.stamp = stamp; goal.header.frame_id = "map"
        goal.point.x, goal.point.y = self.goal; self.goal_pub.publish(goal)

    def tick(self):
        if self.args.mode == "service" and self.auto_start and not self.active and self.route_started is None:
            self.start_route()
        self.publish_route()
        if self.args.mode == "static" and time.monotonic() - self.started >= self.args.duration:
            self.finish()
        elif self.active and self.arrival is None and time.monotonic() - self.route_started > self.args.duration:
            self.finish()
        elif self.arrival and time.monotonic() - self.arrival > 3.0:
            self.finish()

    def finish(self):
        if self.finished: return
        self.finished = True
        if rclpy.ok():
            self.active_pub.publish(Bool(data=False))
        xy = [s["xy_error_m"] for s in self.samples]
        ye = [abs(s["yaw_error_rad"]) for s in self.samples]
        gt = [s.get("goal_error_gt_m") for s in self.samples if "goal_error_gt_m" in s]
        est = [s.get("goal_error_est_m") for s in self.samples if "goal_error_est_m" in s]
        def stats(v):
            if not v: return {"mae": None, "rmse": None, "p95": None, "max": None, "final": None}
            q = sorted(v); return {"mae": sum(abs(x) for x in v)/len(v), "rmse": math.sqrt(sum(x*x for x in v)/len(v)), "p95": q[max(0, int(.95*len(q))-1)], "max": max(v), "final": v[-1]}
        report = {"mode": self.args.mode, "pointlio_ready": self.pointlio_ready, "aligned": self.align is not None,
                  "goal_local_m": [self.args.goal_x, self.args.goal_y], "arrived": self.arrival is not None,
                  "goal_error_gt": stats(gt), "goal_error_est": stats(est), "localization_xy": stats(xy),
                  "localization_yaw_rad": stats(ye), "samples": len(self.samples),
                  "fixed_goal_world_m": list(self.goal) if self.goal else None,
                  "collision": self.collision}
        self.args.report.parent.mkdir(parents=True, exist_ok=True)
        self.args.report.write_text(json.dumps(report, indent=2) + "\n")
        self.get_logger().info("Stage-4C report: " + json.dumps(report, sort_keys=True))
        self.timer.cancel(); rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--mode", choices=["service", "static"], default="service")
    parser.add_argument("--goal-x", type=float, default=1.0); parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=20.0); parser.add_argument("--stop-distance", type=float, default=0.2)
    parser.add_argument("--start-delay", type=float, default=12.0)
    parser.add_argument("--auto-start", type=lambda value: value.lower() in ("1", "true", "yes"), default=False)
    parser.add_argument("--report", type=Path, default=Path("/tmp/stage4c_pointlio_report.json")); args, _ = parser.parse_known_args()
    rclpy.init(); node = Stage4CEvaluator(args)
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.finish()
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()
