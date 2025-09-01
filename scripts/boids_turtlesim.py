#!/usr/bin/env python3

"""
Boids with turtlesim (ROS1, Python 3 / rospy)
- Spawns 5 turtles at random positions
- Implements the three classic Boids rules:
  1) Separation
  2) Alignment
  3) Cohesion
- Simple wall avoidance to keep turtles inside the window

How to run (example):
1) In one terminal:
   $ roscore
2) In a second terminal:
   $ rosrun turtlesim turtlesim_node
3) In a third terminal (after putting this file into your catkin pkg's scripts/ and chmod +x):
   $ rosrun <your_package_name> boids_turtlesim.py

Tune parameters in the PARAMS dict below.
You can also dynamically tweak using rosparam (e.g., rosparam set /boids/max_speed 2.0) between runs.
"""

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import Spawn, TeleportAbsolute


@dataclass
class Vec2:
    x: float
    y: float

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, k: float) -> "Vec2":
        return Vec2(self.x * k, self.y * k)

    def __truediv__(self, k: float) -> "Vec2":
        if k == 0:
            return Vec2(0.0, 0.0)
        return Vec2(self.x / k, self.y / k)

    def mag(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> "Vec2":
        m = self.mag()
        if m == 0:
            return Vec2(0.0, 0.0)
        return self / m


# Global-ish parameters; can be overwritten from ROS params at startup
PARAMS = {
    # Field/Window
    "world_min": 0.5,            # keep a margin from exact border 0
    "world_max": 10.5,           # turtlesim is ~[0, 11]
    "wall_margin": 1.0,          # distance from wall to start pushing back
    "wall_push": 1.2,            # magnitude of wall force

    # Boids
    "neighbor_dist": 2.5,
    "w_separation": 1.6,
    "w_alignment": 1.0,
    "w_cohesion": 0.8,

    # Motion
    "max_speed": 1.6,            # m/s
    "max_turn": 3.0,             # rad/s
    "k_turn": 2.5,               # angular gain from heading error

    # Misc
    "rate_hz": 15.0,
    "seed": None,                # set to an int to make runs reproducible
}


class BoidsTurtles:
    def __init__(self):
        rospy.init_node("boids", anonymous=False)
        self.params = self._load_params()
        if self.params["seed"] is not None:
            random.seed(int(self.params["seed"]))

        # Keep track of turtles and their poses
        self.turtles: List[str] = []
        self.pose: Dict[str, TurtlePose] = {}
        self.pub: Dict[str, rospy.Publisher] = {}

        # Prepare services
        rospy.wait_for_service("/spawn")
        self.spawn_srv = rospy.ServiceProxy("/spawn", Spawn)
        self.teleport_srv = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)

        # Ensure turtle1 exists (turtlesim spawns it). Move it to random pos.
        self.turtles.append("turtle1")
        self._setup_turtle("turtle1")

        # Spawn 4 more to make total 5
        for i in range(4):
            name = self._spawn_random()
            self.turtles.append(name)
            self._setup_turtle(name)

        self.rate = rospy.Rate(self.params["rate_hz"])

    # --------------------- Setup helpers ---------------------
    def _load_params(self) -> Dict:
        p = PARAMS.copy()
        for k, v in p.items():
            p[k] = rospy.get_param(f"/boids/{k}", v)
        return p

    def _random_pose(self) -> Tuple[float, float, float]:
        lo = self.params["world_min"]
        hi = self.params["world_max"]
        x = random.uniform(lo, hi)
        y = random.uniform(lo, hi)
        theta = random.uniform(-math.pi, math.pi)
        return x, y, theta

    def _setup_turtle(self, name: str):
        # Subscribe to pose and create publisher
        rospy.Subscriber(f"/{name}/pose", TurtlePose, self._pose_cb, callback_args=name, queue_size=1)
        self.pub[name] = rospy.Publisher(f"/{name}/cmd_vel", Twist, queue_size=1)

    def _pose_cb(self, msg: TurtlePose, name: str):
        self.pose[name] = msg

    def _spawn_random(self) -> str:
        x, y, theta = self._random_pose()
        try:
            resp = self.spawn_srv(x, y, theta, "")  # empty name => auto name like turtle2,3,...
            return resp.name
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn failed: {e}")
            rospy.signal_shutdown("spawn_failed")
            raise

    # --------------------- Boids core ---------------------
    def run(self):
        # Randomize turtle1 start as well (teleport)
        x, y, th = self._random_pose()
        try:
            self.teleport_srv(x, y, th)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Teleport failed (turtle1 might be mid-move): {e}")

        # Main loop
        while not rospy.is_shutdown():
            self._step()
            self.rate.sleep()

    def _step(self):
        # Need all poses available before computing
        if any(t not in self.pose for t in self.turtles):
            return

        # Compute a command for each turtle independently
        for me in self.turtles:
            cmd = self._compute_cmd(me)
            self.pub[me].publish(cmd)

    def _compute_cmd(self, me: str) -> Twist:
        my = self.pose[me]
        my_pos = Vec2(my.x, my.y)
        my_dir = Vec2(math.cos(my.theta), math.sin(my.theta))

        neighbor_dist = float(self.params["neighbor_dist"])
        w_sep = float(self.params["w_separation"])
        w_align = float(self.params["w_alignment"])
        w_coh = float(self.params["w_cohesion"])

        # --- Gather neighbors ---
        neighbors: List[str] = []
        for other in self.turtles:
            if other == me:
                continue
            p = self.pose[other]
            d = math.hypot(p.x - my.x, p.y - my.y)
            if d < neighbor_dist:
                neighbors.append(other)

        # --- Boids forces ---
        separation = Vec2(0.0, 0.0)
        alignment = Vec2(0.0, 0.0)
        cohesion = Vec2(0.0, 0.0)

        if neighbors:
            # Separation: steer away inversely to distance
            for n in neighbors:
                p = self.pose[n]
                away = Vec2(my.x - p.x, my.y - p.y)
                d = max(1e-6, math.hypot(away.x, away.y))
                separation = separation + (away / (d * d))  # stronger when very close

            # Alignment: match average heading vector
            avg_heading = Vec2(0.0, 0.0)
            for n in neighbors:
                th = self.pose[n].theta
                avg_heading = avg_heading + Vec2(math.cos(th), math.sin(th))
            avg_heading = (avg_heading / len(neighbors)).normalized()
            alignment = avg_heading - my_dir

            # Cohesion: move toward centroid of neighbors
            centroid = Vec2(0.0, 0.0)
            for n in neighbors:
                p = self.pose[n]
                centroid = centroid + Vec2(p.x, p.y)
            centroid = centroid / len(neighbors)
            cohesion = centroid - my_pos

        # Wall avoidance (simple push inward near borders)
        wall_force = self._wall_force(my_pos)

        # Combine
        desired = (
            separation * w_sep +
            alignment * w_align +
            cohesion * w_coh +
            wall_force
        )

        return self._vector_to_twist(desired, my.theta)

    def _wall_force(self, pos: Vec2) -> Vec2:
        lo = float(self.params["world_min"]) + float(self.params["wall_margin"])
        hi = float(self.params["world_max"]) - float(self.params["wall_margin"])
        push = float(self.params["wall_push"])

        fx = 0.0
        fy = 0.0
        # If close to left/right walls, push inward
        if pos.x < lo:
            fx += push * (lo - pos.x)
        elif pos.x > hi:
            fx -= push * (pos.x - hi)
        # If close to bottom/top walls, push inward
        if pos.y < lo:
            fy += push * (lo - pos.y)
        elif pos.y > hi:
            fy -= push * (pos.y - hi)
        return Vec2(fx, fy)

    def _vector_to_twist(self, v: Vec2, current_theta: float) -> Twist:
        tw = Twist()
        max_speed = float(self.params["max_speed"])
        max_turn = float(self.params["max_turn"])
        k_turn = float(self.params["k_turn"])

        mag = v.mag()
        if mag < 1e-3:
            # No strong desire => slow down
            tw.linear.x = 0.0
            tw.angular.z = 0.0
            return tw

        desired_theta = math.atan2(v.y, v.x)
        err = self._ang_diff(desired_theta, current_theta)

        # Turn toward target
        tw.angular.z = max(-max_turn, min(max_turn, k_turn * err))

        # Move faster when mostly facing target, slower when turning
        fwd = max(0.0, math.cos(err))  # 1 when aligned, 0 when opposite
        tw.linear.x = min(max_speed, mag) * fwd
        return tw

    @staticmethod
    def _ang_diff(a: float, b: float) -> float:
        # Smallest signed angle from b->a
        x = (a - b + math.pi) % (2 * math.pi) - math.pi
        return x


if __name__ == "__main__":
    try:
        BoidsTurtles().run()
    except rospy.ROSInterruptException:
        pass
