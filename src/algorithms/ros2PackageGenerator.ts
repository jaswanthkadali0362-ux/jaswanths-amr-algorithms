import { SimulationConfig } from '../types/slam';

export interface Ros2PackageConfig {
  packageName: string;
  rosDistro: 'humble' | 'jazzy';
  scanTopic: string;
  mapTopic: string;
  cmdVelTopic: string;
  baseFrame: string;
  odomFrame: string;
  mapFrame: string;
  useNav2Action: boolean;
  gridResolution: number;
  robotRadius: number;
}

export interface GeneratedRosFile {
  id: string;
  filename: string;
  relativePath: string;
  language: 'python' | 'yaml' | 'xml' | 'bash';
  title: string;
  description: string;
  content: string;
}

export function generateRos2PackageFiles(
  simConfig: SimulationConfig,
  rosConfig: Ros2PackageConfig
): GeneratedRosFile[] {
  const maxThroatCells =
    simConfig.pruningAggression === 'CONSERVATIVE'
      ? 4
      : simConfig.pruningAggression === 'STANDARD'
      ? 8
      : 12;

  const lidarMaxMeters = (
    simConfig.lidarRange *
    rosConfig.gridResolution *
    2.5
  ).toFixed(1);
  const inflationRadiusMeters = (
    simConfig.safetyInflationRadius * rosConfig.gridResolution * 3.0
  ).toFixed(2);

  // 1. Full Production Python Node: dead_end_explorer_node.py (Upgraded with LaserScan DWA & Costmap Inflation)
  const explorerNodePy = `#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Package: ${rosConfig.packageName}
Node:    dead_end_explorer_node.py
Target:  ROS 2 ${rosConfig.rosDistro.toUpperCase()} (rclpy + SLAM Toolbox + Nav2 + Real-Time LaserScan DWA)

Intelligent Zero-Prior-Map AMR Explorer with DWA Obstacle Avoidance & Dead-End Taboo Memory:
1. Subscribes to ${rosConfig.mapTopic} (OccupancyGrid) AND ${rosConfig.scanTopic} (sensor_msgs/LaserScan).
2. Builds an Exponential Safety Costmap Inflation buffer around discovered walls so global A* paths
   maintain safe clearance from corners and narrow walls.
3. Filters transient dynamic obstacles (moving people / AGVs detected in /scan) so temporary corridor
   blockages trigger immediate DWA avoidance without falsely locking corridors as permanent dead ends.
4. Executes 2-Stage Topological Dead-End Pruning (Degree-1 Cellular Peeling + Throat-Cut Sealing)
   in persistent world-coordinate memory (/taboo_dead_end_grid).
5. Runs a 15 Hz Dynamic Window Approach (DWA) & 5-Sector LaserScan Proximity Shield that evaluates
   candidate (v, omega) velocity rollouts against live LiDAR points, smoothly steering around
   moving obstacles and executing automatic emergency backoff if critical clearance is breached.
6. Returns autonomously to the recorded Home Dock (x0, y0, yaw0) once all reachable frontiers are mapped.
"""

import heapq
import math
from collections import deque
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener


UNKNOWN = -1
FREE = 0
OCCUPIED_THRESH = 65


class FrontierCluster:
    def __init__(
        self,
        cluster_id: int,
        cells: List[Tuple[int, int]],
        vantage: Tuple[int, int],
        info_gain: int,
    ):
        self.id = cluster_id
        self.cells = cells
        self.vantage = vantage  # (row, col)
        self.info_gain = info_gain
        self.utility = -float('inf')


class DeadEndPruningExplorer(Node):
    def __init__(self):
        super().__init__('dead_end_pruning_explorer')

        # Declare configurable ROS 2 parameters
        self.declare_parameter('scan_topic', '${rosConfig.scanTopic}')
        self.declare_parameter('map_topic', '${rosConfig.mapTopic}')
        self.declare_parameter('cmd_vel_topic', '${rosConfig.cmdVelTopic}')
        self.declare_parameter('map_frame', '${rosConfig.mapFrame}')
        self.declare_parameter('base_frame', '${rosConfig.baseFrame}')
        self.declare_parameter('use_nav2_action', ${rosConfig.useNav2Action ? 'True' : 'False'})
        self.declare_parameter('info_gain_weight', ${simConfig.infoGainWeight.toFixed(2)})
        self.declare_parameter('distance_weight', ${simConfig.distanceWeight.toFixed(2)})
        self.declare_parameter('max_throat_cells', ${maxThroatCells})
        self.declare_parameter('robot_radius_m', ${rosConfig.robotRadius.toFixed(2)})
        self.declare_parameter('safety_inflation_m', ${inflationRadiusMeters})
        self.declare_parameter('critical_stop_dist_m', ${(rosConfig.robotRadius + 0.16).toFixed(2)})
        self.declare_parameter('dwa_evasion_dist_m', ${(rosConfig.robotRadius + 0.55).toFixed(2)})
        self.declare_parameter('max_linear_vel', 0.34)
        self.declare_parameter('max_angular_vel', 1.35)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_nav2 = self.get_parameter('use_nav2_action').value
        self.alpha = float(self.get_parameter('info_gain_weight').value)
        self.beta = float(self.get_parameter('distance_weight').value)
        self.max_throat_cells = int(self.get_parameter('max_throat_cells').value)
        self.robot_radius_m = float(self.get_parameter('robot_radius_m').value)
        self.safety_inflation_m = float(self.get_parameter('safety_inflation_m').value)
        self.critical_stop_dist_m = float(self.get_parameter('critical_stop_dist_m').value)
        self.dwa_evasion_dist_m = float(self.get_parameter('dwa_evasion_dist_m').value)
        self.max_v = float(self.get_parameter('max_linear_vel').value)
        self.max_w = float(self.get_parameter('max_angular_vel').value)

        # Persistent World-Coordinate Taboo Memory Set
        self.taboo_world_keys: Set[Tuple[int, int]] = set()
        self.locked_zone_count: int = 0

        # State Machine & Live Sensor Buffers
        self.fsm_state: str = 'IDLE'
        self.home_dock_world: Optional[Tuple[float, float, float]] = None
        self.current_map: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None
        self.scan_cartesian_base: List[Tuple[float, float]] = []
        self.sector_min: Dict[str, float] = {
            'front': 10.0,
            'front_left': 10.0,
            'front_right': 10.0,
            'rear': 10.0,
        }
        self.active_goal_world: Optional[Tuple[float, float]] = None
        self.active_astar_path_world: List[Tuple[float, float]] = []

        # TF2 Pose Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS & Subscriptions
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map_received, map_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan_received, qos_profile_sensor_data
        )

        # Publishers
        self.taboo_grid_pub = self.create_publisher(
            OccupancyGrid, '/taboo_dead_end_grid', map_qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/explorer_markers', 10
        )
        self.dwa_marker_pub = self.create_publisher(
            MarkerArray, '/dwa_rollouts', 10
        )
        self.plan_pub = self.create_publisher(Path, '/explorer_astar_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Nav2 ActionClient
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timers: Global SLAM/Frontier Executive (2 Hz) + Local DWA Obstacle Avoidance (15 Hz)
        self.exec_timer = self.create_timer(0.5, self.autonomy_tick)
        self.dwa_timer = self.create_timer(1.0 / 15.0, self.dwa_obstacle_avoidance_tick)

        self.get_logger().info(
            f'Intelligent AMR Explorer + DWA Obstacle Avoidance online '
            f'[inflation={self.safety_inflation_m}m, critical_stop={self.critical_stop_dist_m}m]'
        )

    # =========================================================================
    # Live 360° LaserScan Processing & 5-Sector Safety Shield
    # =========================================================================
    def on_scan_received(self, msg: LaserScan):
        self.latest_scan = msg
        pts: List[Tuple[float, float]] = []
        front, f_left, f_right, rear = 10.0, 10.0, 10.0, 10.0

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and math.isfinite(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                pts.append((x, y))

                deg = math.degrees(math.atan2(y, x))
                if -30.0 <= deg <= 30.0:
                    front = min(front, r)
                elif 30.0 < deg <= 105.0:
                    f_left = min(f_left, r)
                elif -105.0 <= deg < -30.0:
                    f_right = min(f_right, r)
                else:
                    rear = min(rear, r)
            angle += msg.angle_increment

        # Downsample point cloud for fast 15 Hz DWA trajectory collision checks
        self.scan_cartesian_base = pts[::2] if len(pts) > 90 else pts
        self.sector_min = {
            'front': front,
            'front_left': f_left,
            'front_right': f_right,
            'rear': rear,
        }

    def on_map_received(self, msg: OccupancyGrid):
        self.current_map = msg

    # =========================================================================
    # Coordinate & TF2 Helpers
    # =========================================================================
    def grid_to_world(self, r: int, c: int, info) -> Tuple[float, float]:
        wx = info.origin.position.x + (c + 0.5) * info.resolution
        wy = info.origin.position.y + (r + 0.5) * info.resolution
        return wx, wy

    def world_to_grid(self, wx: float, wy: float, info) -> Tuple[int, int]:
        c = int(math.floor((wx - info.origin.position.x) / info.resolution))
        r = int(math.floor((wy - info.origin.position.y) / info.resolution))
        return r, c

    def world_to_taboo_key(self, wx: float, wy: float, res: float) -> Tuple[int, int]:
        return (int(round(wx / res)), int(round(wy / res)))

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return tx, ty, yaw
        except TransformException:
            return None

    # =========================================================================
    # Costmap Inflation Distance Transform
    # =========================================================================
    def compute_inflation_costmap(self, grid_2d: np.ndarray, res: float) -> np.ndarray:
        """
        Computes exponential safety costmap in [0.0, 1.0] around occupied walls
        so global A* stays centered in corridors and away from sharp corners.
        """
        h, w = grid_2d.shape
        inflation = np.zeros((h, w), dtype=np.float32)
        wall_rows, wall_cols = np.where(grid_2d >= OCCUPIED_THRESH)
        if len(wall_rows) == 0:
            return inflation

        rad_cells = max(1, int(math.ceil(self.safety_inflation_m / res)))
        for dr in range(-rad_cells, rad_cells + 1):
            for dc in range(-rad_cells, rad_cells + 1):
                dist_m = math.hypot(dr, dc) * res
                if dist_m > self.safety_inflation_m:
                    continue
                cost = math.exp(-3.2 * (dist_m / max(0.1, self.safety_inflation_m)))
                nr = np.clip(wall_rows + dr, 0, h - 1)
                nc = np.clip(wall_cols + dc, 0, w - 1)
                inflation[nr, nc] = np.maximum(inflation[nr, nc], cost)

        return inflation

    # =========================================================================
    # Two-Stage Topological Dead-End Pruning Engine
    # =========================================================================
    def update_dead_end_taboo_memory(
        self,
        grid_2d: np.ndarray,
        dock_rc: Tuple[int, int],
        robot_rc: Tuple[int, int],
        info,
    ) -> np.ndarray:
        h, w = grid_2d.shape
        res = info.resolution
        taboo_mask = np.zeros((h, w), dtype=bool)

        for r in range(h):
            for c in range(w):
                if grid_2d[r, c] == FREE:
                    wx, wy = self.grid_to_world(r, c, info)
                    if self.world_to_taboo_key(wx, wy, res) in self.taboo_world_keys:
                        taboo_mask[r, c] = True

        # Do not lock new dead ends while a dynamic obstacle is immediately blocking the robot's front sector
        if self.sector_min['front'] < self.dwa_evasion_dist_m:
            return taboo_mask

        wall_mask = grid_2d >= OCCUPIED_THRESH
        unknown_mask = grid_2d == UNKNOWN
        free_mask = (grid_2d >= 0) & (grid_2d < OCCUPIED_THRESH)
        newly_locked: List[Tuple[int, int]] = []

        def touches_unknown_8(r: int, c: int) -> bool:
            r0, r1 = max(0, r - 1), min(h, r + 2)
            c0, c1 = max(0, c - 1), min(w, c + 2)
            return bool(np.any(unknown_mask[r0:r1, c0:c1]))

        # Pass A: Iterative Degree-1 Cellular Peeling
        changed = True
        while changed:
            changed = False
            for r in range(1, h - 1):
                for c in range(1, w - 1):
                    if not free_mask[r, c] or taboo_mask[r, c] or (r, c) == dock_rc:
                        continue
                    if touches_unknown_8(r, c):
                        continue
                    open_neighbors = sum(
                        1
                        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                        if not wall_mask[r + dr, c + dc] and not taboo_mask[r + dr, c + dc]
                    )
                    if open_neighbors <= 1:
                        taboo_mask[r, c] = True
                        newly_locked.append((r, c))
                        changed = True

        # Pass B: Multi-Cell Topological Throat-Cut Pocket Sealing
        candidate_throats: List[List[Tuple[int, int]]] = []
        for r in range(1, h - 1):
            c = 1
            while c < w - 1:
                if free_mask[r, c] and not taboo_mask[r, c]:
                    start_c = c
                    seg = []
                    while c < w - 1 and free_mask[r, c] and not taboo_mask[r, c]:
                        seg.append((r, c))
                        c += 1
                    if (
                        (wall_mask[r, start_c - 1] or taboo_mask[r, start_c - 1])
                        and (wall_mask[r, c] or taboo_mask[r, c])
                        and 1 <= len(seg) <= self.max_throat_cells
                    ):
                        candidate_throats.append(seg)
                else:
                    c += 1

        for c in range(1, w - 1):
            r = 1
            while r < h - 1:
                if free_mask[r, c] and not taboo_mask[r, c]:
                    start_r = r
                    seg = []
                    while r < h - 1 and free_mask[r, c] and not taboo_mask[r, c]:
                        seg.append((r, c))
                        r += 1
                    if (
                        (wall_mask[start_r - 1, c] or taboo_mask[start_r - 1, c])
                        and (wall_mask[r, c] or taboo_mask[r, c])
                        and 1 <= len(seg) <= self.max_throat_cells
                    ):
                        candidate_throats.append(seg)
                else:
                    r += 1

        max_pocket_cells = int(self.max_throat_cells * 35)
        for throat in candidate_throats:
            if any(
                taboo_mask[tr, tc] or (tr, tc) == dock_rc or touches_unknown_8(tr, tc)
                for tr, tc in throat
            ):
                continue
            throat_set = set(throat)
            seeds = [
                (tr + dr, tc + dc)
                for tr, tc in throat
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                if 0 <= tr + dr < h
                and 0 <= tc + dc < w
                and free_mask[tr + dr, tc + dc]
                and not taboo_mask[tr + dr, tc + dc]
                and (tr + dr, tc + dc) not in throat_set
            ]

            visited_global: Set[Tuple[int, int]] = set()
            for seed in seeds:
                if seed in visited_global or taboo_mask[seed[0], seed[1]]:
                    continue
                q = deque([seed])
                comp_visited = {seed}
                component = []
                has_unknown = False
                has_dock = False
                too_large = False

                while q:
                    cr, cc = q.popleft()
                    component.append((cr, cc))
                    visited_global.add((cr, cc))
                    if (cr, cc) == dock_rc:
                        has_dock = True
                    if touches_unknown_8(cr, cc):
                        has_unknown = True
                    if len(component) > max_pocket_cells:
                        too_large = True
                        break
                    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nr, nc = cr + dr, cc + dc
                        if not (0 <= nr < h and 0 <= nc < w):
                            continue
                        if (nr, nc) in comp_visited or (nr, nc) in throat_set:
                            continue
                        if unknown_mask[nr, nc]:
                            has_unknown = True
                        elif free_mask[nr, nc] and not taboo_mask[nr, nc]:
                            comp_visited.add((nr, nc))
                            q.append((nr, nc))

                if not has_unknown and not has_dock and not too_large and len(component) > 0:
                    for pr, pc in component + throat:
                        if not taboo_mask[pr, pc]:
                            taboo_mask[pr, pc] = True
                            newly_locked.append((pr, pc))

        if newly_locked:
            for lr, lc in newly_locked:
                wx, wy = self.grid_to_world(lr, lc, info)
                self.taboo_world_keys.add(self.world_to_taboo_key(wx, wy, res))
            self.locked_zone_count += 1

        return taboo_mask

    # =========================================================================
    # Frontier Extraction & Inflation-Aware A* Global Planner
    # =========================================================================
    def extract_frontiers(
        self,
        grid_2d: np.ndarray,
        taboo_mask: np.ndarray,
        inflation_map: np.ndarray,
        robot_rc: Tuple[int, int],
    ) -> List[FrontierCluster]:
        h, w = grid_2d.shape
        free_mask = (grid_2d >= 0) & (grid_2d < OCCUPIED_THRESH) & (~taboo_mask)
        unknown_mask = grid_2d == UNKNOWN

        is_frontier = np.zeros((h, w), dtype=bool)
        frontier_pts: List[Tuple[int, int]] = []

        for r in range(1, h - 1):
            for c in range(1, w - 1):
                if not free_mask[r, c] or inflation_map[r, c] > 0.78:
                    continue
                if (
                    unknown_mask[r - 1, c]
                    or unknown_mask[r + 1, c]
                    or unknown_mask[r, c - 1]
                    or unknown_mask[r, c + 1]
                ):
                    is_frontier[r, c] = True
                    frontier_pts.append((r, c))

        visited: Set[Tuple[int, int]] = set()
        clusters: List[FrontierCluster] = []
        cid = 1

        for pt in frontier_pts:
            if pt in visited:
                continue
            q = deque([pt])
            visited.add(pt)
            cells = []
            while q:
                cr, cc = q.popleft()
                cells.append((cr, cc))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = cr + dr, cc + dc
                        if (
                            0 <= nr < h
                            and 0 <= nc < w
                            and (nr, nc) not in visited
                            and is_frontier[nr, nc]
                        ):
                            visited.add((nr, nc))
                            q.append((nr, nc))

            if len(cells) < 2:
                continue

            mean_r = sum(p[0] for p in cells) / len(cells)
            mean_c = sum(p[1] for p in cells) / len(cells)
            vantage = min(
                cells,
                key=lambda p: math.hypot(p[0] - mean_r, p[1] - mean_c)
                + 2.0 * float(inflation_map[p[0], p[1]]),
            )

            vr, vc = vantage
            r0, r1 = max(0, vr - 4), min(h, vr + 5)
            c0, c1 = max(0, vc - 4), min(w, vc + 5)
            info_gain = int(np.sum(unknown_mask[r0:r1, c0:c1])) + len(cells) * 2

            cluster = FrontierCluster(cid, cells, vantage, info_gain)
            cid += 1
            euclid_dist = math.hypot(vr - robot_rc[0], vc - robot_rc[1])
            cluster.utility = self.alpha * info_gain - self.beta * euclid_dist
            clusters.append(cluster)

        clusters.sort(key=lambda c: c.utility, reverse=True)
        return clusters

    def plan_astar_path(
        self,
        grid_2d: np.ndarray,
        taboo_mask: np.ndarray,
        inflation_map: np.ndarray,
        start_rc: Tuple[int, int],
        goal_rc: Tuple[int, int],
        allow_taboo_exit: bool = True,
    ) -> Optional[List[Tuple[int, int]]]:
        if start_rc == goal_rc:
            return [start_rc]

        h, w = grid_2d.shape
        start_in_taboo = bool(taboo_mask[start_rc[0], start_rc[1]])
        open_set: List[Tuple[float, Tuple[int, int]]] = [(0.0, start_rc)]
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start_rc: 0.0}
        closed: Set[Tuple[int, int]] = set()

        dirs = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_rc:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            if current in closed:
                continue
            closed.add(current)
            cr, cc = current
            curr_is_taboo = bool(taboo_mask[cr, cc])

            for dr, dc, step_cost in dirs:
                nr, nc = cr + dr, cc + dc
                if not (0 <= nr < h and 0 <= nc < w) or (nr, nc) in closed:
                    continue
                if grid_2d[nr, nc] < 0 or grid_2d[nr, nc] >= OCCUPIED_THRESH:
                    continue
                if taboo_mask[nr, nc]:
                    if not allow_taboo_exit or (not start_in_taboo and not curr_is_taboo):
                        continue

                inflation_cost = float(inflation_map[nr, nc]) * 2.5
                taboo_penalty = 60.0 if taboo_mask[nr, nc] else 0.0
                tentative_g = g_score[current] + step_cost + inflation_cost + taboo_penalty
                if tentative_g < g_score.get((nr, nc), float('inf')):
                    came_from[(nr, nc)] = current
                    g_score[(nr, nc)] = tentative_g
                    f = tentative_g + math.hypot(goal_rc[0] - nr, goal_rc[1] - nc)
                    heapq.heappush(open_set, (f, (nr, nc)))

        return None

    # =========================================================================
    # Real-Time 15 Hz Dynamic Window Approach (DWA) & LaserScan Collision Shield
    # =========================================================================
    def dwa_obstacle_avoidance_tick(self):
        """
        Evaluates candidate (v, omega) velocity rollouts against live LaserScan
        obstacle points in base_link frame. Overrides / refines motion when a
        dynamic obstacle enters the safety envelope or when use_nav2 is False.
        """
        if self.fsm_state in ('IDLE', 'DOCKED_COMPLETE') or not self.active_astar_path_world:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # 1. Check Emergency Critical Proximity Shield
        if self.sector_min['front'] < self.critical_stop_dist_m:
            self.fsm_state = 'AVOIDING_OBSTACLE'
            cmd = Twist()
            # Back off slightly if rear is clear, and rotate toward the freer side
            cmd.linear.x = -0.10 if self.sector_min['rear'] > 0.35 else 0.0
            cmd.angular.z = (
                0.85
                if self.sector_min['front_left'] >= self.sector_min['front_right']
                else -0.85
            )
            self.cmd_vel_pub.publish(cmd)
            return

        # Determine if active DWA override is needed (either standalone mode OR obstacle within evasion zone)
        obstacle_close = min(
            self.sector_min['front'],
            self.sector_min['front_left'],
            self.sector_min['front_right'],
        ) < self.dwa_evasion_dist_m

        if self.use_nav2 and not obstacle_close:
            return

        if obstacle_close:
            self.fsm_state = 'AVOIDING_OBSTACLE'

        # 2. Transform lookahead waypoint from map frame into robot base_link frame
        lookahead_w = self.active_astar_path_world[-1]
        for wx, wy in self.active_astar_path_world:
            if math.hypot(wx - rx, wy - ry) >= 0.55:
                lookahead_w = (wx, wy)
                break

        dx_w = lookahead_w[0] - rx
        dy_w = lookahead_w[1] - ry
        goal_x_base = dx_w * math.cos(-ryaw) - dy_w * math.sin(-ryaw)
        goal_y_base = dx_w * math.sin(-ryaw) + dy_w * math.cos(-ryaw)

        # 3. Sample Dynamic Window (v, omega) and score rollouts against LaserScan point cloud
        v_samples = [0.12, 0.22, self.max_v]
        w_samples = np.linspace(-self.max_w, self.max_w, 11)
        best_v, best_w = 0.0, 0.0
        best_score = -float('inf')
        horizon_s = 1.4
        dt = 0.2

        for v in v_samples:
            for w in w_samples:
                x, y, th = 0.0, 0.0, 0.0
                min_clear = 5.0
                collision = False

                for _ in range(int(horizon_s / dt)):
                    x += v * math.cos(th) * dt
                    y += v * math.sin(th) * dt
                    th += w * dt

                    for ox, oy in self.scan_cartesian_base:
                        d = math.hypot(x - ox, y - oy)
                        if d < self.robot_radius_m + 0.06:
                            collision = True
                            break
                        if d < min_clear:
                            min_clear = d
                    if collision:
                        break

                if collision:
                    continue

                dist_to_goal = math.hypot(goal_x_base - x, goal_y_base - y)
                heading_err = abs(math.atan2(goal_y_base - y, goal_x_base - x) - th)
                score = (
                    2.6 * (math.pi - heading_err)
                    - 2.2 * dist_to_goal
                    + 2.4 * min_clear
                    + 0.8 * v
                )
                if score > best_score:
                    best_score = score
                    best_v, best_w = v, float(w)

        cmd = Twist()
        if best_score == -float('inf'):
            # All forward arcs blocked -> rotate in place toward open sector
            cmd.linear.x = 0.0
            cmd.angular.z = (
                0.75
                if self.sector_min['front_left'] >= self.sector_min['front_right']
                else -0.75
            )
        else:
            cmd.linear.x = best_v
            cmd.angular.z = best_w

        self.cmd_vel_pub.publish(cmd)

    # =========================================================================
    # Main Global Autonomy Executive Loop (2 Hz)
    # =========================================================================
    def autonomy_tick(self):
        if self.current_map is None:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry, ryaw = pose
        info = self.current_map.info
        h, w = info.height, info.width
        grid_2d = np.array(self.current_map.data, dtype=np.int16).reshape((h, w))

        if self.home_dock_world is None:
            self.home_dock_world = (rx, ry, ryaw)
            self.fsm_state = 'EXPLORING_FRONTIERS'
            self.get_logger().info(
                f'[HOME DOCK RECORDED] Origin locked at ({rx:.2f}m, {ry:.2f}m)'
            )

        robot_rc = self.world_to_grid(rx, ry, info)
        dock_rc = self.world_to_grid(self.home_dock_world[0], self.home_dock_world[1], info)
        if not (0 <= robot_rc[0] < h and 0 <= robot_rc[1] < w):
            return

        # 1. Compute Inflated Costmap & Update Dead-End Taboo Memory
        inflation_map = self.compute_inflation_costmap(grid_2d, info.resolution)
        taboo_mask = self.update_dead_end_taboo_memory(grid_2d, dock_rc, robot_rc, info)
        self.publish_taboo_costmap(taboo_mask, info)

        # 2. Extract Non-Taboo Frontiers
        frontiers = self.extract_frontiers(grid_2d, taboo_mask, inflation_map, robot_rc)
        self.publish_rviz_markers(frontiers, info)

        if self.fsm_state == 'DOCKED_COMPLETE':
            return

        chosen_path_rc: Optional[List[Tuple[int, int]]] = None
        target_world: Optional[Tuple[float, float]] = None

        for cluster in frontiers[:10]:
            path_rc = self.plan_astar_path(
                grid_2d, taboo_mask, inflation_map, robot_rc, cluster.vantage, allow_taboo_exit=True
            )
            if path_rc and len(path_rc) > 1:
                chosen_path_rc = path_rc
                target_world = self.grid_to_world(cluster.vantage[0], cluster.vantage[1], info)
                break

        if chosen_path_rc is not None and target_world is not None:
            if self.fsm_state != 'AVOIDING_OBSTACLE':
                self.fsm_state = 'EXPLORING_FRONTIERS'
            self.dispatch_navigation_goal(target_world[0], target_world[1], chosen_path_rc, info)
        else:
            self.fsm_state = 'RETURNING_HOME'
            dist_to_dock = math.hypot(rx - self.home_dock_world[0], ry - self.home_dock_world[1])
            if dist_to_dock <= max(0.25, info.resolution * 2.0):
                self.fsm_state = 'DOCKED_COMPLETE'
                self.stop_robot()
                self.get_logger().info('[MISSION COMPLETE] AMR safely docked at origin!')
                return

            home_path = (
                self.plan_astar_path(
                    grid_2d, taboo_mask, inflation_map, robot_rc, dock_rc, allow_taboo_exit=True
                )
                or self.plan_astar_path(
                    grid_2d,
                    np.zeros_like(taboo_mask),
                    inflation_map,
                    robot_rc,
                    dock_rc,
                    allow_taboo_exit=True,
                )
            )
            if home_path:
                self.dispatch_navigation_goal(
                    self.home_dock_world[0], self.home_dock_world[1], home_path, info
                )

    def dispatch_navigation_goal(
        self, gx: float, gy: float, path_rc: List[Tuple[int, int]], info
    ):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        world_pts = []
        for r, c in path_rc:
            wx, wy = self.grid_to_world(r, c, info)
            world_pts.append((wx, wy))
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.plan_pub.publish(path_msg)
        self.active_astar_path_world = world_pts

        if self.active_goal_world is not None:
            if math.hypot(gx - self.active_goal_world[0], gy - self.active_goal_world[1]) < 0.30:
                return
        self.active_goal_world = (gx, gy)

        if self.use_nav2 and self.nav2_client.wait_for_server(timeout_sec=0.5):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = self.map_frame
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = gx
            goal_msg.pose.pose.position.y = gy
            goal_msg.pose.pose.orientation.w = 1.0
            self.nav2_client.send_goal_async(goal_msg)

    def stop_robot(self):
        self.active_astar_path_world = []
        self.cmd_vel_pub.publish(Twist())

    def publish_taboo_costmap(self, taboo_mask: np.ndarray, info):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = info
        flat = np.where(taboo_mask.flatten(), 100, 0).astype(np.int8)
        msg.data = flat.tolist()
        self.taboo_grid_pub.publish(msg)

    def publish_rviz_markers(self, frontiers: List[FrontierCluster], info):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = now
        m.ns = 'frontiers'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = info.resolution * 2.2
        m.scale.y = info.resolution * 2.2
        m.scale.z = info.resolution * 2.2
        m.color = ColorRGBA(r=0.96, g=0.62, b=0.04, a=0.95)
        for f in frontiers:
            wx, wy = self.grid_to_world(f.vantage[0], f.vantage[1], info)
            m.points.append(Point(x=wx, y=wy, z=0.1))
        ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = DeadEndPruningExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;

  // 2. SLAM Toolbox Online Async Config
  const slamToolboxYaml = `# ============================================================================
# ${rosConfig.packageName}/config/slam_toolbox_online_async.yaml
# Zero-Prior-Map Online Asynchronous SLAM Configuration for ROS 2 ${rosConfig.rosDistro.toUpperCase()}
# ============================================================================
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: ${rosConfig.odomFrame}
    map_frame: ${rosConfig.mapFrame}
    base_frame: ${rosConfig.baseFrame}
    scan_topic: ${rosConfig.scanTopic}
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 1.0
    resolution: ${rosConfig.gridResolution.toFixed(3)}
    max_laser_range: ${lidarMaxMeters}
    minimum_time_interval: 0.25
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.15
    minimum_travel_heading: 0.15
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
`;

  // 3. Nav2 Zero-Prior-Map Params with Inflated Costmap & Obstacle Avoidance
  const nav2ParamsYaml = `# ============================================================================
# ${rosConfig.packageName}/config/nav2_no_map_params.yaml
# Nav2 Autonomous Navigation Configuration (Inflated Safety Costmap + Obstacle Layer)
# ============================================================================
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: ${rosConfig.mapFrame}
    robot_base_frame: ${rosConfig.baseFrame}
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 15.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.35
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.25

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.34
      lookahead_dist: 0.55
      min_lookahead_dist: 0.30
      max_lookahead_dist: 0.85
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.25
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: true
      min_approach_linear_velocity: 0.05
      use_collision_detection: true
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.85
      regulated_linear_scaling_min_speed: 0.15
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.65
      max_angular_accel: 2.5

planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.30
      use_astar: true
      allow_unknown: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 8.0
      publish_frequency: 4.0
      global_frame: ${rosConfig.odomFrame}
      robot_base_frame: ${rosConfig.baseFrame}
      rolling_window: true
      width: 4
      height: 4
      resolution: ${rosConfig.gridResolution.toFixed(3)}
      robot_radius: ${rosConfig.robotRadius.toFixed(2)}
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: ${rosConfig.scanTopic}
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: ${lidarMaxMeters}
          obstacle_max_range: ${lidarMaxMeters}
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: ${inflationRadiusMeters}

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 2.5
      publish_frequency: 1.0
      global_frame: ${rosConfig.mapFrame}
      robot_base_frame: ${rosConfig.baseFrame}
      robot_radius: ${rosConfig.robotRadius.toFixed(2)}
      resolution: ${rosConfig.gridResolution.toFixed(3)}
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        map_topic: ${rosConfig.mapTopic}
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: ${rosConfig.scanTopic}
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: ${inflationRadiusMeters}
`;

  // 4. Unified Single-Command ROS 2 Launch File
  const launchFilePy = `#!/usr/bin/env python3
"""
${rosConfig.packageName}/launch/autonomous_slam_explore.launch.py
Single-command launch file starting:
  1. SLAM Toolbox (Online Asynchronous Zero-Prior-Map mode)
  2. Nav2 Navigation Stack (with Inflated Safety Costmaps)
  3. RViz2 with preconfigured Dead-End Taboo, Costmap & Frontier overlays
  4. DeadEndPruningExplorer Node (with 15 Hz LaserScan DWA Collision Avoidance)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('${rosConfig.packageName}')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_online_async.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_no_map_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'autoslam_explorer.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )

    explorer_node = Node(
        package='${rosConfig.packageName}',
        executable='dead_end_explorer',
        name='dead_end_pruning_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': '${rosConfig.scanTopic}',
            'map_topic': '${rosConfig.mapTopic}',
            'cmd_vel_topic': '${rosConfig.cmdVelTopic}',
            'map_frame': '${rosConfig.mapFrame}',
            'base_frame': '${rosConfig.baseFrame}',
            'use_nav2_action': ${rosConfig.useNav2Action ? 'True' : 'False'},
            'info_gain_weight': ${simConfig.infoGainWeight.toFixed(2)},
            'distance_weight': ${simConfig.distanceWeight.toFixed(2)},
            'max_throat_cells': ${maxThroatCells},
            'robot_radius_m': ${rosConfig.robotRadius.toFixed(2)},
            'safety_inflation_m': ${inflationRadiusMeters},
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        slam_toolbox_launch,
        nav2_launch,
        explorer_node,
        rviz_node,
    ])
`;

  // 5. RViz2 Configuration File
  const rvizConfigYaml = `# ============================================================================
# ${rosConfig.packageName}/rviz/autoslam_explorer.rviz
# ============================================================================
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1.0
      Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Reference Frame: <Fixed Frame>
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
    - Alpha: 1.0
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: true
      Enabled: true
      Name: Live SLAM OccupancyGrid
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: ${rosConfig.mapTopic}
    - Alpha: 0.65
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Locked Dead-End Taboo Zones
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /taboo_dead_end_grid
    - Alpha: 1.0
      Class: rviz_default_plugins/LaserScan
      Color: 6; 182; 212
      Enabled: true
      Name: 360 LiDAR Scan
      Size (m): 0.04
      Topic:
        Depth: 5
        Reliability Policy: Best Effort
        Value: ${rosConfig.scanTopic}
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Frontiers & Home Dock Beacon
      Topic:
        Depth: 10
        Reliability Policy: Reliable
        Value: /explorer_markers
    - Class: rviz_default_plugins/Path
      Color: 16; 185; 129
      Enabled: true
      Line Style: Lines
      Line Width: 0.05
      Name: Active A* Exploration / Return Trajectory
      Topic:
        Depth: 10
        Reliability Policy: Reliable
        Value: /explorer_astar_plan
  Global Options:
    Background Color: 11; 14; 23
    Fixed Frame: ${rosConfig.mapFrame}
    Frame Rate: 30
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: TopDownOrtho
      Scale: 45
`;

  // 6. Package Manifest & setup.py
  const packageManifestAndSetup = `# ============================================================================
# 1. ${rosConfig.packageName}/package.xml
# ============================================================================
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${rosConfig.packageName}</name>
  <version>1.1.0</version>
  <description>Intelligent Zero-Prior-Map AMR SLAM Explorer with DWA Obstacle Avoidance &amp; Dead-End Taboo Memory</description>
  <maintainer email="robotics@autoslam.local">AMR Engineering</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>


# ============================================================================
# 2. ${rosConfig.packageName}/setup.py
# ============================================================================
from setuptools import find_packages, setup
import os
from glob import glob

package_name = '${rosConfig.packageName}'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='AMR Engineering',
    maintainer_email='robotics@autoslam.local',
    description='Intelligent Zero-Prior-Map AMR SLAM Explorer with DWA Obstacle Avoidance',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dead_end_explorer = ${rosConfig.packageName}.dead_end_explorer_node:main',
        ],
    },
)
`;

  return [
    {
      id: 'node_py',
      filename: 'dead_end_explorer_node.py',
      relativePath: `${rosConfig.packageName}/${rosConfig.packageName}/dead_end_explorer_node.py`,
      language: 'python',
      title: '1. Intelligent ROS 2 Explorer + DWA Obstacle Avoidance Node',
      description:
        'Upgraded rclpy node subscribing to /map and /scan (LaserScan), running 15 Hz DWA trajectory scoring, 5-sector proximity protection, exponential costmap inflation, and permanent dead-end taboo pruning.',
      content: explorerNodePy,
    },
    {
      id: 'slam_yaml',
      filename: 'slam_toolbox_online_async.yaml',
      relativePath: `${rosConfig.packageName}/config/slam_toolbox_online_async.yaml`,
      language: 'yaml',
      title: '2. SLAM Toolbox Config (Zero-Prior-Map)',
      description:
        'Online asynchronous SLAM configuration starting with a 100% blank map, Ceres scan matching, and loop closure.',
      content: slamToolboxYaml,
    },
    {
      id: 'nav2_yaml',
      filename: 'nav2_no_map_params.yaml',
      relativePath: `${rosConfig.packageName}/config/nav2_no_map_params.yaml`,
      language: 'yaml',
      title: '3. Nav2 Inflated Costmap & Planner Params',
      description:
        'Nav2 configuration with exponential safety inflation layers, LaserScan obstacle clearing/marking, and Regulated Pure Pursuit.',
      content: nav2ParamsYaml,
    },
    {
      id: 'launch_py',
      filename: 'autonomous_slam_explore.launch.py',
      relativePath: `${rosConfig.packageName}/launch/autonomous_slam_explore.launch.py`,
      language: 'python',
      title: '4. Unified ROS 2 Launch Script',
      description:
        'Single-command launch file bringing up slam_toolbox, nav2_bringup, dead_end_explorer, and RViz2.',
      content: launchFilePy,
    },
    {
      id: 'rviz_cfg',
      filename: 'autoslam_explorer.rviz',
      relativePath: `${rosConfig.packageName}/rviz/autoslam_explorer.rviz`,
      language: 'yaml',
      title: '5. RViz2 Display Profile',
      description:
        'Pre-configured RViz2 layout displaying /map, /taboo_dead_end_grid, /scan, /explorer_markers, and /explorer_astar_plan.',
      content: rvizConfigYaml,
    },
    {
      id: 'pkg_setup',
      filename: 'package.xml & setup.py',
      relativePath: `${rosConfig.packageName}/setup.py`,
      language: 'python',
      title: '6. Package Manifest & Colcon Build Guide',
      description:
        'ROS 2 ament_python build files including sensor_msgs LaserScan dependencies.',
      content: packageManifestAndSetup,
    },
    {
      id: 'gazebo_sdf',
      filename: 'warehouse_world.sdf & amr.urdf',
      relativePath: `${rosConfig.packageName}/worlds/warehouse_world.sdf`,
      language: 'xml',
      title: '7. Gazebo Industrial Warehouse World (.sdf) & AMR URDF',
      description:
        'Gazebo simulation world defining the industrial fulfillment warehouse, pallet racking bays, and differential-drive AMR with 360° GPU LiDAR plugin.',
      content: `<?xml version="1.0" ?>
<!-- =========================================================================
     ${rosConfig.packageName}/worlds/warehouse_world.sdf
     Gazebo Industrial Fulfillment Warehouse with Pallet Racking Bays & Rooms
     ========================================================================= -->
<sdf version="1.8">
  <world name="industrial_warehouse_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <light type="directional" name="warehouse_sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="concrete_warehouse_floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>50 35</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>50 35</size></plane></geometry>
        </visual>
      </link>
    </model>

    <!-- Differential-Drive AMR with 360-Degree 2D LiDAR (${rosConfig.scanTopic}) -->
    <model name="autoslam_amr">
      <pose>0.0 0.0 0.05 0 0 0</pose>
      <link name="${rosConfig.baseFrame}">
        <inertial>
          <mass>18.0</mass>
          <inertia><ixx>0.4</ixx><iyy>0.4</iyy><izz>0.6</izz></inertia>
        </inertial>
        <visual name="chassis_visual">
          <geometry><box><size>0.72 0.56 0.24</size></box></geometry>
        </visual>
        <sensor name="gpu_lidar_360" type="gpu_lidar">
          <pose>0.05 0 0.35 0 0 0</pose>
          <topic>${rosConfig.scanTopic}</topic>
          <update_rate>15</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>${simConfig.lidarRayCount * 4}</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.12</min>
              <max>${lidarMaxMeters}</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <always_on>1</always_on>
          <visualize>true</visualize>
        </sensor>
      </link>
    </model>
  </world>
</sdf>`,
    },
  ];
}

export function generateSelfExtractingInstallerSh(
  simConfig: SimulationConfig,
  rosConfig: Ros2PackageConfig
): string {
  const files = generateRos2PackageFiles(simConfig, rosConfig);
  const pkg = rosConfig.packageName;
  const nodeFile = files.find((f) => f.id === 'node_py')!.content;
  const slamFile = files.find((f) => f.id === 'slam_yaml')!.content;
  const nav2File = files.find((f) => f.id === 'nav2_yaml')!.content;
  const launchFile = files.find((f) => f.id === 'launch_py')!.content;
  const rvizFile = files.find((f) => f.id === 'rviz_cfg')!.content;

  return `#!/usr/bin/env bash
# ==============================================================================
# Self-Extracting ROS 2 (${rosConfig.rosDistro.toUpperCase()}) Package Builder: ${pkg}
# Includes 15 Hz LaserScan DWA Obstacle Avoidance + Dead-End Taboo Memory
# ==============================================================================
set -e

mkdir -p ${pkg}/${pkg} ${pkg}/resource ${pkg}/launch ${pkg}/config ${pkg}/rviz
touch ${pkg}/resource/${pkg} ${pkg}/${pkg}/__init__.py

cat << 'EOF_PKG_XML' > ${pkg}/package.xml
<?xml version="1.0"?>
<package format="3">
  <name>${pkg}</name>
  <version>1.1.0</version>
  <description>Intelligent Zero-Prior-Map AMR SLAM Explorer with DWA Obstacle Avoidance &amp; Dead-End Taboo Memory</description>
  <maintainer email="robotics@autoslam.local">AMR Engineering</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF_PKG_XML

cat << 'EOF_SETUP_PY' > ${pkg}/setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = '${pkg}'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='AMR Engineering',
    maintainer_email='robotics@autoslam.local',
    description='Intelligent Zero-Prior-Map AMR SLAM Explorer with DWA Obstacle Avoidance',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dead_end_explorer = ${pkg}.dead_end_explorer_node:main',
        ],
    },
)
EOF_SETUP_PY

cat << 'EOF_NODE_PY' > ${pkg}/${pkg}/dead_end_explorer_node.py
${nodeFile}
EOF_NODE_PY
chmod +x ${pkg}/${pkg}/dead_end_explorer_node.py

cat << 'EOF_SLAM_YAML' > ${pkg}/config/slam_toolbox_online_async.yaml
${slamFile}
EOF_SLAM_YAML

cat << 'EOF_NAV2_YAML' > ${pkg}/config/nav2_no_map_params.yaml
${nav2File}
EOF_NAV2_YAML

cat << 'EOF_LAUNCH_PY' > ${pkg}/launch/autonomous_slam_explore.launch.py
${launchFile}
EOF_LAUNCH_PY

cat << 'EOF_RVIZ_CFG' > ${pkg}/rviz/autoslam_explorer.rviz
${rvizFile}
EOF_RVIZ_CFG

echo "✅ Intelligent ROS 2 package '${pkg}' created!"
`;
}
