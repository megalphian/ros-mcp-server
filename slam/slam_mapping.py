import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List
from collections import deque

import numpy as np

from utils.websocket_manager import WebSocketManager


@dataclass
class MapConfig:
    resolution_m_per_cell: float = 0.05
    width_cells: int = 400
    height_cells: int = 400
    # Log-odds parameters - improved for better accumulation
    logodds_occ: float = 0.7    # Reduce to avoid over-confidence
    logodds_free: float = 0.3   # Reduce to avoid over-confidence  
    logodds_min: float = -5.0   # Extended range for better accumulation
    logodds_max: float = 5.0    # Extended range for better accumulation


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw from quaternion (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Integer Bresenham line algorithm returning all cells from (x0,y0) to (x1,y1) inclusive."""
    points: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points


class OccupancyGridSLAM:
    """Lightweight occupancy grid mapping using LaserScan and Odometry via rosbridge.

    Maintains a log-odds grid updated by raycasting beams from a refined pose.
    Implements lightweight correlative scan matching around the odometry prior
    to improve localization before integrating each scan.
    """

    def __init__(self, rosbridge_url: str, scan_topic: str = "/scan", odom_topic: str = "/odom", config: Optional[MapConfig] = None):
        self.url = rosbridge_url
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.config = config or MapConfig()

        self._ws = WebSocketManager(self.url)
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # Map storage (log-odds) and metadata
        self._grid = np.zeros((self.config.height_cells, self.config.width_cells), dtype=np.float32)
        # Center of grid as origin (0,0) in world meters
        self._origin_world_m = (
            -self.config.width_cells * self.config.resolution_m_per_cell / 2.0,
            -self.config.height_cells * self.config.resolution_m_per_cell / 2.0,
        )

        # Latest odometry and scan cache
        self._latest_pose_xy_yaw: Optional[Tuple[float, float, float]] = None
        self._last_scan_time: float = 0.0
        # Current refined pose estimate (x, y, yaw). Starts at odom when available
        self._pose_estimate_xy_yaw: Optional[Tuple[float, float, float]] = None
        # Cached probability grid for scoring
        self._prob_grid: Optional[np.ndarray] = None
        # Odom history for time alignment (timestamp_sec, x, y, yaw)
        self._odom_history: deque[Tuple[float, float, float, float]] = deque(maxlen=2000)
        # Scan history so every scan contributes to the map
        # Each record contains: t, angle_min, angle_inc, range_min, range_max, ranges, pose
        self._scan_history: deque[dict] = deque(maxlen=5000)

    def reset_map(self) -> None:
        with self._lock:
            self._grid.fill(0.0)
            self._prob_grid = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, name="slam_loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._ws.close()

    def _subscribe(self) -> None:
        # Subscribe to odom and scan streams (ROS 2 type strings)
        self._ws.send({"op": "subscribe", "topic": self.odom_topic, "type": "nav_msgs/msg/Odometry"})
        self._ws.send({"op": "subscribe", "topic": self.scan_topic, "type": "sensor_msgs/msg/LaserScan"})

    def _run_loop(self) -> None:
        try:
            self._subscribe()
            while not self._stop_event.is_set():
                raw = self._ws.receive_binary()
                if not raw:
                    time.sleep(0.01)
                    continue
                if isinstance(raw, bytes):
                    try:
                        raw = raw.decode("utf-8")
                    except Exception:
                        continue
                try:
                    data = json.loads(raw)
                except Exception:
                    continue

                topic = data.get("topic")
                msg = data.get("msg")
                if not topic or msg is None:
                    continue

                if topic == self.odom_topic:
                    self._handle_odom(msg)
                elif topic == self.scan_topic:
                    self._handle_scan(msg)
        except Exception as e:
            print(f"[SLAM] Loop error: {e}")

    def _handle_odom(self, msg: dict) -> None:
        try:
            pos = msg["pose"]["pose"]["position"]
            ori = msg["pose"]["pose"]["orientation"]
            yaw = quaternion_to_yaw(ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0), ori.get("w", 1.0))
            new_pose = (float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), float(yaw))
            
            # Debug: Print odometry updates only for significant changes
            if self._latest_pose_xy_yaw is not None:
                old_x, old_y, old_yaw = self._latest_pose_xy_yaw
                new_x, new_y, new_yaw = new_pose
                dx = new_x - old_x
                dy = new_y - old_y
                dyaw = new_yaw - old_yaw
                # Only print if significant movement (reduce logging frequency)
                if abs(dx) > 0.1 or abs(dy) > 0.1 or abs(dyaw) > 0.1:
                    print(f"[SLAM] Odom update: ({new_x:.3f}, {new_y:.3f}, {math.degrees(new_yaw):.1f}°) [Δ: {dx:.3f}, {dy:.3f}, {math.degrees(dyaw):.1f}°]")
            
            self._latest_pose_xy_yaw = new_pose
            
            # Header stamp if available
            try:
                stamp = msg.get("header", {}).get("stamp", {})
                t_sec = float(stamp.get("sec", 0.0)) + float(stamp.get("nanosec", 0.0)) / 1e9
            except Exception:
                t_sec = time.time()
            self._odom_history.append((t_sec, self._latest_pose_xy_yaw[0], self._latest_pose_xy_yaw[1], self._latest_pose_xy_yaw[2]))
            
            # Initialize refined pose if not set yet, or update it when map is sparse
            if self._pose_estimate_xy_yaw is None or len(self._scan_history) < 5:
                self._pose_estimate_xy_yaw = self._latest_pose_xy_yaw
                
        except Exception as e:
            print(f"[SLAM] Odom parse error: {e}")

    def _handle_scan(self, msg: dict) -> None:
        if self._latest_pose_xy_yaw is None:
            return
        try:
            angle_min = float(msg.get("angle_min", 0.0))
            angle_inc = float(msg.get("angle_increment", 0.0))
            ranges = msg.get("ranges", [])
        except Exception:
            return

        # Gather scan metadata
        range_min = float(msg.get("range_min", 0.0))
        range_max = float(msg.get("range_max", float("inf")))
        # Select a time-aligned odom prior using header.stamp if available
        try:
            stamp = msg.get("header", {}).get("stamp", {})
            t_scan = float(stamp.get("sec", 0.0)) + float(stamp.get("nanosec", 0.0)) / 1e9
        except Exception:
            t_scan = time.time()
        prior_odom = self._get_prior_pose_for_time(t_scan) or self._pose_estimate_xy_yaw or self._latest_pose_xy_yaw
        if prior_odom is None:
            return
            
        # For the first few scans or when map is sparse, trust odometry more
        if len(self._scan_history) < 10 or np.count_nonzero(self._grid) < 100:
            # Use odometry directly for pose estimate with minimal correction
            robot_x_m, robot_y_m, yaw = prior_odom
            # Only print occasionally for the first few scans
            if len(self._scan_history) % 5 == 0:
                print(f"[SLAM] Using odometry directly: ({robot_x_m:.3f}, {robot_y_m:.3f}, {math.degrees(yaw):.1f}°)")
        else:
            # Refine pose with scan matching using all beams (no subsampling)
            robot_x_m, robot_y_m, yaw = self._refine_pose_with_scan(prior_odom, angle_min, angle_inc, ranges)
            # Only print scan matching info every 10 scans
            if len(self._scan_history) % 10 == 0:
                print(f"[SLAM] Using scan matching: ({robot_x_m:.3f}, {robot_y_m:.3f}, {math.degrees(yaw):.1f}°)")
            
        # Store refined pose for next iteration
        self._pose_estimate_xy_yaw = (robot_x_m, robot_y_m, yaw)

        # Append to history - we'll rebuild the full map when requested
        rec = {
            "t": t_scan,
            "angle_min": angle_min,
            "angle_inc": angle_inc,
            "range_min": range_min,
            "range_max": range_max,
            "ranges": ranges,
            "pose": (robot_x_m, robot_y_m, yaw),
        }
        with self._lock:
            self._scan_history.append(rec)
            # Integrate this scan immediately for real-time updates
            self._integrate_scan_core(rec)
            self._prob_grid = None
            
        # Only print scan integration info every 10 scans
        if len(self._scan_history) % 10 == 0:
            print(f"[SLAM] Scan {len(self._scan_history)} integrated. Pose: ({robot_x_m:.3f}, {robot_y_m:.3f}, {math.degrees(yaw):.1f}°)")

    # --- Scan matching utilities ---
    def _ensure_prob_grid(self) -> np.ndarray:
        if self._prob_grid is None:
            l = np.clip(self._grid, self.config.logodds_min, self.config.logodds_max)
            self._prob_grid = 1.0 - 1.0 / (1.0 + np.exp(l))
        return self._prob_grid

    def _score_pose(self, pose_xy_yaw: Tuple[float, float, float], angle_min: float, angle_inc: float, ranges: List[float],
                    beam_step: int = 1, free_penalty_weight: float = 0.2) -> float:
        """Score a pose by correlating scan endpoints with the current map probabilities.

        - Positive score for endpoints that land on occupied (high prob)
        - Small penalty for occupied cells along the beam path (encourage free space)
        """
        px, py, pyaw = pose_xy_yaw
        probs = self._ensure_prob_grid()
        score = 0.0
        rx, ry = self._world_to_grid(px, py)
        for i in range(0, len(ranges), beam_step):
            r = ranges[i]
            if r is None:
                continue
            try:
                r_f = float(r)
            except Exception:
                continue
            if not math.isfinite(r_f) or r_f <= 0.01:
                continue
            a = pyaw + angle_min + i * angle_inc
            end_x_m = px + r_f * math.cos(a)
            end_y_m = py + r_f * math.sin(a)
            ex, ey = self._world_to_grid(end_x_m, end_y_m)
            if not self._in_bounds(ex, ey):
                continue
            # Endpoint reward
            score += float(probs[ey, ex])
            # Free-space penalty along beam
            cells = bresenham(rx, ry, ex, ey)
            for cx, cy in cells[:-1][::3]:  # sparse samples
                if self._in_bounds(cx, cy):
                    score -= free_penalty_weight * float(probs[cy, cx])
        return score

    def _refine_pose_with_scan(self, prior_pose: Tuple[float, float, float], angle_min: float, angle_inc: float, ranges: List[float]) -> Tuple[float, float, float]:
        """Local correlative scan matching around the prior pose (simplified and more robust)."""
        px, py, pyaw = prior_pose
        
        # If map is still very sparse, just return odometry
        if np.count_nonzero(self._grid) < 50:
            return prior_pose
            
        # Smaller search window for more reliable matching
        dx_search = [d * self.config.resolution_m_per_cell for d in (-1, 0, 1)]
        dy_search = [d * self.config.resolution_m_per_cell for d in (-1, 0, 1)]
        dyaw_search = [math.radians(d) for d in (-2, -1, 0, 1, 2)]

        best_pose = prior_pose
        best_score = self._score_pose(best_pose, angle_min, angle_inc, ranges, beam_step=2)
        
        # Simple grid search with reduced resolution
        search_count = 0
        for dx in dx_search:
            for dy in dy_search:
                for dyaw in dyaw_search:
                    search_count += 1
                    pose = (px + dx, py + dy, pyaw + dyaw)
                    try:
                        s = self._score_pose(pose, angle_min, angle_inc, ranges, beam_step=2)
                        if s > best_score:
                            best_score = s
                            best_pose = pose
                    except Exception:
                        continue  # Skip invalid poses
        
        # If no improvement found, stick with odometry (reduce logging frequency)
        if best_pose == prior_pose and len(self._scan_history) % 20 == 0:
            print(f"[SLAM] Scan matching found no improvement, using odometry")
            
        return best_pose

    def _get_prior_pose_for_time(self, t_scan: float) -> Optional[Tuple[float, float, float]]:
        """Find the nearest odom pose in time to the given scan timestamp."""
        if not self._odom_history:
            return None
        # Linear search is fine for short history; could be optimized with bisect
        best = None
        best_dt = float("inf")
        for t, x, y, yaw in self._odom_history:
            dt = abs(t - t_scan)
            if dt < best_dt:
                best_dt = dt
                best = (x, y, yaw)
        return best

    def _rebuild_map_from_history(self) -> None:
        """Rebuild the entire occupancy grid from all stored scan history.
        
        This ensures that every single scan contributes to the final map image,
        which is crucial for accurate SLAM visualization.
        """
        with self._lock:
            # Reset the grid to start fresh
            self._grid.fill(0.0)
            self._prob_grid = None
            
            # Reintegrate all scans from history
            if len(self._scan_history) > 20:  # Only print for significant rebuilds
                print(f"[SLAM] Rebuilding map from {len(self._scan_history)} scans")
            for i, rec in enumerate(self._scan_history):
                self._integrate_scan_core(rec)
                # Progress indicator for large maps (less frequent)
                if (i + 1) % 200 == 0:
                    print(f"[SLAM] Processed {i + 1}/{len(self._scan_history)} scans")
            
            # Invalidate probability cache after rebuild
            self._prob_grid = None
            if len(self._scan_history) > 20:  # Only print for significant rebuilds
                print(f"[SLAM] Map rebuild complete with {len(self._scan_history)} scans integrated")

    def _integrate_scan_core(self, rec: dict) -> None:
        """Integrate one scan record into the occupancy grid using its stored pose."""
        ranges: List[float] = rec["ranges"]
        angle_min: float = rec["angle_min"]
        angle_inc: float = rec["angle_inc"]
        range_min: float = rec["range_min"]
        range_max: float = rec["range_max"]
        px, py, pyaw = rec["pose"]

        rx, ry = self._world_to_grid(px, py)
        
        # Skip if robot position is out of bounds
        if not self._in_bounds(rx, ry):
            return
            
        valid_rays = 0
        for i, r in enumerate(ranges):
            if r is None:
                continue
            try:
                r_f = float(r)
            except Exception:
                continue
            if not math.isfinite(r_f):
                continue
            # Be more permissive with range limits to capture more data
            if r_f < 0.1 or r_f > 12.0:  # Reasonable LiDAR limits
                continue
                
            a = pyaw + angle_min + i * angle_inc
            end_x_m = px + r_f * math.cos(a)
            end_y_m = py + r_f * math.sin(a)
            ex, ey = self._world_to_grid(end_x_m, end_y_m)
            
            cells = bresenham(rx, ry, ex, ey)
            if not cells:
                continue
                
            valid_rays += 1
            
            # Mark all cells along the ray as free (except the last one)
            for cx, cy in cells[:-1]:
                if self._in_bounds(cx, cy):
                    self._grid[cy, cx] = max(self.config.logodds_min, 
                                           self._grid[cy, cx] - self.config.logodds_free)
            
            # Mark the endpoint as occupied
            cx, cy = cells[-1]
            if self._in_bounds(cx, cy):
                self._grid[cy, cx] = min(self.config.logodds_max, 
                                       self._grid[cy, cx] + self.config.logodds_occ)
        
        # Debug info for scan integration (only print warnings for problematic scans)
        if valid_rays == 0:
            print(f"[SLAM] Warning: No valid rays in scan at pose ({px:.2f}, {py:.2f})")
        elif valid_rays < 5:  # Only warn for very low ray counts
            print(f"[SLAM] Warning: Only {valid_rays} valid rays in scan")

    def _in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.config.width_cells and 0 <= y < self.config.height_cells

    def _world_to_grid(self, x_m: float, y_m: float) -> Tuple[int, int]:
        # origin world meters maps to (0,0) grid; increase x to right, y up
        gx = int((x_m - self._origin_world_m[0]) / self.config.resolution_m_per_cell)
        gy = int((y_m - self._origin_world_m[1]) / self.config.resolution_m_per_cell)
        return gx, gy

    def _grid_to_prob(self) -> np.ndarray:
        # Convert log-odds to occupancy probability [0..1]
        l = np.clip(self._grid, self.config.logodds_min, self.config.logodds_max)
        return 1.0 - 1.0 / (1.0 + np.exp(l))

    def get_map_metadata(self) -> dict:
        return {
            "resolution": self.config.resolution_m_per_cell,
            "width": self.config.width_cells,
            "height": self.config.height_cells,
            "origin": {"x": self._origin_world_m[0], "y": self._origin_world_m[1]},
        }
    
    def get_slam_stats(self) -> dict:
        """Get SLAM statistics for debugging and monitoring."""
        with self._lock:
            return {
                "total_scans": len(self._scan_history),
                "odom_history_length": len(self._odom_history),
                "current_pose": self._pose_estimate_xy_yaw,
                "latest_odom": self._latest_pose_xy_yaw,
                "grid_occupancy_stats": {
                    "min_logodds": float(np.min(self._grid)),
                    "max_logodds": float(np.max(self._grid)),
                    "mean_logodds": float(np.mean(self._grid)),
                    "non_zero_cells": int(np.count_nonzero(self._grid)),
                }
            }

    def get_map_data(self) -> dict:
        # Rebuild the entire map from all stored scans to ensure completeness
        self._rebuild_map_from_history()
        
        probs = self._grid_to_prob()
        # Convert to ROS-like occupancy data 0..100, -1 unknown
        # Unknown here defined as near 0 log-odds magnitude
        unknown_mask = np.isclose(self._grid, 0.0, atol=1e-3)
        occ = (probs * 100.0).astype(np.int16)
        occ[unknown_mask] = -1
        return {
            "metadata": self.get_map_metadata(),
            "data": occ.tolist(),
        }

    def get_map_image_base64(self) -> Optional[str]:
        try:
            import cv2
            import base64

            # Rebuild the entire map from all stored scans to ensure completeness
            self._rebuild_map_from_history()
            
            probs = self._grid_to_prob()
            # Visualize: unknown=127 gray, free=255 white, occupied=0 black
            img = (255 - (probs * 255.0)).astype(np.uint8)
            unknown_mask = np.isclose(self._grid, 0.0, atol=1e-3)
            img[unknown_mask] = 127

            _, buf = cv2.imencode(".png", img)
            return base64.b64encode(buf).decode("utf-8")
        except Exception as e:
            print(f"[SLAM] Image export failed: {e}")
            return None


