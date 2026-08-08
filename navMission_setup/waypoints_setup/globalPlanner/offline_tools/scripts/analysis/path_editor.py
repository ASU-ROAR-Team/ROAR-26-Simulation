#!/usr/bin/env python3
"""
Offline Path Editor
--------------------
Loads a costmap + an offline path (both CSV) and lets you tweak the path
by hand before competition day.

Two editing modes (toggle with TAB):
  WAYPOINT mode - click to insert a point onto the path, drag an existing
                  point to relocate it, right-click to delete one.
  DRAW mode     - hold left mouse and draw freehand; on release, the
                  stroke is spliced into the path between the two nearest
                  existing points, replacing whatever was between them.
                  This is the fast way to "redraw" a bad section instead
                  of placing waypoints one at a time.

Controls:
  Left click / drag   - add point / move point (WAYPOINT) or draw (DRAW)
  Right click         - delete nearest point
  Mouse wheel         - zoom in/out (centered on cursor)
  Middle-mouse drag   - pan
  TAB                 - toggle WAYPOINT / DRAW mode
  Ctrl+Z              - undo
  Ctrl+Y / Ctrl+Shift+Z - redo
  R                   - reset view (zoom/pan)
  S                   - save path to edited_offline_path.csv
  ESC                 - quit
"""

import pathlib
import pygame
import csv
import math
import os
import copy

# --- Configuration ---
_TOOLS_ROOT = str(pathlib.Path(__file__).resolve().parent.parent.parent)

def _resolve_file(filename, subfolder='data'):
    cwd = os.getcwd()
    here = str(pathlib.Path(__file__).resolve().parent)
    candidates = [
        str(pathlib.Path(_TOOLS_ROOT) / subfolder / filename),
        str(pathlib.Path(cwd) / subfolder / filename),
        str(pathlib.Path(cwd) / filename),
        str(pathlib.Path(here) / filename),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]

COSTMAP_FILE = _resolve_file('costmap.csv', 'data')
PATH_FILE    = _resolve_file('combined_offline_path.csv', 'data')
OUTPUT_FILE  = str(pathlib.Path(_TOOLS_ROOT) / 'data' / 'edited_offline_path.csv')

RESOLUTION = 0.1
WINDOW_SIZE = 900
UI_HEIGHT = 66

MIN_ZOOM = 0.2
MAX_ZOOM = 25.0
ZOOM_STEP = 1.15

# Orientation of the rendered map image relative to the raw CSV grid
# (raw pixel (col=x_idx, row=y_idx) with row 0 = world y = 0).
# Toggle these two if the map ever looks mirrored/upside-down again --
# draw_map()'s blit anchor derives from these automatically, no need to
# re-work the flip math by hand.
MAP_FLIP_X = False  # mirror left-right
MAP_FLIP_Y = False  # mirror top-bottom

# how close (in world meters, at effective zoom) the mouse must be to a
# point before it counts as a "hit" for dragging / deleting
POINT_HIT_RADIUS_PX = 10

# minimum spacing (meters) between consecutively sampled points while
# freehand-drawing, so a slow stroke doesn't dump thousands of points
DRAW_SAMPLE_SPACING = 0.05

MAX_UNDO_STEPS = 100

# --- Colors ---
COLOR_FREE = (255, 255, 255)
COLOR_OBSTACLE = (40, 40, 40)
COLOR_UNKNOWN = (200, 200, 200)
COLOR_PATH = (0, 100, 255)
COLOR_POINT = (255, 0, 0)
COLOR_POINT_HOVER = (255, 170, 0)
COLOR_STROKE = (0, 200, 90)
COLOR_TEXT = (0, 0, 0)
COLOR_UI_BG = (220, 220, 220)
COLOR_BG = (20, 20, 20)

MODE_WAYPOINT = 'WAYPOINT'
MODE_DRAW = 'DRAW'


class PathEditor:
    def __init__(self):
        pygame.init()
        self.font = pygame.font.SysFont("Arial", 16)
        self.font_small = pygame.font.SysFont("Arial", 13)

        self.resolution = RESOLUTION
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.costmap = []
        self.grid_width = 0
        self.grid_height = 0
        self.path = []

        self.load_costmap()
        self.ref_waypoints = []
        self.load_ref_waypoints()

        # Calculate screen size based on costmap aspect ratio
        self.screen_width = WINDOW_SIZE
        aspect_ratio = self.grid_height / self.grid_width if self.grid_width > 0 else 1.0
        self.screen_height = int(WINDOW_SIZE * aspect_ratio) + UI_HEIGHT

        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Offline Path Editor")

        self.calculate_layout()
        self.load_path()

        # unscaled (grid_width x grid_height) surface, y-flipped once so it
        # can be scaled/blitted directly by the camera transform at any zoom
        self.raw_map_surface = self.create_raw_map_surface()
        self._scaled_map_cache = None
        self._scaled_map_cache_size = None

        # --- camera state ---
        self.viewport_cx = self.screen_width / 2.0
        self.viewport_cy = UI_HEIGHT + (self.screen_height - UI_HEIGHT) / 2.0
        self.zoom, self.cam_wx, self.cam_wy = self.default_view()

        # --- editing state ---
        self.mode = MODE_WAYPOINT
        self.dragging_idx = None
        self.drag_started_undo_pushed = False
        self.drawing_stroke = []
        self.panning = False
        self.last_pan_pos = None

        self.undo_stack = []
        self.redo_stack = []

        self.clock = pygame.time.Clock()
        self.save_message_timer = 0

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------
    def load_costmap(self):
        print(f"Loading costmap from {COSTMAP_FILE}...")
        try:
            lines = []
            with open(COSTMAP_FILE, 'r') as f:
                for line in f:
                    if line.startswith('#'):
                        if 'resolution=' in line:
                            self.resolution = float(line.split('resolution=')[1].split()[0])
                        if 'origin_x=' in line:
                            parts = line.split()
                            for p in parts:
                                if p.startswith('origin_x='):
                                    self.origin_x = float(p.split('=')[1])
                                elif p.startswith('origin_y='):
                                    self.origin_y = float(p.split('=')[1])
                    else:
                        lines.append(line)

            reader = csv.reader(lines)
            header = next(reader, None)

            # Determine exact declared column count from header
            declared_cols = len([h for h in header[1:] if h.strip() != ''])

            raw_rows = []
            for row in reader:
                clean_row = []
                for val in row[1:declared_cols + 1]:
                    val = val.strip()
                    if val:
                        try:
                            clean_row.append(int(val))
                        except ValueError:
                            clean_row.append(0)
                    else:
                        clean_row.append(0)
                raw_rows.append(clean_row)

            # Straight row-major read: CSV row index = world y, CSV column
            # index = world x. (Reverted -- the transpose/flip variants
            # tried earlier were wrong; this matches what the map is
            # actually supposed to look like.)
            self.grid_height = len(raw_rows)
            self.grid_width = len(raw_rows[0]) if raw_rows else declared_cols

            if self.origin_x == 0.0 and self.origin_y == 0.0 and self.grid_width > 0:
                self.origin_x = - (self.grid_width * self.resolution) / 2.0
                self.origin_y = - (self.grid_height * self.resolution) / 2.0

            for row in raw_rows:
                self.costmap.extend(row)

            # Padding safeguard (shouldn't be needed after a clean transpose,
            # kept for safety against ragged input)
            padding = (self.grid_width * self.grid_height) - len(self.costmap)
            if padding > 0:
                self.costmap.extend([0] * padding)

            print(f"Map aligned (after x/y transpose fix): {self.grid_width}x{self.grid_height}")
        except Exception as e:
            print(f"Error loading costmap: {e}")

    def load_ref_waypoints(self):
        ref_path = str(pathlib.Path(__file__).resolve().parent.parent.parent / 'reference' / 'waypoints.csv')
        # (fallback removed — path is always resolved relative to __file__)
        if os.path.exists(ref_path):
            try:
                with open(ref_path, 'r') as f:
                    reader = csv.reader(f)
                    for idx, row in enumerate(reader):
                        row = [c.strip() for c in row]
                        if len(row) >= 2:
                            # Label comes from column 3 if present, else auto-generate
                            lbl = row[2].strip() if len(row) >= 3 and row[2].strip() else f"WP{idx}"
                            self.ref_waypoints.append((float(row[0]), float(row[1]), lbl))
                print(f"Loaded {len(self.ref_waypoints)} reference waypoints for labeling.")
            except Exception as e:
                print(f"Error loading ref waypoints: {e}")

    def calculate_path_cost(self):
        if len(self.path) < 2:
            return 0.0
        total_cost = 0.0
        for i in range(len(self.path) - 1):
            p1, p2 = self.path[i], self.path[i+1]
            dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            col = int((p2[0] - self.origin_x) / self.resolution)
            row = int((p2[1] - self.origin_y) / self.resolution)
            if 0 <= row < self.grid_height and 0 <= col < self.grid_width:
                val = self.costmap[row * self.grid_width + col]
            else:
                val = -1
            
            if val == 100:
                cost_term = 100.0
            elif val == -1 or val == 255:
                cost_term = 1.0
            else:
                cost_term = 1.0 + val / 252.0
            total_cost += dist * cost_term
        return total_cost

    def draw_waypoint_labels(self):
        for wx, wy, label in self.ref_waypoints:
            sx, sy = self.world_to_screen(wx, wy)
            pygame.draw.circle(self.screen, (255, 0, 0), (sx, sy), 5)
            lbl_surf = self.font.render(label, True, (255, 255, 255))
            bg_rect = pygame.Rect(sx + 8, sy - 10, lbl_surf.get_width() + 4, lbl_surf.get_height() + 2)
            pygame.draw.rect(self.screen, (0, 0, 0), bg_rect)
            self.screen.blit(lbl_surf, (sx + 10, sy - 8))

    def load_path(self):
        if os.path.exists(PATH_FILE):
            with open(PATH_FILE, 'r') as f:
                reader = csv.reader(f)
                next(reader, None)
                self.path = [[float(row[0]), float(row[1])] for row in reader if len(row) >= 2]

    def save_path(self):
        with open(OUTPUT_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            writer.writerows(self.path)
        print(f"Path saved to {OUTPUT_FILE}")

        # Automatically generate matching Edited Path Attraction Costmap Layer
        try:
            from offline_tools.scripts.mapping.generate_path_attraction_costmap import generate_attraction_costmap, export_attraction_costmap
        except ImportError:
            try:
                from scripts.mapping.generate_path_attraction_costmap import generate_attraction_costmap, export_attraction_costmap
            except ImportError:
                generate_attraction_costmap = None

        if generate_attraction_costmap is not None:
            meta = {'resolution': self.resolution, 'origin_x': self.origin_x, 'origin_y': self.origin_y,
                    'width': self.grid_width, 'height': self.grid_height}
            grid = generate_attraction_costmap(self.path, meta, max_bonus=30.0, corridor_radius=1.5, sigma=0.5)

            stem = os.path.splitext(OUTPUT_FILE)[0]
            out_attr_path = f"{stem}_attraction_costmap.csv"
            try:
                export_attraction_costmap(grid, meta, out_attr_path)
                print(f"Generated edited path attraction costmap: {out_attr_path}")
            except Exception as e:
                print(f"Failed to write edited attraction costmap: {e}")

        self.save_message_timer = 120

    # ------------------------------------------------------------------
    # Layout / rendering
    # ------------------------------------------------------------------
    def calculate_layout(self):
        """Base (zoom=1) scale that fits the whole grid under the UI bar."""
        available_height = self.screen_height - UI_HEIGHT
        world_w = self.grid_width * self.resolution
        world_h = self.grid_height * self.resolution
        scale_x = self.screen_width / world_w if world_w > 0 else 1.0
        scale_y = available_height / world_h if world_h > 0 else 1.0
        self.base_scale = min(scale_x, scale_y)  # pixels per meter at zoom 1

    def create_raw_map_surface(self):
        surface = pygame.Surface((self.grid_width, self.grid_height))
        pixels = pygame.PixelArray(surface)

        for y in range(self.grid_height):
            for x in range(self.grid_width):
                val = self.costmap[y * self.grid_width + x]

                if val == -1 or val == 255:
                    color = COLOR_UNKNOWN
                elif val == 0:
                    color = COLOR_FREE
                elif val >= 90:
                    color = COLOR_OBSTACLE
                else:
                    intensity = 240 - int((val / 100.0) * 140)
                    color = (intensity, intensity, intensity)

                pixels[x, y] = surface.map_rgb(color)

        pixels.close()
        # Flip to match the desired visual orientation. Path drawing maps
        # world meters into this same display frame via world_to_display().
        return pygame.transform.flip(surface, MAP_FLIP_X, MAP_FLIP_Y)

    def world_to_display(self, x, y):
        rx = x - self.origin_x
        ry = y - self.origin_y
        dx = (self.grid_width * self.resolution - rx) if MAP_FLIP_X else rx
        dy = ry if MAP_FLIP_Y else (self.grid_height * self.resolution - ry)
        return dx, dy

    def display_to_world(self, dx, dy):
        rx = (self.grid_width * self.resolution - dx) if MAP_FLIP_X else dx
        ry = dy if MAP_FLIP_Y else (self.grid_height * self.resolution - dy)
        x = rx + self.origin_x
        y = ry + self.origin_y
        return x, y

    def default_view(self):
        """Frame the whole costmap in the viewport."""
        margin = 1.0  # meters of breathing room around the path
        w = self.grid_width * self.resolution
        h = self.grid_height * self.resolution
        pts = [
            self.world_to_display(self.origin_x, self.origin_y),
            self.world_to_display(self.origin_x + w, self.origin_y),
            self.world_to_display(self.origin_x, self.origin_y + h),
            self.world_to_display(self.origin_x + w, self.origin_y + h),
        ]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs) - margin, max(xs) + margin
        min_y, max_y = min(ys) - margin, max(ys) + margin

        span_x = max(max_x - min_x, 0.01)
        span_y = max(max_y - min_y, 0.01)
        available_height = self.screen_height - UI_HEIGHT
        fit_scale = min(self.screen_width / span_x, available_height / span_y)
        zoom = max(MIN_ZOOM, min(MAX_ZOOM, fit_scale / self.base_scale))
        cam_wx = (min_x + max_x) / 2.0
        cam_wy = (min_y + max_y) / 2.0
        return zoom, cam_wx, cam_wy

    @property
    def effective_scale(self):
        return self.base_scale * self.zoom

    def world_to_screen(self, x, y):
        dx, dy = self.world_to_display(x, y)
        s = self.effective_scale
        sx = self.viewport_cx + (dx - self.cam_wx) * s
        sy = self.viewport_cy - (dy - self.cam_wy) * s
        return int(sx), int(sy)

    def screen_to_world(self, px, py):
        s = self.effective_scale
        dx = self.cam_wx + (px - self.viewport_cx) / s
        dy = self.cam_wy - (py - self.viewport_cy) / s
        return self.display_to_world(dx, dy)

    def get_scaled_map_surface(self):
        s = self.effective_scale
        w = max(1, int(round(self.grid_width * self.resolution * s)))
        h = max(1, int(round(self.grid_height * self.resolution * s)))
        if self._scaled_map_cache is None or self._scaled_map_cache_size != (w, h):
            self._scaled_map_cache = pygame.transform.smoothscale(self.raw_map_surface, (w, h))
            self._scaled_map_cache_size = (w, h)
        return self._scaled_map_cache

    def draw_map(self):
        scaled = self.get_scaled_map_surface()
        # Top-left of the flipped surface in world meters -- world_to_screen
        # converts through the display frame so this stays aligned with the path.
        anchor_x = self.grid_width * self.resolution + self.origin_x if MAP_FLIP_X else self.origin_x
        anchor_y = self.grid_height * self.resolution + self.origin_y if MAP_FLIP_Y else self.origin_y
        top_left = self.world_to_screen(anchor_x, anchor_y)
        self.screen.blit(scaled, top_left)

    def hit_radius_world(self):
        return POINT_HIT_RADIUS_PX / self.effective_scale

    # ------------------------------------------------------------------
    # Undo / redo
    # ------------------------------------------------------------------
    def push_undo(self):
        self.undo_stack.append(copy.deepcopy(self.path))
        if len(self.undo_stack) > MAX_UNDO_STEPS:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def undo(self):
        if not self.undo_stack:
            return
        self.redo_stack.append(copy.deepcopy(self.path))
        self.path = self.undo_stack.pop()

    def redo(self):
        if not self.redo_stack:
            return
        self.undo_stack.append(copy.deepcopy(self.path))
        self.path = self.redo_stack.pop()

    # ------------------------------------------------------------------
    # Path editing helpers
    # ------------------------------------------------------------------
    def insert_point(self, wx, wy):
        """Insert a single point into the best-fit segment of the path."""
        if len(self.path) < 2:
            self.path.append([wx, wy])
            return

        min_dist, insert_idx = float('inf'), len(self.path)
        for i in range(len(self.path) - 1):
            p1, p2 = self.path[i], self.path[i + 1]
            line_len_sq = (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2
            if line_len_sq == 0:
                continue
            t = max(0, min(1, ((wx - p1[0]) * (p2[0] - p1[0]) + (wy - p1[1]) * (p2[1] - p1[1])) / line_len_sq))
            dist = math.hypot(wx - (p1[0] + t * (p2[0] - p1[0])), wy - (p1[1] + t * (p2[1] - p1[1])))
            if dist < min_dist:
                min_dist, insert_idx = dist, i + 1
        self.path.insert(insert_idx, [wx, wy])

    def get_closest_point_index(self, wx, wy, max_dist=None):
        best_idx, best_dist = -1, float('inf')
        for i, pt in enumerate(self.path):
            d = math.hypot(pt[0] - wx, pt[1] - wy)
            if d < best_dist:
                best_dist, best_idx = d, i
        if max_dist is not None and best_dist > max_dist:
            return -1
        return best_idx

    def splice_stroke_into_path(self, stroke):
        """Replace the section of self.path between the points nearest to
        the stroke's two ends with the freehand-drawn stroke, so drawing
        over a bad section 'redraws' it instead of just appending."""
        if len(stroke) < 2:
            return

        if len(self.path) < 2:
            self.path = [list(p) for p in stroke]
            return

        start_idx = self.get_closest_point_index(*stroke[0])
        end_idx = self.get_closest_point_index(*stroke[-1])

        if start_idx == -1 or end_idx == -1:
            self.path = [list(p) for p in stroke]
            return

        lo, hi = min(start_idx, end_idx), max(start_idx, end_idx)
        new_stroke = stroke if start_idx <= end_idx else list(reversed(stroke))
        self.path = self.path[:lo] + [list(p) for p in new_stroke] + self.path[hi + 1:]

    def describe_world_point(self, wx, wy):
        """For the diagnostic readout: world coords, the costmap cell they
        map to (using the tool's current row=y/col=x convention), and that
        cell's cost -- so alignment can be checked by hovering instead of
        guessing from screenshots."""
        # Truncate (not round()) to match worldToMapSafe() in the D* planner --
        # rounding here would sometimes report a different cell than the one
        # the planner actually uses for the same world coordinate.
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        if 0 <= row < self.grid_height and 0 <= col < self.grid_width:
            cost = self.costmap[row * self.grid_width + col]
        else:
            cost = None
        return wx, wy, row, col, cost

    def draw_origin_marker(self):
        """Marks world (0, 0) with a magenta crosshair so you can see exactly
        where the origin lands relative to the map, whatever the current flip
        settings are."""
        ox, oy = self.world_to_screen(0.0, 0.0)
        size = 10
        color = (255, 0, 255)
        pygame.draw.line(self.screen, color, (ox - size, oy), (ox + size, oy), 2)
        pygame.draw.line(self.screen, color, (ox, oy - size), (ox, oy + size), 2)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def draw_ui(self, hover_info=None):
        pygame.draw.rect(self.screen, COLOR_UI_BG, (0, 0, self.screen_width, UI_HEIGHT))

        # --- Row 1 (y=5): Mode (left) | zoom (right) ---
        mode_text = self.font.render(f"Mode: {self.mode}  (TAB to switch)", True, COLOR_TEXT)
        self.screen.blit(mode_text, (10, 5))

        zoom_text = self.font_small.render(f"zoom {self.zoom:.2f}x", True, COLOR_TEXT)
        self.screen.blit(zoom_text, (self.screen_width - 80, 7))

        # --- Row 2 (y=24): Path metrics (left) | Waypoint order (right) ---
        path_len = sum(math.hypot(p2[0]-p1[0], p2[1]-p1[1]) for p1, p2 in zip(self.path[:-1], self.path[1:]))
        live_cost = self.calculate_path_cost()
        cost_text = self.font_small.render(
            f"Path Length: {path_len:.2f} m  |  D* Cost: {live_cost:.2f}", True, (20, 20, 120))
        self.screen.blit(cost_text, (10, 26))

        order_str = " -> ".join([wp[2] for wp in self.ref_waypoints])
        if self.ref_waypoints:
            order_str += f" -> {self.ref_waypoints[0][2]}"
        order_text = self.font_small.render(f"Order: {order_str}", True, (20, 20, 120))
        order_w = order_text.get_width()
        self.screen.blit(order_text, (self.screen_width - order_w - 10, 26))

        # --- Row 3 (y=46): hover diagnostics when mouse is on map, else help ---
        if hover_info is not None:
            wx, wy, row, col, cost = hover_info
            cost_str = str(cost) if cost is not None else "out of bounds"
            diag_text = self.font_small.render(
                f"world=({wx:.2f}, {wy:.2f})  cell(row={row}, col={col})  cost={cost_str}",
                True, COLOR_TEXT)
            self.screen.blit(diag_text, (10, 48))
        else:
            help_text = self.font_small.render(
                "L-click: add/drag/draw | R-click: delete | wheel: zoom | mid-drag: pan | "
                "Ctrl+Z/Y: undo/redo | R: reset | S: save | ESC: quit",
                True, COLOR_TEXT)
            self.screen.blit(help_text, (10, 48))

        if self.save_message_timer > 0:
            save_text = self.font_small.render(f"Saved to {OUTPUT_FILE}!", True, (0, 130, 0))
            self.screen.blit(save_text, (self.screen_width - 200, 48))
            self.save_message_timer -= 1

    # ------------------------------------------------------------------
    # Event handling
    # ------------------------------------------------------------------
    def handle_wheel(self, y):
        mx, my = pygame.mouse.get_pos()
        # Zoom around the cursor in display-frame coords (same space as cam_*).
        s = self.effective_scale
        disp_x = self.cam_wx + (mx - self.viewport_cx) / s
        disp_y = self.cam_wy - (my - self.viewport_cy) / s

        factor = ZOOM_STEP if y > 0 else (1.0 / ZOOM_STEP)
        self.zoom = max(MIN_ZOOM, min(MAX_ZOOM, self.zoom * factor))

        s = self.effective_scale
        self.cam_wx = disp_x - (mx - self.viewport_cx) / s
        self.cam_wy = disp_y - (self.viewport_cy - my) / s

    def handle_mouse_down(self, event):
        mx, my = event.pos
        if my <= UI_HEIGHT:
            return

        if event.button == 2:  # middle click -> start pan
            self.panning = True
            self.last_pan_pos = (mx, my)
            return

        wx, wy = self.screen_to_world(mx, my)

        if event.button == 1:
            if self.mode == MODE_WAYPOINT:
                hit_idx = self.get_closest_point_index(wx, wy, max_dist=self.hit_radius_world())
                if hit_idx != -1:
                    self.push_undo()
                    self.dragging_idx = hit_idx
                else:
                    self.push_undo()
                    self.insert_point(wx, wy)
            elif self.mode == MODE_DRAW:
                self.drawing_stroke = [(wx, wy)]

        elif event.button == 3:  # right click -> delete nearest point
            idx = self.get_closest_point_index(wx, wy)
            if idx != -1:
                self.push_undo()
                self.path.pop(idx)

    def handle_mouse_up(self, event):
        if event.button == 2:
            self.panning = False
            self.last_pan_pos = None
        elif event.button == 1:
            if self.dragging_idx is not None:
                self.dragging_idx = None
            elif self.mode == MODE_DRAW and len(self.drawing_stroke) >= 2:
                self.push_undo()
                self.splice_stroke_into_path(self.drawing_stroke)
            self.drawing_stroke = []

    def handle_mouse_motion(self, event):
        mx, my = event.pos

        if self.panning and self.last_pan_pos is not None:
            dx = mx - self.last_pan_pos[0]
            dy = my - self.last_pan_pos[1]
            s = self.effective_scale
            self.cam_wx -= dx / s
            self.cam_wy += dy / s
            self.last_pan_pos = (mx, my)
            return

        if my <= UI_HEIGHT:
            return
        wx, wy = self.screen_to_world(mx, my)

        if self.dragging_idx is not None:
            self.path[self.dragging_idx] = [wx, wy]
        elif self.mode == MODE_DRAW and pygame.mouse.get_pressed()[0] and self.drawing_stroke:
            last = self.drawing_stroke[-1]
            if math.hypot(wx - last[0], wy - last[1]) >= DRAW_SAMPLE_SPACING:
                self.drawing_stroke.append((wx, wy))

    def handle_keydown(self, event):
        mods = pygame.key.get_mods()
        ctrl = mods & pygame.KMOD_CTRL

        if event.key == pygame.K_s:
            self.save_path()
        elif event.key == pygame.K_ESCAPE:
            return False
        elif event.key == pygame.K_TAB:
            self.mode = MODE_DRAW if self.mode == MODE_WAYPOINT else MODE_WAYPOINT
            self.drawing_stroke = []
            self.dragging_idx = None
        elif event.key == pygame.K_r and not ctrl:
            self.zoom, self.cam_wx, self.cam_wy = self.default_view()
        elif ctrl and event.key == pygame.K_z and (mods & pygame.KMOD_SHIFT):
            self.redo()
        elif ctrl and event.key == pygame.K_z:
            self.undo()
        elif ctrl and event.key == pygame.K_y:
            self.redo()
        return True

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    running = self.handle_keydown(event)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button in (4, 5):  # legacy wheel-as-button events
                        self.handle_wheel(1 if event.button == 4 else -1)
                    else:
                        self.handle_mouse_down(event)
                elif event.type == pygame.MOUSEBUTTONUP:
                    self.handle_mouse_up(event)
                elif event.type == pygame.MOUSEMOTION:
                    self.handle_mouse_motion(event)
                elif event.type == pygame.MOUSEWHEEL:
                    self.handle_wheel(event.y)

            self.screen.fill(COLOR_BG)
            self.draw_map()
            self.draw_origin_marker()
            self.draw_waypoint_labels()

            mx, my = pygame.mouse.get_pos()
            hover_idx = -1
            hover_info = None
            if my > UI_HEIGHT:
                wx, wy = self.screen_to_world(mx, my)
                hover_info = self.describe_world_point(wx, wy)
                if self.mode == MODE_WAYPOINT:
                    hover_idx = self.get_closest_point_index(wx, wy, max_dist=self.hit_radius_world())

            for i, pt in enumerate(self.path):
                color = COLOR_POINT_HOVER if i == hover_idx else COLOR_POINT
                pygame.draw.circle(self.screen, color, self.world_to_screen(*pt), 3)

            if len(self.path) > 1:
                # Distinct leg colors for visual segmentation
                LEG_COLORS = [
                    (255, 50, 50),   # Leg 1: Red (S1 -> W1)
                    (50, 255, 50),   # Leg 2: Green (W1 -> W3)
                    (50, 100, 255),  # Leg 3: Blue (W3 -> W2)
                    (255, 50, 255),  # Leg 4: Magenta (W2 -> S8)
                    (50, 255, 255),  # Leg 5: Cyan (S8 -> S1)
                ]
                
                leg_idx = 0
                current_pts = [self.world_to_screen(*self.path[0])]
                
                for i in range(len(self.path) - 1):
                    p2 = self.path[i+1]
                    s2 = self.world_to_screen(*p2)
                    current_pts.append(s2)
                    
                    if len(self.ref_waypoints) > leg_idx + 1:
                        next_wp = self.ref_waypoints[leg_idx + 1]
                        dist = math.hypot(p2[0] - next_wp[0], p2[1] - next_wp[1])
                        if dist < 0.25:
                            color = LEG_COLORS[leg_idx % len(LEG_COLORS)]
                            if len(current_pts) >= 2:
                                pygame.draw.lines(self.screen, color, False, current_pts, 3)
                            leg_idx += 1
                            current_pts = [s2]
                
                if len(current_pts) >= 2:
                    color = LEG_COLORS[leg_idx % len(LEG_COLORS)]
                    pygame.draw.lines(self.screen, color, False, current_pts, 3)

            if self.mode == MODE_DRAW and len(self.drawing_stroke) > 1:
                pts = [self.world_to_screen(*p) for p in self.drawing_stroke]
                pygame.draw.lines(self.screen, COLOR_STROKE, False, pts, 3)

            self.draw_ui(hover_info)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()


if __name__ == '__main__':
    editor = PathEditor()
    editor.run()