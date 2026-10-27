#!/usr/bin/env python3
import sys
import os
import re
import glob
import subprocess
import threading
import math
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QGridLayout,
                             QMessageBox, QFrame, QComboBox, QGraphicsDropShadowEffect,
                             QCheckBox, QStackedWidget, QListWidget, QSpinBox)
from PyQt5.QtCore import Qt, QTimer, QProcess
from PyQt5.QtGui import QFont, QColor, QPixmap
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# ============================================================
#  ASU ROAR — RACING TEAM LIVERY
# ============================================================
RACE_RED      = "#E10600"   # primary accent — F1-style racing red
RACE_RED_DIM  = "#8C0400"
CARBON_BLACK  = "#0B0B0D"   # main background
PANEL_BLACK   = "#151518"   # panel / sidebar background
PANEL_EDGE    = "#232327"   # subtle borders
STEEL         = "#8A8D93"   # secondary text / labels
WHITE         = "#F2F2F2"

MONO_FONT   = "Consolas"      # telemetry-style readouts
STACK_FONT  = "Arial"         # general UI


class ASURoarGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ASU ROAR — MISSION CONTROL")
        self.resize(1280, 880)
        self.setMinimumSize(1100, 750)
        self.setStyleSheet(f"background-color: {CARBON_BLACK};")

                # Dynamically resolve monorepo root
        from pathlib import Path
        _script_dir = Path(__file__).resolve().parent
        self.ws_root = str(Path(os.environ.get("ROAR_WS", _script_dir.parents[2])).resolve())
        
        self.workspace_root = self.ws_root
        self.sim_ws = os.path.join(self.ws_root, "src", "roar2026")
        self.nav_ws = os.path.join(self.ws_root, "src", "navstack")
        self.mission_ws = os.path.join(self.ws_root, "src", "navigation")
        
        self.yaml_path = os.path.join(self.ws_root, "src", "navstack", "navstack_nav2", "config", "roar_nav2_sim.yaml")
        self.referee_inputs_dir = os.path.join(self.ws_root, "install", "referee_engine", "share", "referee_engine", "inputs")
        if not os.path.exists(self.referee_inputs_dir):
            self.referee_inputs_dir = os.path.join(self.mission_ws, "referee_engine", "inputs")
            
        self.referee_report_dir = os.path.join(self.ws_root, "reports", "referee")
        self.slam_report_dir    = os.path.join(self.ws_root, "reports", "slam")
        os.makedirs(self.referee_report_dir, exist_ok=True)
        os.makedirs(self.slam_report_dir,    exist_ok=True)

        self.current_gui_mission = None
        self.current_world = 1
        
        # Batch Queue State
        self.batch_queue = []
        self.batch_process = None
        self.total_batch_count = 0
        self.completed_batch_count = 0
        self.batch_start_time = None
        self.run_timer = QTimer(self)
        self.run_timer.timeout.connect(self._update_run_time)

        # A batch item that isn't finished within this window is force-killed and skipped.
        self.BATCH_ITEM_TIMEOUT_MS = 20 * 60 * 1000
        self.batch_timeout_timer = QTimer(self)
        self.batch_timeout_timer.setSingleShot(True)
        self.batch_timeout_timer.timeout.connect(self._on_batch_item_timeout)

        # Per-item readiness gating: sim -> nav -> referee/evaluators -> waypoints.
        self.SIM_READY_TIMEOUT_S = 90
        self.NAV_READY_TIMEOUT_S = 120
        self.BATCH_READY_POLL_MS = 1500
        self._batch_generation = 0
        self._batch_item_ctx = None
        self._nav_ready = False

        self.nav_pages = [
            ("PREVIEW",       "01"),
            ("EVALUATION",    "02"),
            ("MISSION SETUP", "03"),
            ("LAUNCH",        "04"),
            ("REPORTS",       "05"),
        ]

        self.music_path = os.path.join(os.path.dirname(__file__), "You And Whose Army_.mp3")
        self.music_process = None
        self.music_muted = True

        self.init_ui()
        self._start_status_blink()
        
        # self.select_world(1, switch_page=False)
        self.show_page(0)

        self._report_timer = QTimer(self)
        self._report_timer.timeout.connect(self._refresh_report_links)
        self._report_timer.start(3000)

    # ------------------------------------------------------
    #  UI CONSTRUCTION
    # ------------------------------------------------------
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        outer_layout = QVBoxLayout(central_widget)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        outer_layout.addWidget(self._build_top_bar())
        outer_layout.addWidget(self._build_checkered_strip(height=4))

        body = QWidget()
        body_layout = QHBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(0)
        body_layout.addWidget(self._build_sidebar())
        
        # Central Stacked Widget
        self.stack = QStackedWidget()
        self.stack.addWidget(self._build_page_mission_preview())
        self.stack.addWidget(self._build_page_evaluation())
        self.stack.addWidget(self._build_page_mission_setup())
        self.stack.addWidget(self._build_page_launch())
        self.stack.addWidget(self._build_page_reports())
        body_layout.addWidget(self.stack, stretch=1)
        
        outer_layout.addWidget(body, stretch=1)
        outer_layout.addWidget(self._build_checkered_strip(height=4))
        outer_layout.addWidget(self._build_status_bar())


    def _build_top_bar(self):
        bar = QFrame()
        bar.setFixedHeight(80)
        bar.setStyleSheet(f"background-color: {PANEL_BLACK}; border-bottom: 2px solid {RACE_RED};")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(30, 10, 30, 10)

        self.btn_music = QPushButton("🔊 MUSIC: ON")
        self.btn_music.setCursor(Qt.PointingHandCursor)
        self.btn_music.setFont(QFont(MONO_FONT, 11, QFont.Bold))
        self.btn_music.setStyleSheet(f"background-color: {RACE_RED}; color: {WHITE}; border: 1px solid {RACE_RED}; font-weight: bold; border-radius: 4px; padding: 6px 14px;")
        self.btn_music.clicked.connect(self._toggle_music)
        layout.addWidget(self.btn_music)
        layout.addStretch()

        team_box = QVBoxLayout()
        team_box.setSpacing(2)
        name = QLabel("ASU  R O A R")
        name.setFont(QFont(STACK_FONT, 22, QFont.Black))
        name.setStyleSheet(f"color: {WHITE}; letter-spacing: 4px; border: none;")
        name.setAlignment(Qt.AlignCenter)
        tag = QLabel("SIMULATION & TESTING TEAM")
        tag.setFont(QFont(MONO_FONT, 10, QFont.Bold))
        tag.setStyleSheet(f"color: {RACE_RED}; letter-spacing: 3px; border: none;")
        tag.setAlignment(Qt.AlignCenter)
        team_box.addWidget(name)
        team_box.addWidget(tag)
        layout.addLayout(team_box)
        layout.addStretch()

        self.status_dot = QLabel("●")
        self.status_dot.setFont(QFont(MONO_FONT, 16))
        self.status_dot.setStyleSheet(f"color: {WHITE}; border: none;")
        self.status_text = QLabel("SYSTEM STANDBY")
        self.status_text.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        self.status_text.setStyleSheet(f"color: {STEEL}; letter-spacing: 2px; border: none;")

        status_box = QHBoxLayout()
        status_box.setSpacing(8)
        status_box.addWidget(self.status_dot)
        status_box.addWidget(self.status_text)
        status_box_widget = QWidget()
        status_box_widget.setLayout(status_box)
        status_box_widget.setFixedWidth(220)
        layout.addWidget(status_box_widget, alignment=Qt.AlignRight)
        return bar

    def _build_checkered_strip(self, height=4):
        strip = QFrame()
        strip.setFixedHeight(height)
        strip.setStyleSheet(f"""
            background: repeating-linear-gradient(
                45deg,
                {WHITE} 0px, {WHITE} 8px,
                {CARBON_BLACK} 8px, {CARBON_BLACK} 16px
            );
        """)
        return strip

    def _build_sidebar(self):
        sidebar = QFrame()
        sidebar.setFixedWidth(230)
        sidebar.setStyleSheet(f"background-color: {PANEL_BLACK}; border-right: 2px solid {PANEL_EDGE};")
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(20, 30, 20, 30)
        layout.setSpacing(14)

        section = QLabel("MISSION MENU")
        section.setFont(QFont(MONO_FONT, 11, QFont.Bold))
        section.setStyleSheet(f"color: {STEEL}; letter-spacing: 3px; border: none;")
        layout.addWidget(section)
        layout.addSpacing(6)

        self.nav_buttons = []
        for i, (name, num) in enumerate(self.nav_pages):
            btn = QPushButton(f" {num} — {name}")
            btn.setFont(QFont(STACK_FONT, 12, QFont.Bold))
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFixedHeight(50)
            btn.setStyleSheet(self._nav_btn_style(False))
            btn.clicked.connect(lambda checked, idx=i: self.show_page(idx))
            self.nav_buttons.append(btn)
            layout.addWidget(btn)

        layout.addStretch()

        # ASU ROAR Transparent Logo at Bottom Left
        logo_path = os.path.join(os.path.dirname(__file__), "asu_roar_logo_transparent.png")
        if os.path.exists(logo_path):
            lbl_logo = QLabel()
            pixmap = QPixmap(logo_path)
            scaled_pixmap = pixmap.scaledToWidth(180, Qt.SmoothTransformation)
            lbl_logo.setPixmap(scaled_pixmap)
            lbl_logo.setAlignment(Qt.AlignCenter)
            lbl_logo.setStyleSheet("border: none; background: transparent;")
            layout.addWidget(lbl_logo)
            layout.addSpacing(10)

        return sidebar

    def _nav_btn_style(self, active=False):
        if active:
            return f"""
                QPushButton {{
                    background-color: {RACE_RED};
                    color: {WHITE};
                    border: none;
                    border-radius: 4px;
                    text-align: left;
                    padding-left: 14px;
                }}
            """
        else:
            return f"""
                QPushButton {{
                    background-color: {PANEL_BLACK};
                    color: {STEEL};
                    border: 1px solid {PANEL_EDGE};
                    border-radius: 4px;
                    text-align: left;
                    padding-left: 14px;
                }}
                QPushButton:hover {{
                    border: 1px solid {RACE_RED};
                    color: {WHITE};
                }}
            """

    def show_page(self, index):
        self.stack.setCurrentIndex(index)
        for i, btn in enumerate(self.nav_buttons):
            btn.setStyleSheet(self._nav_btn_style(i == index))

    # ============================================================
    #  PAGES
    # ============================================================



    def _build_page_mission_preview(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)
        
        title = QLabel("MISSION MAP PREVIEW")
        title.setFont(QFont(STACK_FONT, 26, QFont.Black))
        title.setStyleSheet(f"color: {WHITE}; border: none;")
        layout.addWidget(title)
        layout.addSpacing(10)

        # Controls
        ctrl_layout = QHBoxLayout()
        lbl_w = QLabel("WORLD:")
        lbl_w.setStyleSheet(f"color: {STEEL}; font-weight: bold;")
        ctrl_layout.addWidget(lbl_w)
        
        self.preview_world = QSpinBox()
        self.preview_world.setRange(1, 3)
        self.preview_world.setStyleSheet(f"background-color: {PANEL_BLACK}; color: {WHITE}; font-size: 14px;")
        ctrl_layout.addWidget(self.preview_world)
        
        lbl_wp = QLabel("WP INDEX:")
        lbl_wp.setStyleSheet(f"color: {STEEL}; font-weight: bold;")
        ctrl_layout.addWidget(lbl_wp)
        
        self.preview_wp = QSpinBox()
        self.preview_wp.setRange(0, 299)
        self.preview_wp.setStyleSheet(f"background-color: {PANEL_BLACK}; color: {WHITE}; font-size: 14px;")
        ctrl_layout.addWidget(self.preview_wp)
        
        self.btn_preview = QPushButton("RENDER MAP")
        self.btn_preview.setStyleSheet(f"background-color: {RACE_RED}; color: {WHITE}; font-weight: bold; border-radius: 4px; padding: 5px 15px;")
        self.btn_preview.clicked.connect(self._load_preview)
        ctrl_layout.addWidget(self.btn_preview)
        
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)
        layout.addSpacing(20)

        # Plot area container - fills panel height nicely
        plot_container = QFrame()
        plot_container.setStyleSheet(f"background-color: {PANEL_BLACK}; border: 1px solid {PANEL_EDGE}; border-radius: 6px;")
        plot_layout = QHBoxLayout(plot_container)
        plot_layout.setContentsMargins(12, 12, 12, 12)
        plot_layout.setSpacing(15)

        # Info panel on the left
        self.preview_info_panel = QFrame()
        self.preview_info_panel.setStyleSheet(f"background-color: {CARBON_BLACK}; border: 1px solid {PANEL_EDGE}; border-radius: 6px;")
        self.preview_info_panel.setFixedWidth(320)
        info_layout = QVBoxLayout(self.preview_info_panel)
        info_layout.setContentsMargins(20, 20, 20, 20)
        
        info_title = QLabel("MISSION CONFIGURATION DETAILS")
        info_title.setFont(QFont(STACK_FONT, 12, QFont.Bold))
        info_title.setStyleSheet(f"color: {RACE_RED}; border: none;")
        info_layout.addWidget(info_title)
        
        self.preview_info_label = QLabel("Click 'RENDER MAP' to load details.")
        self.preview_info_label.setFont(QFont(STACK_FONT, 10))
        self.preview_info_label.setStyleSheet(f"color: {WHITE}; border: none;")
        self.preview_info_label.setWordWrap(True)
        self.preview_info_label.setAlignment(Qt.AlignTop)
        info_layout.addWidget(self.preview_info_label, stretch=1)
        
        plot_layout.addWidget(self.preview_info_panel)

        self.preview_figure, self.preview_ax = plt.subplots(figsize=(6.5, 6.5))
        self.preview_figure.patch.set_facecolor(PANEL_BLACK)
        self.preview_ax.set_facecolor(CARBON_BLACK)
        self.preview_ax.tick_params(colors=WHITE)
        for spine in self.preview_ax.spines.values():
            spine.set_edgecolor(PANEL_EDGE)
            
        self.preview_canvas = FigureCanvas(self.preview_figure)
        plot_layout.addWidget(self.preview_canvas, stretch=1)
        layout.addWidget(plot_container, stretch=1)
        
        return page

    def _load_preview(self):
        w = self.preview_world.value()
        wp = self.preview_wp.value()
        
        dev_world = os.path.join(self.sim_ws, f"dev_environment/worlds/world{w}")
        heightmap_path = os.path.join(dev_world, "heightmap.npz")
        obs_path = os.path.join(dev_world, "obstacle_data.npy")
        wp_file = os.path.join(self.mission_ws, f"missions/mission_{w}/waypoints/wp{wp:02d}/wp{wp:02d}.npy")
        
        self.preview_ax.clear()
        self.preview_ax.set_facecolor(CARBON_BLACK)
        self.preview_ax.tick_params(colors=WHITE)
        for spine in self.preview_ax.spines.values():
            spine.set_edgecolor(PANEL_EDGE)

        if hasattr(self, 'cbar') and self.cbar is not None:
            self.cbar.remove()
            self.cbar = None
        
        if os.path.exists(heightmap_path):
            h = np.load(heightmap_path)
            res = float(h.get('resolution', 0.25))
            grid_data = h['grid']
            origin_x = float(h.get('origin_x', -18.431461))
            origin_y = float(h.get('origin_y', -7.687172))
            
            # Mask out padding (-1.5m abyss) so it appears transparent/white
            grid_data = np.ma.masked_where(grid_data <= -1.49, grid_data)
            
            img = self.preview_ax.imshow(grid_data, cmap='terrain', origin='lower')
            
            # Add colorbar
            self.cbar = self.preview_ax.figure.colorbar(img, ax=self.preview_ax, fraction=0.046, pad=0.04)
            self.cbar.set_label("Elevation (m)", color=WHITE)
            self.cbar.ax.yaxis.set_tick_params(color=WHITE, labelcolor=WHITE)
            self.cbar.outline.set_edgecolor(PANEL_EDGE)
            
            # Plot Obstacles
            if os.path.exists(obs_path):
                obs_raw = np.load(obs_path, allow_pickle=True)
                items = obs_raw.flat if isinstance(obs_raw, np.ndarray) else obs_raw
                safe_x, safe_y = [], []
                coll_x, coll_y = [], []
                aruco_x, aruco_y = [], []
                for obj in items:
                    if isinstance(obj, dict):
                        px = (obj.get("x", 0.0) - origin_x) / res
                        py = (obj.get("y", 0.0) - origin_y) / res
                        ipx, ipy = int(round(px)), int(round(py))
                        # Only plot if obstacle lies within active terrain bounds
                        if 0 <= ipy < grid_data.shape[0] and 0 <= ipx < grid_data.shape[1]:
                            elev = grid_data[ipy, ipx]
                            if not np.isnan(elev) and elev > -1.49:
                                if obj.get("mesh_id") == -1:
                                    aruco_x.append(px)
                                    aruco_y.append(py)
                                elif obj.get("is_collidable", True) or obj.get("is_barrier", False):
                                    coll_x.append(px)
                                    coll_y.append(py)
                                else:
                                    safe_x.append(px)
                                    safe_y.append(py)
                
                if safe_x:
                    self.preview_ax.scatter(safe_x, safe_y, color='magenta', marker='x', s=20, label=f'Safe Pebbles ({len(safe_x)})')
                if coll_x:
                    self.preview_ax.scatter(coll_x, coll_y, color='red', marker='X', s=35, label=f'Collidable Obstacles ({len(coll_x)})')
                if aruco_x:
                    self.preview_ax.scatter(aruco_x, aruco_y, color='cyan', marker='s', s=45, edgecolors='black', linewidth=1.5, label=f'ArUco Markers ({len(aruco_x)})')
            
            # Plot Waypoints
            if os.path.exists(wp_file):
                wp_data = np.load(wp_file)
                
                spawn_px = (0.0 - origin_x) / res
                spawn_py = (0.0 - origin_y) / res

                # The generated waypoints are structured as: [Home, WP1, WP2, WP3, WP4, Home]
                # We want to extract only the actual intermediate waypoints.
                pts = list(wp_data)
                
                # Strip the first point (Home) and the last point (Home)
                if len(pts) > 2:
                    pts = pts[1:-1]
                
                # Limit to 4 waypoints just in case
                pts = pts[:4]
                
                path_x = [spawn_px] + [p[0] for p in pts] + [spawn_px]
                path_y = [spawn_py] + [p[1] for p in pts] + [spawn_py]

                self.preview_ax.plot(path_x, path_y, color='#00FF00', marker='o', linewidth=2, markersize=6, label='Driving Path')
                self.preview_ax.plot(spawn_px, spawn_py, color='cyan', marker='*', markersize=14, markeredgecolor='black', label='Rover Start (0,0)')
                
                # Annotate Rover Start
                self.preview_ax.annotate("ROVER START", (spawn_px, spawn_py), xytext=(0, -25), textcoords='offset points', ha='center',
                                         color='black', fontweight='bold', bbox=dict(boxstyle='square,pad=0.3', fc='cyan', ec='black', lw=1.5))

                # Annotate waypoints as WP 1, WP 2, WP 3...
                phys_pts = []
                for idx, pt in enumerate(pts):
                    px, py = pt[0], pt[1]
                    # Calculate physical meters
                    phys_x = origin_x + px * res
                    phys_y = origin_y + py * res
                    phys_pts.append((phys_x, phys_y))
                    
                    if idx == 0:
                        # WP 1: Green box
                        self.preview_ax.annotate("WP 1", (px, py), xytext=(-35, 5), textcoords='offset points',
                                                 color='white', fontweight='bold', bbox=dict(boxstyle='square,pad=0.3', fc='#009900', ec='black'))
                    elif idx == len(pts) - 1:
                        # Last WP: Black box with Red text
                        self.preview_ax.annotate(f"WP {idx+1}", (px, py), xytext=(5, -15), textcoords='offset points',
                                                 color='red', fontweight='bold', bbox=dict(boxstyle='square,pad=0.3', fc='black', ec='black'))
                    else:
                        # Intermediate WP
                        self.preview_ax.annotate(f"WP {idx+1}", (px, py), xytext=(5, 5), textcoords='offset points',
                                                 color='white', fontweight='bold', bbox=dict(boxstyle='square,pad=0.3', fc='black', ec='black'))

                # Calculate total path distance starting and ending at Rover Start (0,0)
                full_phys_path = [(0.0, 0.0)] + phys_pts + [(0.0, 0.0)]
                total_dist = 0.0
                for i in range(len(full_phys_path) - 1):
                    p1 = full_phys_path[i]
                    p2 = full_phys_path[i+1]
                    total_dist += math.hypot(p2[0] - p1[0], p2[1] - p1[1])

                leg = self.preview_ax.legend(bbox_to_anchor=(0.5, 1.08), loc='lower center', ncol=3, facecolor=PANEL_BLACK, edgecolor=PANEL_EDGE, fontsize=8.5, framealpha=0.9, borderpad=0.4, handletextpad=0.3, labelspacing=0.25)
                self.preview_ax.figure.subplots_adjust(top=0.78, bottom=0.08)
                for text in leg.get_texts():
                    text.set_color(WHITE)

                # Update HTML Details Panel
                details_html = (
                    f"<div style='font-family: Arial; line-height: 1.5; color: {WHITE};'>"
                    f"<p style='margin: 2px 0;'><b>World:</b> {w}</p>"
                    f"<p style='margin: 2px 0;'><b>Mission:</b> wp{wp:02d}</p>"
                    f"<p style='margin: 2px 0;'><b>Grid Size:</b> {grid_data.shape[1]}x{grid_data.shape[0]}</p>"
                    f"<p style='margin: 2px 0;'><b>Resolution:</b> {res:.2f} m/px</p>"
                    f"<p style='margin: 2px 0;'><b>Total Waypoints:</b> {len(pts)}</p>"
                    f"<p style='margin: 2px 0; color: #00FF00;'><b>Est. Distance:</b> {total_dist:.2f} m</p>"
                    f"<hr style='border: 0; border-top: 1px solid {PANEL_EDGE}; margin: 12px 0;'/>"
                    f"<p style='font-weight: bold; color: {RACE_RED}; margin-bottom: 6px;'>WAYPOINT COORD TABLE (m)</p>"
                    f"<table style='width: 100%; border-collapse: collapse; font-family: monospace; font-size: 11px;'>"
                )
                details_html += f"<tr style='border-bottom: 1px solid {PANEL_EDGE};'><td style='padding: 3px 0;'>Start</td><td style='text-align: right;'>(0.00, 0.00)</td></tr>"
                for idx, (px, py) in enumerate(phys_pts):
                    details_html += f"<tr style='border-bottom: 1px solid {PANEL_EDGE};'><td style='padding: 3px 0;'>WP {idx+1}</td><td style='text-align: right;'>({px:.2f}, {py:.2f})</td></tr>"
                details_html += f"<tr><td style='padding: 3px 0;'>Finish</td><td style='text-align: right;'>(0.00, 0.00)</td></tr>"
                details_html += "</table></div>"
                
                self.preview_info_label.setText(details_html)
            else:
                self.preview_ax.text(0.5, 0.5, "Waypoint Not Found", color=RACE_RED, ha='center', va='center', transform=self.preview_ax.transAxes, fontsize=16)
                self.preview_info_label.setText("<p style='color: red;'>Waypoint file not found.</p>")
        else:
            self.preview_ax.text(0.5, 0.5, "Terrain Map Not Found", color=RACE_RED, ha='center', va='center', transform=self.preview_ax.transAxes, fontsize=16)
            self.preview_info_label.setText("<p style='color: red;'>Terrain heightmap not found.</p>")
            
        self.preview_ax.set_title(f"World {w} — Mission wp{wp:02d}", color=WHITE, fontsize=14, fontweight='bold', pad=12)
        self.preview_ax.set_xlabel("Grid X", color=WHITE, labelpad=8)
        self.preview_ax.set_ylabel("Grid Y", color=WHITE, labelpad=8)
        
        # Scale to map dimensions exactly
        if os.path.exists(heightmap_path):
            self.preview_ax.set_xlim(0, grid_data.shape[1])
            self.preview_ax.set_ylim(0, grid_data.shape[0])
            
        self.preview_figure.tight_layout()
        self.preview_canvas.draw()

    def _build_page_mission_setup(self):

        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)
        
        title = QLabel("MISSION CONFIGURATION")
        title.setFont(QFont(STACK_FONT, 26, QFont.Black))
        title.setStyleSheet(f"color: {WHITE}; border: none;")
        layout.addWidget(title)
        layout.addSpacing(20)

        # -- Configuration Row --
        config_row = QHBoxLayout()
        
        # World SpinBoxes
        world_from_lbl = QLabel("FROM WORLD:")
        world_from_lbl.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        world_from_lbl.setStyleSheet(f"color: {STEEL};")
        config_row.addWidget(world_from_lbl)
        
        self.spin_world_from = QSpinBox()
        self.spin_world_from.setRange(1, 3)
        self.spin_world_from.setFixedHeight(40)
        self.spin_world_from.setStyleSheet(f"background-color: {PANEL_BLACK}; color: {WHITE}; font-size: 14px; font-weight: bold;")
        config_row.addWidget(self.spin_world_from)

        config_row.addSpacing(10)

        world_to_lbl = QLabel("TO WORLD:")
        world_to_lbl.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        world_to_lbl.setStyleSheet(f"color: {STEEL};")
        config_row.addWidget(world_to_lbl)
        
        self.spin_world_to = QSpinBox()
        self.spin_world_to.setRange(1, 3)
        self.spin_world_to.setFixedHeight(40)
        self.spin_world_to.setStyleSheet(f"background-color: {PANEL_BLACK}; color: {WHITE}; font-size: 14px; font-weight: bold;")
        config_row.addWidget(self.spin_world_to)
        
        config_row.addSpacing(20)
        
        # WP From SpinBox
        from_lbl = QLabel("FROM WP:")
        from_lbl.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        from_lbl.setStyleSheet(f"color: {STEEL};")
        config_row.addWidget(from_lbl)
        
        self.spin_wp_from = QSpinBox()
        self.spin_wp_from.setRange(0, 299)
        self.spin_wp_from.setValue(0)
        self.spin_wp_from.setFixedHeight(40)
        self.spin_wp_from.setStyleSheet(f"background-color: {PANEL_BLACK}; color: {WHITE}; font-size: 14px; font-weight: bold;")
        config_row.addWidget(self.spin_wp_from)
        
        config_row.addSpacing(10)
        
        # WP To SpinBox
        to_lbl = QLabel("TO WP:")
        to_lbl.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        to_lbl.setStyleSheet(f"color: {STEEL};")
        config_row.addWidget(to_lbl)
        
        self.spin_wp_to = QSpinBox()
        self.spin_wp_to.setRange(0, 299)
        self.spin_wp_to.setValue(3)
        self.spin_wp_to.setFixedHeight(40)
        self.spin_wp_to.setStyleSheet(f"background-color: {PANEL_BLACK}; color: {WHITE}; font-size: 14px; font-weight: bold;")
        config_row.addWidget(self.spin_wp_to)
        
        config_row.addSpacing(20)
        
        # Iterations SpinBox
        lbl_iter = QLabel("ITERATIONS:")
        lbl_iter.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        lbl_iter.setStyleSheet(f"color: {STEEL};")
        config_row.addWidget(lbl_iter)
        
        self.spin_iterations = QSpinBox()
        self.spin_iterations.setRange(1, 50)
        self.spin_iterations.setValue(1)
        self.spin_iterations.setFixedHeight(40)
        self.spin_iterations.setStyleSheet(f"background-color: {PANEL_BLACK}; color: {WHITE}; font-size: 14px; font-weight: bold; padding: 4px 8px; border: 1px solid {PANEL_EDGE}; border-radius: 4px;")
        config_row.addWidget(self.spin_iterations)

        config_row.addStretch()
        layout.addLayout(config_row)
        layout.addSpacing(20)
        
        # Add button
        self.btn_add_batch = QPushButton("ADD TO BATCH QUEUE")
        self.btn_add_batch.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        self.btn_add_batch.setFixedHeight(45)
        self.btn_add_batch.setStyleSheet(f"QPushButton {{ background-color: {RACE_RED}; color: {WHITE}; border-radius: 4px; }} QPushButton:hover {{ background-color: {RACE_RED_DIM}; }}")
        self.btn_add_batch.clicked.connect(self.add_selected_to_batch)
        layout.addWidget(self.btn_add_batch)
        
        layout.addSpacing(30)
        
        # -- Batch Queue List --
        batch_title = QLabel("CURRENT BATCH QUEUE:")
        batch_title.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        batch_title.setStyleSheet(f"color: {WHITE};")
        layout.addWidget(batch_title)
        
        self.batch_list_widget = QListWidget()
        self.batch_list_widget.setStyleSheet(f"QListWidget {{ background-color: {PANEL_BLACK}; color: {WHITE}; border: 1px solid {PANEL_EDGE}; padding: 10px; font-size: 14px; font-family: {MONO_FONT}; }}")
        layout.addWidget(self.batch_list_widget)
        
        # Remove button
        self.btn_remove_batch = QPushButton("REMOVE SELECTED ITEM")
        self.btn_remove_batch.setFixedHeight(40)
        self.btn_remove_batch.setStyleSheet(f"QPushButton {{ background-color: {PANEL_BLACK}; color: {WHITE}; border: 1px solid {PANEL_EDGE}; border-radius: 4px; font-weight: bold; }} QPushButton:hover {{ border: 1px solid {RACE_RED}; }}")
        self.btn_remove_batch.clicked.connect(self.remove_selected_batch)
        layout.addWidget(self.btn_remove_batch)
        
        layout.addStretch()
        return page

    def _build_page_launch(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)
        
        title = QLabel("ORCHESTRATION & BATCH QUEUE")
        title.setFont(QFont(STACK_FONT, 26, QFont.Black))
        title.setStyleSheet(f"color: {WHITE}; border: none;")
        layout.addWidget(title)
        layout.addSpacing(10)

        # -- Rover Module Selection --
        row_module = QHBoxLayout()
        lbl = QLabel("ROVER CONFIG:")
        lbl.setFont(QFont(MONO_FONT, 11, QFont.Bold))
        lbl.setStyleSheet(f"color: {STEEL};")
        row_module.addWidget(lbl)
        
        self.rover_combo = QComboBox()
        self.rover_combo.addItems(["Noisy", "Noise Free", "With Arm"])
        self.rover_combo.setFont(QFont(MONO_FONT, 11, QFont.Bold))
        self.rover_combo.setFixedHeight(40)
        self.rover_combo.setStyleSheet(f"""
            QComboBox {{
                background-color: {PANEL_BLACK};
                color: {WHITE};
                padding: 5px 14px;
                border: 1px solid {PANEL_EDGE};
                border-radius: 4px;
            }}
        """)
        row_module.addWidget(self.rover_combo)
        row_module.addStretch()
        layout.addLayout(row_module)
        layout.addSpacing(20)

        # -- Batch Queue Controls --
        batch_title = QLabel("EXECUTE BATCH QUEUE:")
        batch_title.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        batch_title.setStyleSheet(f"color: {WHITE};")
        layout.addWidget(batch_title)
        
        # -- Batch Execution Options --
        cb_style = f"""
            QCheckBox {{ color: {WHITE}; font-family: {MONO_FONT}; font-size: 14px; font-weight: bold; spacing: 12px; padding: 10px; background-color: {PANEL_BLACK}; border: 1px solid {PANEL_EDGE}; border-radius: 4px; }}
            QCheckBox::indicator {{ width: 20px; height: 20px; border: 1px solid {STEEL}; border-radius: 4px; background: {CARBON_BLACK}; }}
            QCheckBox::indicator:checked {{ background: {RACE_RED}; border: 1px solid {RACE_RED}; }}
        """
        
        opt_layout = QHBoxLayout()
        self.cb_headless = QCheckBox("Run Headless (No Gazebo GUI)")
        self.cb_headless.setStyleSheet(cb_style)
        self.cb_headless.setChecked(True)
        opt_layout.addWidget(self.cb_headless)
        
        self.cb_rviz = QCheckBox("Open RViz2")
        self.cb_rviz.setStyleSheet(cb_style)
        opt_layout.addWidget(self.cb_rviz)

        opt_layout.addStretch()
        layout.addLayout(opt_layout)
        layout.addSpacing(10)

        batch_btn_row = QHBoxLayout()
        self.btn_run_batch = QPushButton("RUN BATCH PIPELINE")
        self.btn_run_batch.setCursor(Qt.PointingHandCursor)
        self.btn_run_batch.setFixedHeight(48)
        self.btn_run_batch.setStyleSheet(f"QPushButton {{ background-color: {RACE_RED}; color: {WHITE}; border: none; border-radius: 4px; font-weight: bold; letter-spacing: 1px; }} QPushButton:hover {{ background-color: {WHITE}; color: {CARBON_BLACK}; }}")
        self.btn_run_batch.clicked.connect(self.run_batch_pipeline)
        batch_btn_row.addWidget(self.btn_run_batch)

        self.btn_clear_batch = QPushButton("CLEAR & RESET BATCH (KILL NODES)")
        self.btn_clear_batch.setCursor(Qt.PointingHandCursor)
        self.btn_clear_batch.setFixedHeight(48)
        self.btn_clear_batch.setStyleSheet(f"QPushButton {{ background-color: {PANEL_BLACK}; color: {RACE_RED}; border: 1px solid {RACE_RED}; border-radius: 4px; font-weight: bold; }} QPushButton:hover {{ background-color: {RACE_RED}; color: {WHITE}; }}")
        self.btn_clear_batch.clicked.connect(self.clear_batch_queue)
        batch_btn_row.addWidget(self.btn_clear_batch)
        
        self.btn_skip_test = QPushButton("SKIP CURRENT TEST")
        self.btn_skip_test.setCursor(Qt.PointingHandCursor)
        self.btn_skip_test.setFixedHeight(48)
        self.btn_skip_test.setStyleSheet(f"QPushButton {{ background-color: {STEEL}; color: {WHITE}; border: none; border-radius: 4px; font-weight: bold; letter-spacing: 1px; }} QPushButton:hover {{ background-color: {WHITE}; color: {CARBON_BLACK}; }}")
        self.btn_skip_test.clicked.connect(lambda: self._batch_abort_item("manually skipped by user"))
        self.btn_skip_test.setEnabled(False)
        batch_btn_row.addWidget(self.btn_skip_test)
        
        layout.addLayout(batch_btn_row)

        layout.addSpacing(20)
        
        # -- Run Status Monitor --
        status_lbl = QLabel("RUN STATUS MONITOR:")
        status_lbl.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        status_lbl.setStyleSheet(f"color: {STEEL};")
        layout.addWidget(status_lbl)
        
        self.status_box = QFrame()
        self.status_box.setStyleSheet(f"background-color: {PANEL_BLACK}; border: 1px solid {PANEL_EDGE}; border-radius: 4px;")
        status_layout = QGridLayout(self.status_box)
        
        self.lbl_tests_done = QLabel("Tests Completed: 0 / 0")
        self.lbl_tests_done.setStyleSheet(f"color: {WHITE}; font-family: {MONO_FONT}; font-size: 14px; border: none;")
        status_layout.addWidget(self.lbl_tests_done, 0, 0)
        
        self.lbl_tests_left = QLabel("Tests Remaining: 0")
        self.lbl_tests_left.setStyleSheet(f"color: {WHITE}; font-family: {MONO_FONT}; font-size: 14px; border: none;")
        status_layout.addWidget(self.lbl_tests_left, 0, 1)
        
        self.lbl_elapsed_time = QLabel("Elapsed Time: 00:00")
        self.lbl_elapsed_time.setStyleSheet(f"color: {WHITE}; font-family: {MONO_FONT}; font-size: 14px; border: none;")
        status_layout.addWidget(self.lbl_elapsed_time, 1, 0, 1, 2)
        layout.addWidget(self.status_box)
        
        layout.addSpacing(20)
        div = QFrame()
        div.setFixedHeight(1)
        div.setStyleSheet(f"background-color: {PANEL_EDGE};")
        layout.addWidget(div)
        layout.addSpacing(20)

        # -- Manual Launch --
        man_title = QLabel("MANUAL LAUNCH OVERRIDES:")
        man_title.setFont(QFont(MONO_FONT, 12, QFont.Bold))
        man_title.setStyleSheet(f"color: {STEEL};")
        layout.addWidget(man_title)

        man_row = QHBoxLayout()
        self.btn_sim = QPushButton("1. LAUNCH SIM")
        self.btn_sim.setFixedHeight(40)
        self.btn_sim.clicked.connect(self.launch_simulation)
        man_row.addWidget(self.btn_sim)
        
        self.btn_nav = QPushButton("2. LAUNCH NAV2")
        self.btn_nav.setFixedHeight(40)
        self.btn_nav.clicked.connect(self.launch_navstack)
        man_row.addWidget(self.btn_nav)

        self.btn_waypoints = QPushButton("3. SEND WPTS")
        self.btn_waypoints.setFixedHeight(40)
        self.btn_waypoints.clicked.connect(self.send_waypoints_manually)
        man_row.addWidget(self.btn_waypoints)
        
        self.btn_kill = QPushButton("TERMINATE ALL")
        self.btn_kill.setFixedHeight(40)
        self.btn_kill.setStyleSheet(f"color: {RACE_RED}; border: 1px solid {RACE_RED}; border-radius:4px; font-weight:bold;")
        self.btn_kill.clicked.connect(self.kill_all_nodes)
        man_row.addWidget(self.btn_kill)
        layout.addLayout(man_row)

        layout.addStretch()
        return page

    def _build_page_evaluation(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)
        
        title = QLabel("EVALUATION SUITES")
        title.setFont(QFont(STACK_FONT, 26, QFont.Black))
        title.setStyleSheet(f"color: {WHITE}; border: none;")
        layout.addWidget(title)
        layout.addSpacing(20)

        cb_style = f"""
            QCheckBox {{
                color: {WHITE};
                font-family: {MONO_FONT};
                font-size: 14px;
                font-weight: bold;
                spacing: 12px;
                padding: 10px;
                background-color: {PANEL_BLACK};
                border: 1px solid {PANEL_EDGE};
                border-radius: 4px;
            }}
            QCheckBox::indicator {{
                width: 20px; height: 20px;
                border: 1px solid {STEEL};
                border-radius: 4px;
                background: {CARBON_BLACK};
            }}
            QCheckBox::indicator:checked {{
                background: {RACE_RED};
                border: 1px solid {RACE_RED};
            }}
        """

        self.cb_slam = QCheckBox("SLAM EVALUATION ENABLED")
        self.cb_slam.setChecked(True)
        self.cb_slam.setStyleSheet(cb_style)
        layout.addWidget(self.cb_slam)

        self.cb_perception = QCheckBox("PERCEPTION EVALUATION ENABLED")
        self.cb_perception.setChecked(False)
        self.cb_perception.setStyleSheet(cb_style)
        layout.addWidget(self.cb_perception)

        self.cb_pursuit = QCheckBox("PURE PURSUIT EVALUATION ENABLED")
        self.cb_pursuit.setChecked(False)
        self.cb_pursuit.setStyleSheet(cb_style)
        layout.addWidget(self.cb_pursuit)

        layout.addStretch()
        return page

    def _build_page_reports(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)
        
        title = QLabel("LATEST REPORTS")
        title.setFont(QFont(STACK_FONT, 26, QFont.Black))
        title.setStyleSheet(f"color: {WHITE}; border: none;")
        layout.addWidget(title)
        layout.addSpacing(20)

        self.btn_open_referee_report = QPushButton("MASTER EVALUATION — No report yet")
        self.btn_open_referee_report.setFixedHeight(60)
        self.btn_open_referee_report.clicked.connect(lambda: self._open_report(self._latest_referee_pdf))
        layout.addWidget(self.btn_open_referee_report)

        self.btn_open_slam_report = QPushButton("SLAM — No report yet")
        self.btn_open_slam_report.setFixedHeight(60)
        self.btn_open_slam_report.clicked.connect(lambda: self._open_report(self._latest_slam_html))
        layout.addWidget(self.btn_open_slam_report)

        self.btn_open_perception_report = QPushButton("PERCEPTION — No report yet")
        self.btn_open_perception_report.setFixedHeight(60)
        self.btn_open_perception_report.clicked.connect(lambda: self._open_report(self._latest_perception_html))
        layout.addWidget(self.btn_open_perception_report)

        layout.addStretch()
        
        self._latest_referee_pdf = None
        self._latest_slam_html   = None
        self._latest_perception_html = None
        return page

    def _build_status_bar(self):
        bar = QFrame()
        bar.setFixedHeight(30)
        bar.setStyleSheet(f"background-color: {PANEL_BLACK};")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(20, 4, 20, 4)

        left = QLabel("ASU ROAR // MISSION CONTROL v2.1 (PAGINATED)")
        left.setFont(QFont(MONO_FONT, 8))
        left.setStyleSheet(f"color: {PANEL_EDGE}; letter-spacing: 1px; border: none;")
        layout.addWidget(left)
        layout.addStretch()
        return bar

    # ============================================================
    #  LOGIC
    # ============================================================

    def add_selected_to_batch(self):
        w_from = self.spin_world_from.value()
        w_to = self.spin_world_to.value()
        wp_from = self.spin_wp_from.value()
        wp_to = self.spin_wp_to.value()
        
        if w_from > w_to:
            QMessageBox.warning(self, "Invalid Range", "<font color='white'>'FROM WORLD' must be less than or equal to 'TO WORLD'.</font>")
            return
        if wp_from > wp_to:
            QMessageBox.warning(self, "Invalid Range", "<font color='white'>'FROM WP' must be less than or equal to 'TO WP'.</font>")
            return
            
        selected_indices = list(range(wp_from, wp_to + 1))
        
        try:
            for w in range(w_from, w_to + 1):
                costmap_path = os.path.join(self.mission_ws, f"missions/mission_{w}/assets/costmap.npz")
                c = np.load(costmap_path)
                res, ox, oy = float(c['resolution']), float(c['origin_x']), float(c['origin_y'])

                for wp_index in selected_indices:
                    mission_id = f"batch_{len(self.batch_queue) + 1}_w{w}_wp{wp_index}"
                    wp_str = f"wp{wp_index:02d}"
                    wp_file = os.path.join(self.mission_ws, f"missions/mission_{w}/waypoints/{wp_str}/{wp_str}.npy")
                    if not os.path.exists(wp_file):
                        wp_str = f"wp{wp_index}"
                        wp_file = os.path.join(self.mission_ws, f"missions/mission_{w}/waypoints/{wp_str}/{wp_str}.npy")

                    if os.path.exists(wp_file):
                        waypoints_m = []
                        wp_data = np.load(wp_file)
                        for col, row in wp_data:
                            x = round(ox + col * res, 3)
                            y = round(oy + row * res, 3)
                            waypoints_m.extend([x, y])

                        iterations = self.spin_iterations.value()
                        for iter_idx in range(iterations):
                            mission_id = f"batch_{len(self.batch_queue) + 1}_w{w}_wp{wp_index}_iter{iter_idx+1}"
                            self._save_gui_mission(waypoints_m, override_dir=mission_id)
                            self.batch_queue.append((w, mission_id, [wp_index], waypoints_m))
                            self.batch_list_widget.addItem(f"[{mission_id}] World {w} | WP: {wp_index}")
                    else:
                        print(f"Waypoint {wp_index} for World {w} not found. Skipping.")
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"<font color='white'>Failed to add to batch:<br>{str(e)}</font>")
            
    def remove_selected_batch(self):
        current_row = self.batch_list_widget.currentRow()
        if current_row >= 0:
            self.batch_list_widget.takeItem(current_row)
            self.batch_queue.pop(current_row)

    def _save_gui_mission(self, waypoints_m, override_dir=None):
        try:
            target_name = override_dir if override_dir else "gui_mission"
            mission_dir = os.path.join(self.referee_inputs_dir, target_name)
            os.makedirs(mission_dir, exist_ok=True)

            pairs = [(waypoints_m[i], waypoints_m[i + 1]) for i in range(0, len(waypoints_m), 2)]
            wp_array = np.array([[x, y, 1.0] for x, y in pairs], dtype=float)
            np.save(os.path.join(mission_dir, "wp_gui.npy"), wp_array)

            all_pts = [(0.0, 0.0)] + pairs
            segments = []
            for i in range(len(pairs)):
                seg_start = all_pts[i]
                seg_end   = all_pts[i + 1]
                segments.append([list(seg_start), list(seg_end)])
            path_array = np.array(segments, dtype=float)
            np.save(os.path.join(mission_dir, "optimal_path.npy"), path_array)

            self.current_gui_mission = target_name

        except Exception as e:
            QMessageBox.warning(self, "Warning", f"<font color='white'>Could not save mission array:<br>{str(e)}</font>")

    def update_yaml(self, waypoints_list):
        with open(self.yaml_path, 'r') as f:
            lines = f.readlines()
        new_lines = []
        in_waypoints_block = False
        for line in lines:
            if line.strip().startswith("waypoints: ["):
                new_lines.append(f"      waypoints: [{', '.join(map(str, waypoints_list))}]\n")
                if "]" not in line:
                    in_waypoints_block = True
            elif in_waypoints_block:
                if "]" in line:
                    in_waypoints_block = False
            else:
                new_lines.append(line)
        with open(self.yaml_path, 'w') as f:
            f.writelines(new_lines)

    def clear_batch_queue(self):
        self.batch_queue.clear()
        self.batch_list_widget.clear()
        self.completed_batch_count = 0
        self.total_batch_count = 0
        self._batch_generation += 1
        self._batch_item_ctx = None
        
        if hasattr(self, 'run_timer'):
            self.run_timer.stop()
        if hasattr(self, 'batch_timeout_timer'):
            self.batch_timeout_timer.stop()
        if hasattr(self, 'lbl_tests_done'):
            self.lbl_tests_done.setText("Tests Completed: 0 / 0")
        if hasattr(self, 'lbl_tests_left'):
            self.lbl_tests_left.setText("Tests Remaining: 0")
        if hasattr(self, 'lbl_elapsed_time'):
            self.lbl_elapsed_time.setText("Elapsed Time: 00:00")
        if hasattr(self, 'btn_run_batch'):
            self.btn_run_batch.setEnabled(True)
            self.btn_run_batch.setText("RUN BATCH PIPELINE")
        if hasattr(self, 'btn_skip_test'):
            self.btn_skip_test.setEnabled(False)

        # Terminate any running ROS 2 nodes, simulation processes, or followers
        self.kill_all_nodes()

    def run_batch_pipeline(self):
        if not self.batch_queue:
            QMessageBox.information(self, "Empty Queue", "<font color='white'>The batch queue is empty.</font>")
            return
            
        import glob
        import os
        for f in glob.glob(os.path.join(self.workspace_root, "reports", "master_report*.html")):
            try: os.remove(f)
            except: pass
        self.batch_aggregated_results = {}
        
        self.btn_run_batch.setEnabled(False)
        self.btn_run_batch.setText("BATCH RUNNING...")
        if hasattr(self, 'btn_skip_test'):
            self.btn_skip_test.setEnabled(True)
        
        self.total_batch_count = len(self.batch_queue)
        self.completed_batch_count = 0
        
        import time
        self.batch_start_time = time.time()
        self.run_timer.start(1000)
        self._update_run_time()
        
        self._process_next_batch_item()
        
    def _update_run_time(self):
        if self.batch_start_time is None: return
        import time
        elapsed = int(time.time() - self.batch_start_time)
        mins = elapsed // 60
        secs = elapsed % 60
        self.lbl_elapsed_time.setText(f"Elapsed Time: {mins:02d}:{secs:02d}")

    def _process_next_batch_item(self):
        tests_left = len(self.batch_queue)
        self.lbl_tests_done.setText(f"Tests Completed: {self.completed_batch_count} / {self.total_batch_count}")
        self.lbl_tests_left.setText(f"Tests Remaining: {tests_left}")
        
        if not self.batch_queue:
            self.run_timer.stop()
            self.btn_run_batch.setEnabled(True)
            self.btn_run_batch.setText("RUN BATCH PIPELINE")
            if hasattr(self, 'btn_skip_test'):
                self.btn_skip_test.setEnabled(False)
            QMessageBox.information(self, "Batch Complete", "<font color='white'>All items in the batch queue have been completed!</font>")
            return

        world_num, mission_id, wp_indices, waypoints_m = self.batch_queue.pop(0)
        
        for i in range(self.batch_list_widget.count()):
            item = self.batch_list_widget.item(i)
            if item and mission_id in item.text():
                self.batch_list_widget.takeItem(i)
                break
                
        self.update_yaml(waypoints_m)
        self.current_gui_mission = mission_id

        raw_cfg = self.rover_combo.currentText()
        cfg_map = {"Noisy": "noise", "Noise Free": "clean", "With Arm": "full"}
        rover_cfg = cfg_map.get(raw_cfg, raw_cfg.lower())
        rviz_val = "true" if hasattr(self, 'cb_rviz') and self.cb_rviz.isChecked() else "false"
        slam_val = "true" if hasattr(self, 'cb_slam') and self.cb_slam.isChecked() else "false"
        perc_val = "true" if hasattr(self, 'cb_perception') and self.cb_perception.isChecked() else "false"
        pursuit_val = "true" if hasattr(self, 'cb_pursuit') and self.cb_pursuit.isChecked() else "false"

        # Evaluators ALWAYS run to populate the master report.
        self._batch_item_ctx = {
            "mission_id": mission_id,
            "world_num": world_num,
            "wp_indices": wp_indices,
            "rover_cfg": rover_cfg,
            "rviz_val": rviz_val,
            "slam_val": slam_val,
            "perc_val": perc_val,
            "pursuit_val": pursuit_val
        }

        self._batch_generation += 1
        generation = self._batch_generation
        self.batch_process = None
        self.sim_ready = False

        self.batch_timeout_timer.stop()
        self.batch_timeout_timer.start(self.BATCH_ITEM_TIMEOUT_MS)

        # 1. Launch simulation — same command as the "1. LAUNCH SIM" button.
        world_name = f"world{world_num}"
        sim_cmd = (f"cd {self.ws_root} && source /opt/ros/humble/setup.bash && " \
                   f"source install/setup.bash && " \
                   f"cd src/roar2026/dev_environment && bash launch_test.sh {rover_cfg} {world_name}")
        self._open_in_terminal(sim_cmd, f"BATCH SIM: {mission_id}", keep_open=False)

        self._batch_wait_for_sim_ready(generation, elapsed=0)

    def _batch_wait_for_sim_ready(self, generation, elapsed):
        if generation != self._batch_generation:
            return  # item was aborted/skipped while waiting

        if self.sim_ready:
            self._batch_launch_nav(generation)
            return

        if elapsed >= self.SIM_READY_TIMEOUT_S:
            self._batch_abort_item("simulation never became ready")
            return

        QTimer.singleShot(
            self.BATCH_READY_POLL_MS,
            lambda: self._batch_wait_for_sim_ready(generation, elapsed + self.BATCH_READY_POLL_MS / 1000)
        )

    def _batch_launch_nav(self, generation):
        if generation != self._batch_generation:
            return

        ctx = self._batch_item_ctx
        # 2. Launch Nav2 — same command as the "2. LAUNCH NAV2" button, plus the
        # rviz/enable_slam args master_evaluation.launch.py already passes it today.
        nav_cmd = (f"cd {self.ws_root} && source /opt/ros/humble/setup.bash && " \
                   f"source install/setup.bash && " \
                   f"ros2 launch navstack_nav2 roar_nav2_only.launch.py "
                   f"rviz:={ctx['rviz_val']} enable_slam:={ctx.get('slam_val', 'true')}")
        self._open_in_terminal(nav_cmd, f"BATCH NAV2: {ctx['mission_id']}", keep_open=False)

        self._nav_ready = False
        self._batch_wait_for_nav_ready(generation, elapsed=0)

    def _check_nav_ready_worker(self):
        try:
            cmd = "source /opt/ros/humble/setup.bash && ros2 action list"
            result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=1.5)
            self._nav_ready = ("/navigate_to_pose" in result.stdout)
        except Exception:
            pass

    def _batch_wait_for_nav_ready(self, generation, elapsed):
        if generation != self._batch_generation:
            return  # item was aborted/skipped while waiting

        if self._nav_ready:
            self._batch_launch_referee_and_waypoints(generation)
            return

        if elapsed >= self.NAV_READY_TIMEOUT_S:
            self._batch_abort_item("nav2 never became ready")
            return

        threading.Thread(target=self._check_nav_ready_worker, daemon=True).start()
        QTimer.singleShot(
            self.BATCH_READY_POLL_MS,
            lambda: self._batch_wait_for_nav_ready(generation, elapsed + self.BATCH_READY_POLL_MS / 1000)
        )

    def _batch_launch_referee_and_waypoints(self, generation):
        if generation != self._batch_generation:
            return

        ctx = self._batch_item_ctx
        mission_id = ctx["mission_id"]

        # 3. Launch the referee/evaluators — mirrors the eval_launch include inside
        # master_evaluation.launch.py, now fired only once Nav2 is confirmed ready.
        referee_cmd = (\
            f"cd {self.ws_root} && source /opt/ros/humble/setup.bash && " \
            f"source install/setup.bash && " \
            f"ros2 launch referee_engine roar_full_evaluation.launch.py "
            f"mission:={mission_id} world_num:={ctx['world_num']} run_mock:=false "
            f"enable_slam:={ctx.get('slam_val', 'false')} enable_perception:={ctx.get('perc_val', 'true')} enable_pursuit:={ctx.get('pursuit_val', 'true')}"
        )
        self._open_in_terminal(referee_cmd, f"BATCH REFEREE: {mission_id}", keep_open=False)

        # 4. Send waypoints — same follower invocation as the "3. SEND WPTS" button,
        # with the same graceful teardown, tracked so we know when the item is done.
        kills = "pkill -2 -f ros2; pkill -2 -f ign; pkill -2 -f gazebo; pkill -2 -f ruby; pkill -2 -f rviz2; pkill -2 -f nav2"
        wp_cmd = (\
            f"cd {self.ws_root} && source /opt/ros/humble/setup.bash && " \
            f"source install/setup.bash && " \
            f"python3 {self.nav_ws}/path-planning/globalPlanner/scripts/batch_waypoint_follower.py --params {self.yaml_path} ; "
            f"echo 'Waypoints finished. Tearing down batch item...' ; sleep 2 ; {kills}"
        )

        self.batch_process = QProcess(self)
        self.batch_process.finished.connect(lambda: self._on_batch_process_finished(generation))
        self.batch_process.start("gnome-terminal", ["--wait", "--title", f"BATCH WPTS: {mission_id}", "--", "bash", "-c", wp_cmd])

    def _on_batch_process_finished(self, generation):
        if generation != self._batch_generation:
            return  # stale — this item was already aborted/skipped
        self.batch_timeout_timer.stop()
        self._compile_master_report(self.current_gui_mission)
        self.completed_batch_count += 1
        self.kill_all_nodes(show_msg=False) # Guarantee all leftover terminals and nodes are wiped!
        self._process_next_batch_item()

    def _on_batch_item_timeout(self):
        self._batch_abort_item(f"exceeded the {self.BATCH_ITEM_TIMEOUT_MS // 60000}-minute limit")

    def _batch_abort_item(self, reason):
        ctx = self._batch_item_ctx
        mission_id = ctx["mission_id"] if ctx else self.current_gui_mission
        print(f"[BATCH] '{mission_id}' skipped — {reason}.")

        self.batch_timeout_timer.stop()
        # Invalidate any in-flight readiness polls / finished-signal handlers for this item.
        self._batch_generation += 1

        # Ask everything to shut down gracefully first (same SIGINT path the normal
        # completion uses), so a component that's actually fine gets a clean exit.
        self.kill_all_nodes(show_msg=False)
        # Give it a short grace period to exit on its own; force-close the waypoints
        # terminal afterwards so the queue advances even if something ignored SIGINT.
        proc = self.batch_process
        self.batch_process = None
        QTimer.singleShot(10000, lambda: self._force_close_batch_process(proc))

        # No report to compile — the item didn't complete, and compiling now would
        # risk picking up a stale HTML left over from an earlier world.
        self.completed_batch_count += 1
        self._process_next_batch_item()

    def _force_close_batch_process(self, proc):
        if proc is not None and proc.state() != QProcess.NotRunning:
            proc.kill()

    def _compile_master_report(self, mission_id):
        import numpy as np
        import os
        import glob
        ctx = self._batch_item_ctx
        if not ctx: return
        
        # Read the stats from run_summary.npy
        summary_file = os.path.join(self.workspace_root, "reports", "referee", "run_summary.npy")
        if not os.path.exists(summary_file): return
        
        try:
            data = np.load(summary_file, allow_pickle=True).item()
        except:
            return
            
        status = data.get('terminal_status', 'UNKNOWN')
        score = data.get('final_score', 0.0)
        
        test_name = f"World {ctx['world_num']} WP {ctx['wp_indices'][0]}" if 'wp_indices' in ctx else f"World {ctx['world_num']} {mission_id}"
        w_name = f"World {ctx['world_num']}"
        wp_name = f"WP {ctx['wp_indices'][0]}" if 'wp_indices' in ctx else "Unknown"
        
        if test_name not in self.batch_aggregated_results:
            self.batch_aggregated_results[test_name] = {'w': w_name, 'wp': wp_name, 'iterations': 0, 'passed': 0, 'failed_collision': 0, 'timeout': 0, 'scores': [], 'reports': []}
            
        stats = self.batch_aggregated_results[test_name]
        stats['iterations'] += 1
        stats['scores'].append(score)
        if status == 'SUCCESS': stats['passed'] += 1
        elif status == 'COLLISION': stats['failed_collision'] += 1
        elif status in ['TIMEOUT', 'KILL']: stats['timeout'] += 1
        else: stats['timeout'] += 1 # Any other abort
        
        # Find the latest generated HTML report that is NOT live
        referee_dir = os.path.join(self.workspace_root, "reports", "referee")
        htmls = [f for f in glob.glob(os.path.join(referee_dir, "*.html")) if "Live" not in f]
        htmls.sort(key=os.path.getmtime, reverse=True)
        if htmls:
            stats['reports'].append(htmls[0])
        else:
            stats['reports'].append("#")
        
        master_html_path = os.path.join(self.workspace_root, f"reports/master_report_batch.html")
        os.makedirs(os.path.join(self.workspace_root, "reports"), exist_ok=True)
        
        html = "<html><head><style>body { font-family: sans-serif; background: #0B0B0D; color: #F2F2F2; padding: 20px; } table { width: 100%; border-collapse: collapse; margin-top: 10px; margin-bottom: 30px;} th, td { border: 1px solid #333; padding: 10px; text-align: center; } th { background: #1a1a1a; color: #E10600; } h2 { color: #00D9FF; border-bottom: 1px solid #333; padding-bottom: 10px; } a { text-decoration: none; font-weight: bold; } a:hover { text-decoration: underline; }</style></head><body>"
        html += "<h1 style='color: #E10600;'>ASU ROAR Master Batch Evaluation Report</h1>"
        
        # Table 1: Test Details
        html += "<h2>Test Details</h2><table><tr><th>Test Name</th><th>Iterations</th><th>Passed</th><th>Failed (Collision)</th><th>Timeout</th><th>Average Score</th><th>Evaluation Reports</th></tr>"
        for t_name, t_stats in self.batch_aggregated_results.items():
            avg_score = sum(t_stats['scores']) / len(t_stats['scores']) if t_stats['scores'] else 0
            
            links = []
            for i, rep in enumerate(t_stats['reports']):
                if rep != "#":
                    links.append(f"<a href='file://{rep}' target='_blank' style='color:#00D9FF;'>[Iter {i+1}]</a>")
            links_str = " ".join(links) if links else "None"
            
            html += f"<tr><td>{t_name}</td><td>{t_stats['iterations']}</td><td>{t_stats['passed']}</td><td>{t_stats['failed_collision']}</td><td>{t_stats['timeout']}</td><td>{avg_score:.2f}</td><td>{links_str}</td></tr>"
        html += "</table>"
        
        # Table 2: Benchmarking Per World
        world_aggs = {}
        for t_name, t_stats in self.batch_aggregated_results.items():
            w = t_stats['w']
            wp = t_stats['wp']
            if w not in world_aggs:
                world_aggs[w] = {'passed': 0, 'total': 0, 'wps': {}}
            world_aggs[w]['passed'] += t_stats['passed']
            world_aggs[w]['total'] += t_stats['iterations']
            
            if wp not in world_aggs[w]['wps']:
                world_aggs[w]['wps'][wp] = {'passed': 0, 'total': 0}
            world_aggs[w]['wps'][wp]['passed'] += t_stats['passed']
            world_aggs[w]['wps'][wp]['total'] += t_stats['iterations']

        html += "<h2>Benchmarking Overview (Per World)</h2><table><tr><th>World Name</th><th>Overall Success Rate</th><th>Top Waypoint</th><th>Lowest Waypoint</th></tr>"
        for w, w_stats in world_aggs.items():
            overall_sr = (w_stats['passed'] / w_stats['total'] * 100) if w_stats['total'] else 0
            
            top_wp = max(w_stats['wps'].items(), key=lambda x: x[1]['passed']/x[1]['total'] if x[1]['total'] else -1, default=(None, None))
            low_wp = min(w_stats['wps'].items(), key=lambda x: x[1]['passed']/x[1]['total'] if x[1]['total'] else 2, default=(None, None))
            
            top_str = f"{top_wp[0]} ({(top_wp[1]['passed']/top_wp[1]['total']*100):.1f}%)" if top_wp[0] else "N/A"
            low_str = f"{low_wp[0]} ({(low_wp[1]['passed']/low_wp[1]['total']*100):.1f}%)" if low_wp[0] else "N/A"
            
            html += f"<tr><td>{w}</td><td>{overall_sr:.1f}%</td><td>{top_str}</td><td>{low_str}</td></tr>"
        html += "</table></body></html>"
        
        with open(master_html_path, "w") as f:
            f.write(html)
        
        print(f"Master batch report updated: {master_html_path}")

    def launch_simulation(self):
        raw_cfg = self.rover_combo.currentText()
        cfg_map = {"Noisy": "noise", "Noise Free": "clean", "With Arm": "full"}
        rover_cfg = cfg_map.get(raw_cfg, raw_cfg.lower())
        w = self.spin_world_from.value()
        world_name = f"world{w}"
        cmd = (f"cd {self.ws_root} && source /opt/ros/humble/setup.bash && "
               f"source install/setup.bash && "
               f"cd src/roar2026/dev_environment && bash launch_test.sh {rover_cfg} {world_name}")
        self._open_in_terminal(cmd, "ROAR Simulation")

    def launch_navstack(self):
        cmd = (f"cd {self.ws_root} && source /opt/ros/humble/setup.bash && "
               f"source install/setup.bash && "
               f"ros2 launch navstack_nav2 roar_nav2_only.launch.py")
        self._open_in_terminal(cmd, "Nav2 Stack")

    def send_waypoints_manually(self):
        w = self.spin_world_from.value()
        wp_index = self.spin_wp_from.value()
        
        try:
            costmap_path = os.path.join(self.mission_ws, f"missions/mission_{w}/assets/costmap.npz")
            c = np.load(costmap_path)
            res, ox, oy = float(c['resolution']), float(c['origin_x']), float(c['origin_y'])
            wp_str = f"wp{wp_index:02d}"
            wp_file = os.path.join(self.mission_ws, f"missions/mission_{w}/waypoints/{wp_str}/{wp_str}.npy")
            if not os.path.exists(wp_file):
                wp_str = f"wp{wp_index}"
                wp_file = os.path.join(self.mission_ws, f"missions/mission_{w}/waypoints/{wp_str}/{wp_str}.npy")
            
            if not os.path.exists(wp_file):
                QMessageBox.warning(self, "Error", f"<font color='white'>Waypoint {wp_index} not found for World {w}</font>")
                return
                
            wp_data = np.load(wp_file)
            waypoints_m = []
            for col, row in wp_data:
                x = round(ox + col * res, 3)
                y = round(oy + row * res, 3)
                waypoints_m.extend([x, y])
                
            self.update_yaml(waypoints_m)
        except Exception as e:
            QMessageBox.warning(self, "Error", f"<font color='white'>Failed to load waypoints: {str(e)}</font>")
            return
            
        kills = "pkill -2 -f ros2; pkill -2 -f ign; pkill -2 -f gazebo; pkill -2 -f ruby; pkill -2 -f rviz2; pkill -2 -f nav2"
        cmd = (f"cd {self.ws_root} && source /opt/ros/humble/setup.bash && "
               f"source install/setup.bash && "
               f"python3 {self.nav_ws}/path-planning/globalPlanner/scripts/batch_waypoint_follower.py --params {self.yaml_path} ; "
               f"echo 'Waypoints finished. Tearing down simulation...' ; sleep 2 ; {kills}")
        self._open_in_terminal(cmd, f"Send W{w} WP{wp_index}")

    def kill_all_nodes(self, show_msg=True):
        targets = [
            "ros2", "ign", "gazebo", "ruby", "gzserver", "gzclient", "rviz2", "controller_manager",
            "erc_waypoint_follower.py", "nav2", "robot_state_publisher", "launch_test.sh", "asu_gui_term"
        ]
        for t in targets:
            subprocess.run(["pkill", "-2", "-f", t])
        self.sim_ready = False
        self.status_text.setText("SYSTEM STANDBY")
        if show_msg:
            QMessageBox.information(self, "System Shutdown", "<font color='white'>Systems terminated.</font>")

    def _open_in_terminal(self, bash_cmd, title, keep_open=True):
        import shlex
        suffix = "; exec bash" if keep_open else ""
        safe_cmd = shlex.quote(bash_cmd + suffix)
        subprocess.Popen(["gnome-terminal", "--title", title, "--", "bash", "-c", f"exec -a asu_gui_term bash -c {safe_cmd}"])

    def _start_status_blink(self):
        self.sim_ready = False
        self._blink_on = True
        self._blink_timer = QTimer(self)
        self._blink_timer.timeout.connect(self._toggle_status_dot)
        self._blink_timer.start(900)
        self._check_timer = QTimer(self)
        self._check_timer.timeout.connect(lambda: threading.Thread(target=self._check_sim_ready_worker, daemon=True).start())
        self._check_timer.start(2500)

    def _toggle_status_dot(self):
        self._blink_on = not self._blink_on
        if getattr(self, 'sim_ready', False):
            color = WHITE if self._blink_on else PANEL_EDGE
            self.status_text.setText("SIMULATION READY")
        else:
            color = RACE_RED if self._blink_on else PANEL_EDGE
            self.status_text.setText("SYSTEM STANDBY")
        self.status_dot.setStyleSheet(f"color: {color}; border: none;")

    def _check_sim_ready_worker(self):
        try:
            cmd = "source /opt/ros/humble/setup.bash && ros2 topic list"
            result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=1.5)
            self.sim_ready = ("/odom" in result.stdout)
        except Exception:
            pass

    def _refresh_report_links(self):
        import glob
        master_htmls = sorted(glob.glob(os.path.join(self.workspace_root, "reports", "master_report_*.html")), key=os.path.getmtime, reverse=True)
        if master_htmls:
            self._latest_referee_pdf = master_htmls[0]
            name = os.path.basename(master_htmls[0])
            self.btn_open_referee_report.setText(f"MASTER EVALUATION  {name[:40]}")
            self.btn_open_referee_report.setStyleSheet(f"background-color:{PANEL_BLACK}; color:{WHITE}; border:1px solid {WHITE}; font-weight:bold;")

        slam_htmls = sorted(glob.glob(os.path.join(self.slam_report_dir, "*.html")), key=os.path.getmtime, reverse=True)
        if slam_htmls:
            self._latest_slam_html = slam_htmls[0]
            name = os.path.basename(slam_htmls[0])
            self.btn_open_slam_report.setText(f"SLAM  {name[:40]}")
            self.btn_open_slam_report.setStyleSheet(f"background-color:{PANEL_BLACK}; color:{RACE_RED}; border:1px solid {RACE_RED}; font-weight:bold;")

        perception_html = os.path.join(self.mission_ws, "reports", "perception", "dashboard.html")
        if os.path.exists(perception_html):
            self._latest_perception_html = perception_html
            self.btn_open_perception_report.setText(f"PERCEPTION  WORLD {self.current_world}")
            self.btn_open_perception_report.setStyleSheet(f"background-color:{PANEL_BLACK}; color:#00D9FF; border:1px solid #00D9FF; font-weight:bold;")

    def _open_report(self, path):
        if path and os.path.exists(path):
            subprocess.Popen(["xdg-open", path])
        else:
            QMessageBox.information(self, "No Report", "<font color='white'>No report file found yet.</font>")

    def _start_music(self):
        if os.path.exists(self.music_path) and not self.music_muted:
            try:
                self.music_process = subprocess.Popen(
                    ["mpg123", "--loop", "-1", "-q", self.music_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except Exception as e:
                print("Audio playback error:", e)

    def _stop_music(self):
        if self.music_process is not None:
            try:
                self.music_process.terminate()
                self.music_process.wait(timeout=1)
            except Exception:
                try:
                    self.music_process.kill()
                except Exception:
                    pass
            self.music_process = None

    def _toggle_music(self):
        self.music_muted = not self.music_muted
        if self.music_muted:
            self._stop_music()
            if hasattr(self, 'btn_music'):
                self.btn_music.setText("🔇 MUSIC: OFF")
                self.btn_music.setStyleSheet(f"background-color: {CARBON_BLACK}; color: {STEEL}; border: 1px solid {PANEL_EDGE}; font-weight: bold; border-radius: 4px; padding: 6px 14px;")
        else:
            self._start_music()
            if hasattr(self, 'btn_music'):
                self.btn_music.setText("🔊 MUSIC: ON")
                self.btn_music.setStyleSheet(f"background-color: {RACE_RED}; color: {WHITE}; border: 1px solid {RACE_RED}; font-weight: bold; border-radius: 4px; padding: 6px 14px;")

    def closeEvent(self, event):
        self._stop_music()
        super().closeEvent(event)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # Generic button styling across the app (excluding custom styled ones)
    app.setStyleSheet(f"""
        QPushButton {{
            background-color: {PANEL_BLACK};
            color: {WHITE};
            border: 1px solid {PANEL_EDGE};
            border-radius: 4px;
            font-weight: bold;
        }}
        QPushButton:hover {{
            background-color: {PANEL_EDGE};
            border: 1px solid {RACE_RED};
        }}
        QMessageBox {{
            background-color: {PANEL_BLACK};
            color: {WHITE};
        }}
        QMessageBox QLabel {{
            color: {WHITE};
            background-color: transparent;
        }}
        QMessageBox QPushButton {{
            background-color: {PANEL_EDGE};
            color: {WHITE};
            border: 1px solid {STEEL};
            padding: 5px 15px;
        }}
    """)
    
    window = ASURoarGUI()
    window.show()
    sys.exit(app.exec_())