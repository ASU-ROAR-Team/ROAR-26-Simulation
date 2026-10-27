import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import datetime
import json
from pathlib import Path

class ReportGeneratorNode(Node):
    """
    Generates HTML report and terminal log summary for ERC Navigation Task evaluations.
    """
    def __init__(self):
        super().__init__('report_generator_node')

        ws_root = os.environ.get('ROAR_WS', str(Path(__file__).resolve().parents[4]))
        default_report_dir = os.path.join(ws_root, 'reports', 'referee')
        self.declare_parameter('output_dir', default_report_dir)
        self.declare_parameter('mission_name', 'Marsyard')
        self.output_dir = self.get_parameter('output_dir').value
        self.mission_name = self.get_parameter('mission_name').value
        os.makedirs(self.output_dir, exist_ok=True)

        self.score_sub = self.create_subscription(
            String, '/referee/score', self.score_callback, 10
        )
        self.latest_score = "N/A"
        self.report_generated = False
        self.get_logger().info(f"Report Generator Node Initialized. Output Dir -> {self.output_dir}")

    def score_callback(self, msg: String):
        try:
            self.latest_score = json.loads(msg.data)
            if isinstance(self.latest_score, dict):
                status = self.latest_score.get("terminal_status", "RUNNING")
                # Always generate live report file
                self.generate_terminal_report(status, is_live=True)

                if self.latest_score.get("is_final", False) and not self.report_generated:
                    self.get_logger().info(f"Final score payload received for status '{status}'. Saving final HTML report...")
                    self.generate_terminal_report(status, is_live=False)
                    self.report_generated = True
        except json.JSONDecodeError:
            self.latest_score = msg.data

    def generate_terminal_report(self, status: str, is_live: bool = True):
        score_val = "0.00"
        waypoints = []
        nav_010 = "0.0"
        nav_020 = "0.0"
        nav_030 = "0.0"
        nav_040 = "0.0"
        nav_050 = "0.0"
        nav_060 = "0.0"
        elapsed = "0.0"
        t_min = "0.0"
        t_max = "0.0"
        act_dist = "0.0"
        ideal_dist = "0.0"

        if isinstance(self.latest_score, dict):
            s = self.latest_score
            score_val = f"{s.get('final_score', 0):.2f}"
            waypoints = s.get('waypoint_details', [])
            nav_010 = f"{s.get('nav_010_waypoint_reached', 0):.1f}"
            nav_020 = f"{s.get('nav_020_autonomy_bonus', 0):.1f}"
            nav_030 = f"{s.get('nav_030_back_to_start', 0):.1f}"
            nav_040 = f"{s.get('nav_040_back_to_start_autonomy', 0):.1f}"
            nav_050 = f"{s.get('nav_050_traversal_plan', 0):.1f}"
            nav_060 = f"{s.get('nav_060_return_penalty', 0):.1f}"
            elapsed = f"{s.get('elapsed_time_sec', 0):.1f} s"
            t_min = f"{s.get('t_min_computed_sec', 0):.1f} s"
            t_max = f"{s.get('t_max_computed_sec', 0):.1f} s"
            act_dist = f"{s.get('actual_distance_m', 0):.2f} m"
            ideal_dist = f"{s.get('ideal_distance_m', 0):.2f} m"

        is_success = status in ("COMPLETED", "SUCCESS")
        status_colour = "#00D26A" if is_success else "#E10600"
        status_bg = "#0a1f0e" if is_success else "#1a0500"
        score_class = "green" if is_success else "red"

        wp_rows = ""
        for wp in waypoints:
            is_failed = wp.get("status") == "FAILED" or not wp.get("reached", True)
            if is_failed:
                err_str = f"<span style='color:#ff4d4d; font-weight:bold;'>FAILED ({wp.get('positional_error', 0.0):.3f} m)</span>"
            else:
                err_str = f"{wp.get('positional_error', 0.0):.3f} m"

            wp_rows += (
                f"<tr>"
                f"<td>{wp.get('id','N/A')}</td>"
                f"<td>{wp.get('timestamp',0.0):.2f}</td>"
                f"<td>{err_str}</td>"
                f"<td>{wp.get('d_edge',0.0):.3f} m</td>"
                f"<td>{wp.get('score_010',0.0):.1f}</td>"
                f"<td>{wp.get('score_020',0.0):.1f}</td>"
                f"<td>{wp.get('total_score',0.0):.1f}</td>"
                f"</tr>"
            )
        if not wp_rows:
            wp_rows = "<tr><td colspan='7' style='color:#888;'>No waypoints reached</td></tr>"

        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>ERC Navigation Task — Scoring Report</title>
    <style>
        * {{ box-sizing: border-box; margin: 0; padding: 0; }}
        body {{
            font-family: 'Consolas', 'Helvetica Neue', Arial, sans-serif;
            background: #0B0B0D;
            color: #F2F2F2;
            padding: 30px;
            max-width: 900px;
            margin: 0 auto;
        }}
        .header {{
            background: #151518;
            border-bottom: 4px solid #E10600;
            padding: 24px;
            margin-bottom: 24px;
        }}
        .header h1 {{ font-size: 24px; color: #F2F2F2; letter-spacing: 2px; }}
        .header h1 span {{ color: #E10600; }}
        .header p {{ color: #8A8D93; font-size: 11px; margin-top: 6px; letter-spacing: 1px; }}

        .status-banner {{
            background: {status_bg};
            border: 1px solid {status_colour};
            color: {status_colour};
            text-align: center;
            padding: 12px;
            margin-bottom: 24px;
            font-weight: bold;
            font-size: 14px;
            letter-spacing: 2px;
            border-radius: 4px;
        }}

        .cards {{
            display: flex;
            gap: 12px;
            margin-bottom: 24px;
            flex-wrap: wrap;
        }}
        .card {{
            background: #151518;
            border: 1px solid #232327;
            border-top: 3px solid #E10600;
            padding: 14px;
            flex: 1;
            min-width: 120px;
            text-align: center;
            border-radius: 4px;
        }}
        .card-title {{ font-size: 9px; color: #8A8D93; text-transform: uppercase; margin-bottom: 6px; }}
        .card-value {{ font-size: 20px; font-weight: bold; color: #F2F2F2; }}
        .card-value.green {{ color: #00D26A; }}
        .card-value.red {{ color: #E10600; }}

        .section-title {{
            font-size: 11px;
            font-weight: bold;
            color: #8A8D93;
            letter-spacing: 2px;
            text-transform: uppercase;
            margin-bottom: 12px;
            padding-bottom: 6px;
            border-bottom: 1px solid #232327;
        }}

        table {{ width: 100%; border-collapse: collapse; margin-bottom: 24px; font-size: 12px; }}
        th {{ background: #151518; color: #8A8D93; padding: 8px 10px; text-align: center; font-size: 9px; border-bottom: 1px solid #E10600; }}
        td {{ border: 1px solid #232327; padding: 8px 10px; text-align: center; color: #F2F2F2; }}
        tr:nth-child(even) td {{ background: #151518; }}

        .footer {{ color: #333; font-size: 9px; text-align: center; margin-top: 30px; border-top: 1px solid #222; padding-top: 12px; }}
    </style>
</head>
<body>

<div class="header">
    <h1>ERC <span>NAVIGATION TASK</span></h1>
    <p>OFFICIAL AUTOMATED REFEREE EVALUATION REPORT (TRAVERSE SUB-TASK: MAX 300 PTS)</p>
    <p>Generated: {timestamp_str}</p>
</div>

<div class="status-banner">MISSION STATUS: {status}</div>

<div class="cards">
    <div class="card">
        <div class="card-title">Final Score</div>
        <div class="card-value {score_class}">{score_val} / 300</div>
    </div>
    <div class="card">
        <div class="card-title">Elapsed Time</div>
        <div class="card-value">{elapsed}</div>
    </div>
    <div class="card">
        <div class="card-title">Computed T_min</div>
        <div class="card-value">{t_min}</div>
    </div>
    <div class="card">
        <div class="card-title">Computed T_max</div>
        <div class="card-value">{t_max}</div>
    </div>
    <div class="card">
        <div class="card-title">Actual Dist</div>
        <div class="card-value">{act_dist}</div>
    </div>
    <div class="card">
        <div class="card-title">Ideal Dist</div>
        <div class="card-value">{ideal_dist}</div>
    </div>
</div>

<div class="section-title">ERC Rule Breakdown</div>
<table>
    <thead>
        <tr>
            <th>Rule No.</th>
            <th>Scored Parameter</th>
            <th>Max Points</th>
            <th>Score Awarded</th>
        </tr>
    </thead>
    <tbody>
        <tr><td>O/NAV-010</td><td>Waypoint Reached</td><td>120</td><td>{nav_010}</td></tr>
        <tr><td>O/NAV-020</td><td>Autonomy Bonus</td><td>100</td><td>{nav_020}</td></tr>
        <tr><td>O/NAV-030</td><td>Back to Start</td><td>25</td><td>{nav_030}</td></tr>
        <tr><td>O/NAV-040</td><td>Back to Start Autonomy Bonus</td><td>25</td><td>{nav_040}</td></tr>
        <tr><td>O/NAV-050</td><td>Execution of Traversal Plan</td><td>30</td><td>{nav_050}</td></tr>
        <tr><td>O/NAV-060</td><td>Return Penalty (-5 per return)</td><td>-</td><td>{nav_060}</td></tr>
    </tbody>
</table>

<div class="section-title">Waypoint & Positional Error Details</div>
<table>
    <thead>
        <tr>
            <th>Waypoint ID</th>
            <th>Time (s)</th>
            <th>Positional Error (m)</th>
            <th>Edge Distance d (m)</th>
            <th>O/NAV-010 Pts</th>
            <th>O/NAV-020 Pts</th>
            <th>Total Waypoint Pts</th>
        </tr>
    </thead>
    <tbody>
        {wp_rows}
    </tbody>
</table>

<div class="footer">ASU ROAR // ERC AUTOMATED REFEREE ENGINE</div>

</body>
</html>"""

        if is_live:
            html_path = os.path.join(self.output_dir, "ERC_Report_Live.html")
        else:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            html_path = os.path.join(self.output_dir, f"ERC_Report_{timestamp}_{self.mission_name}.html")

        with open(html_path, 'w') as f:
            f.write(html)


def main(args=None):
    rclpy.init(args=args)
    node = ReportGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
