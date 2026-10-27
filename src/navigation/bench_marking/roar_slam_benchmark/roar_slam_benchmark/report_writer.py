import os
import json
import csv
import matplotlib.pyplot as plt
import numpy as np

class ReportWriter:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
    def write_trajectory_csv(self, raw_data):
        filepath = os.path.join(self.output_dir, "trajectory_error.csv")
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['t', 'gt_x', 'gt_y', 'gt_z', 'est_x', 'est_y', 'est_z', 'error'])
            for row in raw_data:
                writer.writerow(row)
                
    def write_json_summary(self, trajectory_stats, drift_metrics, phantom_events, topo_metrics, final_score):
        filepath = os.path.join(self.output_dir, "slam_score_report.json")
        
        report = {
            "final_slam_score": final_score,
            "trajectory_metrics": trajectory_stats,
            "drift_metrics": drift_metrics,
            "phantom_drift_events": phantom_events,
            "topography_metrics": topo_metrics
        }
        
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=4)
            
    def plot_trajectory_error(self, times, errors):
        if not times:
            return
            
        plt.figure(figsize=(10, 5))
        plt.plot(times, errors, label='Trajectory Error (m)', color='blue')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (m)')
        plt.title('SLAM Trajectory Error Over Time')
        plt.grid(True)
        plt.legend()
        
        filepath = os.path.join(self.output_dir, "trajectory_error.png")
        plt.savefig(filepath)
        plt.close()

    def write_html_report(self, times, errors, final_score_data, traj_stats, extra_stats=None):
        import base64
        extra_stats = extra_stats or {}
        
        # Plot to base64 so HTML is completely standalone
        plot_base64 = ""
        if times:
            plot_path = os.path.join(self.output_dir, "trajectory_error.png")
            fig = plt.figure(figsize=(10, 4))
            plt.plot(times, errors, label='Trajectory Error (m)', color='#3b4cc0')
            plt.axhline(y=traj_stats['mean_error'], color='#d32f2f', linestyle='--', linewidth=1, label='Mean Error')
            plt.xlabel('Time (s)')
            plt.ylabel('Error (m)')
            plt.title('SLAM Trajectory Error Trace')
            plt.grid(True, linestyle='--', alpha=0.6)
            plt.legend()
            plt.tight_layout()
            plt.savefig(plot_path, dpi=200, format='png', bbox_inches='tight')
            plt.close(fig)
            
            with open(plot_path, "rb") as image_file:
                plot_base64 = base64.b64encode(image_file.read()).decode('utf-8')

        hard_fail = final_score_data['hard_fail_triggered']
        overall_status_title = "OVERALL SYSTEM EVALUATION: CRITICAL DRIFT DETECTED / MISSION FAILED" if hard_fail else "OVERALL SYSTEM EVALUATION: STATE ESTIMATION NOMINAL"
        status_bg = "#111" if hard_fail else "#000"
        
        # Card values
        traj_rmse = final_score_data['components']['trajectory']['raw_value']
        drift_rate = final_score_data['components']['drift_rate']['raw_value']
        static_drift = final_score_data['components']['static_drift']['raw_value']
        f1_score = final_score_data['components']['topography']['raw_value']['obstacle_f1']
        
        html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>ASU ROAR - SLAM Diagnostics Report</title>
    <style>
        body {{ font-family: 'Helvetica Neue', Helvetica, Arial, sans-serif; color: #333; margin: 40px auto; max-width: 900px; }}
        .header {{ background-color: #000; padding: 25px 20px; border-bottom: 5px solid #d32f2f; }}
        .header h1 {{ margin: 0; color: #fff; font-size: 24px; font-weight: 800; letter-spacing: 1px; }}
        .header h1 span {{ color: #d32f2f; }}
        .header p {{ color: #aaa; margin: 8px 0 0 0; font-size: 12px; }}
        
        .status-banner {{ background-color: {status_bg}; color: #fff; text-align: center; padding: 15px; margin: 25px 0; border-top: 3px solid #d32f2f; border-bottom: 3px solid #d32f2f; font-weight: bold; font-size: 14px; letter-spacing: 1px; }}
        
        .section-title {{ font-size: 14px; font-weight: bold; margin-top: 30px; margin-bottom: 15px; border-bottom: 2px solid #000; padding-bottom: 8px; }}
        
        .cards-container {{ display: flex; justify-content: space-between; margin-bottom: 30px; gap: 15px; }}
        .card {{ border: 1px solid #ddd; border-top: 3px solid #d32f2f; padding: 15px 10px; flex: 1; text-align: center; background: #fff; box-shadow: 0 2px 4px rgba(0,0,0,0.02); }}
        .card-title {{ font-size: 9px; color: #666; text-transform: uppercase; font-weight: bold; margin-bottom: 10px; min-height: 22px; display: flex; align-items: center; justify-content: center; }}
        .card-value {{ font-size: 18px; font-weight: bold; color: #000; }}
        .text-red {{ color: #d32f2f; }}
        
        table {{ width: 100%; border-collapse: collapse; margin-bottom: 30px; font-size: 11px; box-shadow: 0 1px 3px rgba(0,0,0,0.05); }}
        th {{ background-color: #000; color: #fff; padding: 12px; text-align: center; text-transform: uppercase; font-size: 9px; border-right: 1px solid #444; letter-spacing: 0.5px; }}
        th:last-child {{ border-right: none; }}
        td {{ border: 1px solid #eee; padding: 12px; text-align: center; }}
        tr:nth-child(even) {{ background-color: #fafafa; }}
        
        .plot-container {{ margin-top: 30px; border-top: 2px solid #d32f2f; padding-top: 20px; text-align: center; }}
        .plot-title {{ font-size: 14px; font-weight: bold; text-align: left; margin-bottom: 15px; color: #333; }}
        img {{ max-width: 100%; height: auto; border: 1px solid #ddd; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }}
    </style>
</head>
<body>

    <div style="font-size: 9px; color: #666; margin-bottom: 10px; text-transform: uppercase; letter-spacing: 1px;">ASU ROAR — SLAM Diagnostics Report</div>
    
    <div class="header">
        <h1>ASU <span>ROAR</span></h1>
        <p>State-Estimation Fidelity & Performance Evaluation</p>
    </div>
    
    <div class="status-banner">
        {overall_status_title}
    </div>
    
    <div class="section-title">System Summary Diagnostics</div>
    
    <div class="cards-container">
        <div class="card">
            <div class="card-title">FINAL SLAM SCORE</div>
            <div class="card-value {"text-red" if hard_fail else ""}">{final_score_data['final_score']:.2f} / 100</div>
        </div>
        <div class="card">
            <div class="card-title">TRAJECTORY RMSE</div>
            <div class="card-value">{traj_rmse:.4f} m</div>
        </div>
        <div class="card">
            <div class="card-title">MEAN POSITION ERROR</div>
            <div class="card-value">{traj_stats['mean_error']:.4f} m</div>
        </div>
        <div class="card">
            <div class="card-title">AVG YAW ERROR</div>
            <div class="card-value">{extra_stats.get('avg_yaw_err', 0.0):.2f} °</div>
        </div>
        <div class="card">
            <div class="card-title">STATIC DRIFT</div>
            <div class="card-value {"text-red" if static_drift > 0.1 else ""}">{static_drift:.4f} m/s</div>
        </div>
    </div>
    
    <div class="section-title">Whole-System Statistical Analysis</div>
    
    <table>
        <thead>
            <tr>
                <th style="text-align: left;">METRIC VECTOR</th>
                <th>RAW VALUE</th>
                <th>NORMALIZED SCORE</th>
                <th>WEIGHT</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td style="text-align: left; font-weight: bold;">Trajectory Deviation</td>
                <td>{traj_rmse:.4f} m</td>
                <td>{final_score_data['components']['trajectory']['normalized_score']:.4f}</td>
                <td>{final_score_data['components']['trajectory']['weight']:.2f}</td>
            </tr>
            <tr>
                <td style="text-align: left; font-weight: bold;">Mean Position Error</td>
                <td>{traj_stats['mean_error']:.4f} m</td>
                <td style="color: #888;">N/A</td>
                <td style="color: #888;">N/A</td>
            </tr>
            <tr>
                <td style="text-align: left; font-weight: bold;">Avg Yaw Error</td>
                <td>{extra_stats.get('avg_yaw_err', 0.0):.2f} °</td>
                <td style="color: #888;">N/A</td>
                <td style="color: #888;">N/A</td>
            </tr>
            <tr>
                <td style="text-align: left; font-weight: bold;">Avg X-Axis Error</td>
                <td>{extra_stats.get('avg_x_err', 0.0):.4f} m</td>
                <td style="color: #888;">N/A</td>
                <td style="color: #888;">N/A</td>
            </tr>
            <tr>
                <td style="text-align: left; font-weight: bold;">Avg Y-Axis Error</td>
                <td>{extra_stats.get('avg_y_err', 0.0):.4f} m</td>
                <td style="color: #888;">N/A</td>
                <td style="color: #888;">N/A</td>
            </tr>
            <tr>
                <td style="text-align: left; font-weight: bold;">Windowed Drift Rate</td>
                <td>{drift_rate:.4f} m/s</td>
                <td>{final_score_data['components']['drift_rate']['normalized_score']:.4f}</td>
                <td>{final_score_data['components']['drift_rate']['weight']:.2f}</td>
            </tr>
            <tr>
                <td style="text-align: left; font-weight: bold;">Phantom Drift (Parked)</td>
                <td class="{"text-red" if static_drift > 0.1 else ""}">{static_drift:.4f} m/s</td>
                <td>{final_score_data['components']['static_drift']['normalized_score']:.4f}</td>
                <td>{final_score_data['components']['static_drift']['weight']:.2f}</td>
            </tr>
            <tr>
                <td style="text-align: left; font-weight: bold;">Topography Classification</td>
                <td>{f1_score:.4f} F1</td>
                <td>{final_score_data['components']['topography']['normalized_score']:.4f}</td>
                <td>{final_score_data['components']['topography']['weight']:.2f}</td>
            </tr>
        </tbody>
    </table>
    
    <div class="plot-container">
        <div class="plot-title">Continuous Tracking Telemetry</div>
        <img src="data:image/png;base64,{plot_base64}" alt="Trajectory Plot">
    </div>

</body>
</html>"""

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(self.output_dir, f"slam_score_report_{timestamp}.html")
        with open(filepath, 'w') as f:
            f.write(html)
        return filepath

    def generate_full_report(self, traj_tracker, drift_calculator, static_monitor, topo_validator, scorer, extra_stats=None):
        print(f"Generating SLAM Score Report in: {self.output_dir}")
        
        times = traj_tracker.times
        errors = traj_tracker.errors
        
        self.write_trajectory_csv(traj_tracker.raw_data)
        self.plot_trajectory_error(times, errors)
        
        traj_stats = traj_tracker.get_stats()
        drift_metrics = drift_calculator.get_drift_metrics(times, errors)
        phantom_score = static_monitor.get_phantom_drift_score()
        
        topo_metrics = {
            "elevation_rmse": 0.0,
            "coverage_fraction": 0.0,
            "obstacle_precision": 0.0,
            "obstacle_recall": 0.0,
            "obstacle_f1_score": 0.0
        }
        
        final_score = scorer.compute(
            traj_rmse=traj_stats['rmse'],
            max_windowed_drift_rate=drift_metrics['max_windowed_drift_rate'],
            phantom_drift_score=phantom_score,
            elevation_rmse=topo_metrics['elevation_rmse'],
            obstacle_f1_score=topo_metrics['obstacle_f1_score']
        )
        
        self.write_json_summary(
            traj_stats,
            drift_metrics,
            static_monitor.phantom_drift_events,
            topo_metrics,
            final_score
        )
        
        html_path = self.write_html_report(times, errors, final_score, traj_stats, extra_stats or {})
        print(f"Report generated successfully. Final Score: {final_score['final_score']:.2f}/100")
        print(f"Average Trajectory Error (Mean): {traj_stats['mean_error']:.4f} m")
        print(f"Trajectory RMSE:                 {traj_stats['rmse']:.4f} m")
        if extra_stats:
            print(f"Average Yaw Error:               {extra_stats.get('avg_yaw_err', 0.0):.2f} deg")
            print(f"Average X Error:                 {extra_stats.get('avg_x_err', 0.0):.4f} m")
            print(f"Average Y Error:                 {extra_stats.get('avg_y_err', 0.0):.4f} m")
        print(f"HTML Report Link: file://{html_path}")
        
        try:
            import webbrowser
            webbrowser.open(f"file://{html_path}")
        except Exception as e:
            print(f"Could not automatically open HTML report: {e}")
