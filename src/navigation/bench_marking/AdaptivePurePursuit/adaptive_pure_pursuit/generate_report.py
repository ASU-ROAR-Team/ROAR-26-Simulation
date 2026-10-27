#!/usr/bin/env python3

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from reportlab.lib.pagesizes import letter
from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Image,
    Table,
    TableStyle
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors


def analyze_log_data(csv_path):
    """
    Expects CSV columns:
    timestamp, v_cmd, w_cmd, pose_x, pose_y, ld_dist, theta
    """

    df = pd.read_csv(csv_path)

    df['time_diff'] = df['timestamp'].diff()

    df['pose_delta'] = np.sqrt(
        df['pose_x'].diff() ** 2 +
        df['pose_y'].diff() ** 2
    )

    stuck_frames = df[
        (df['v_cmd'] > 0.1) &
        (df['pose_delta'] < 0.001)
    ]

    stuck_time_total = stuck_frames['time_diff'].sum()

    metrics = {
        "total_runtime":
            df['timestamp'].iloc[-1] -
            df['timestamp'].iloc[0],

        "max_loop_gap":
            df['time_diff'].max(),

        "stuck_duration":
            stuck_time_total,

        "avg_v_cmd":
            df['v_cmd'].mean(),

        "status":
            "FAIL (Stuck Detected)"
            if stuck_time_total > 3.0
            else "PASS"
    }

    return df, metrics



def generate_plots(
    df,
    output_img="/tmp/mppi_controller_metrics.png"
):

    plt.figure(figsize=(7, 3))

    runtime = (
        df['timestamp'] -
        df['timestamp'].iloc[0]
    )

    plt.plot(
        runtime,
        df['v_cmd'],
        label='v_cmd (m/s)'
    )

    plt.plot(
        runtime,
        df['pose_delta'] / df['time_diff'],
        label='v_actual (m/s)'
    )

    plt.title(
        "Controller Command vs Displacement Velocity"
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")

    plt.legend()
    plt.grid(True)

    plt.tight_layout()

    plt.savefig(
        output_img,
        dpi=200
    )

    plt.close()



def build_pdf_report(
    metrics,
    plot_path,
    pdf_path="/tmp/erc_navigation_report.pdf"
):

    doc = SimpleDocTemplate(
        pdf_path,
        pagesize=letter
    )

    styles = getSampleStyleSheet()

    story = []

    title_style = ParagraphStyle(
        'TitleStyle',
        parent=styles['Heading1'],
        fontSize=20
    )

    story.append(
        Paragraph(
            "ERC Autonomous Navigation Analytics Report",
            title_style
        )
    )

    story.append(
        Spacer(1, 12)
    )


    table_data = [

        [
            "Metric",
            "Value",
            "Status"
        ],

        [
            "Total Runtime",
            f"{metrics['total_runtime']:.2f}s",
            "N/A"
        ],

        [
            "Max Loop Gap",
            f"{metrics['max_loop_gap']:.3f}s",
            "OK"
            if metrics['max_loop_gap'] < 0.5
            else "WARN"
        ],

        [
            "Stuck Duration",
            f"{metrics['stuck_duration']:.2f}s",
            "OK"
            if metrics['stuck_duration'] <= 3.0
            else "FAIL"
        ],

        [
            "Overall Result",
            metrics['status'],
            metrics['status']
        ]
    ]


    table = Table(
        table_data
    )


    table.setStyle(
        TableStyle(
            [
                (
                    'GRID',
                    (0,0),
                    (-1,-1),
                    0.5,
                    colors.grey
                ),

                (
                    'BACKGROUND',
                    (0,0),
                    (-1,0),
                    colors.lightgrey
                )
            ]
        )
    )


    story.append(table)

    story.append(
        Spacer(1,18)
    )


    story.append(
        Paragraph(
            "Velocity Trace",
            styles['Heading2']
        )
    )

    story.append(
        Image(
            plot_path,
            width=500,
            height=214
        )
    )


    doc.build(
        story
    )



def main():

    csv_path = (
        "/tmp/mppi_controller_metrics.csv"
    )

    plot_path = (
        "/tmp/mppi_controller_metrics.png"
    )

    pdf_path = (
        "/tmp/erc_navigation_report.pdf"
    )


    if not os.path.exists(csv_path):

        print(
            f"ERROR: CSV not found: {csv_path}"
        )

        return


    df, metrics = analyze_log_data(
        csv_path
    )


    generate_plots(
        df,
        plot_path
    )


    build_pdf_report(
        metrics,
        plot_path,
        pdf_path
    )


    print(
        f"Report generated: {pdf_path}"
    )



if __name__ == "__main__":
    main()