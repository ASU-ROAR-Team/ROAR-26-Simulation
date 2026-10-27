#!/usr/bin/env python3
"""
Filters a path CSV that has extra columns (index, x_pixel, y_pixel, cost,
real_x, real_y) down to the plain x,y format used by combined_offline_path.csv
/ edited_offline_path.csv, so it can be dropped straight into the path editor.

Uses x_pixel/y_pixel * RESOLUTION so the path lands on the same cells as in
costmap.csv. (real_x/real_y are in a shifted meter frame and do not match
this costmap's (0,0) origin.)
"""

import csv

import pathlib as _pl
_ROOT = _pl.Path(__file__).resolve().parent.parent.parent
INPUT_FILE  = str(_ROOT / 'reference' / 'real_path.csv')   # change if needed
OUTPUT_FILE = str(_ROOT / 'data' / 'converted_path.csv')
RESOLUTION = 0.05

with open(INPUT_FILE, 'r', newline='') as f_in:
    reader = csv.DictReader(f_in)
    rows = [
        (float(row['x_pixel']) * RESOLUTION, float(row['y_pixel']) * RESOLUTION)
        for row in reader
    ]

with open(OUTPUT_FILE, 'w', newline='') as f_out:
    writer = csv.writer(f_out)
    writer.writerow(['x', 'y'])
    writer.writerows(rows)

print(f"Wrote {len(rows)} points to {OUTPUT_FILE}")
