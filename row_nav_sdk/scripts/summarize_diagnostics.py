#!/usr/bin/env python3
"""Summarize caller-exported diagnostics CSV; no runtime or camera dependency."""
import argparse
import csv
import json
import math

FIELDS = ('undistort_ms', 'inference_ms', 'preprocess_ms', 'input_setup_ms',
          'npu_run_ms', 'output_copy_ms', 'postprocess_ms', 'geometry_tracking_ms',
          'total_ms', 'frame_age_ms')
DETAIL = set(FIELDS[2:7])

def summarize(rows):
    rows = [row for row in rows if row.get('available') == '1']
    result = {'available_samples': len(rows), 'stages': {}}
    for field in FIELDS:
        values = []
        for row in rows:
            if field in DETAIL and row.get('detailed_inference') != '1':
                continue
            try:
                value = float(row[field])
            except (KeyError, ValueError, TypeError):
                continue
            if math.isfinite(value) and value >= 0:
                values.append(value)
        if values:
            values.sort()
            result['stages'][field] = dict(samples=len(values), mean=sum(values)/len(values),
                p50=values[math.ceil(.5*len(values))-1],
                p95=values[math.ceil(.95*len(values))-1], maximum=values[-1])
    return result

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    args = parser.parse_args()
    with open(args.csv, newline='', encoding='utf-8') as source:
        print(json.dumps(summarize(list(csv.DictReader(source))), indent=2, ensure_ascii=False))
