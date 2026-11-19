#!/usr/bin/env python3
"""Write a copy of the shared RViz layout (rviz/bitt.rviz) with the replay-only display groups switched on.

  rviz_layout.py SRC DST
"""
import sys

import yaml

REPLAY_GROUPS = {'nvblox', 'Bag replay'}


def main():
    src, dst = sys.argv[1:3]
    with open(src) as f:
        cfg = yaml.safe_load(f)
    for display in cfg['Visualization Manager']['Displays']:
        if display.get('Name') in REPLAY_GROUPS:
            display['Enabled'] = True
    with open(dst, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=False, indent=2)


main()
