#!/usr/bin/env python3
import argparse
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--terminal', required=True)
p.add_argument('--debug', required=True)
p.add_argument('--mode', choices=['trap', 'direct'], required=True)
args = p.parse_args()

term = Path(args.terminal).read_text(errors='ignore')
dbg = Path(args.debug).read_text(errors='ignore')

needles = [
    'vLPI IRQ INTID=',
    'latency=',
    'EOI done',
]
for needle in needles:
    if needle not in term:
        raise SystemExit(f'missing guest marker: {needle}')

if args.mode == 'trap':
    mode_needle = 'vLPI trap inject pINTID='
else:
    mode_needle = 'vLPI direct inject vINTID='

if mode_needle not in dbg:
    raise SystemExit(f'missing ITS mode marker: {mode_needle}')

print(f'PASS: {args.mode} mode closed-loop verified')
