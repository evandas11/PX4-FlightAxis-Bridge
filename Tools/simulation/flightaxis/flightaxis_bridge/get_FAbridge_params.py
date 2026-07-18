#!/usr/bin/env python3
"""Flatten models/<model>.json to flightaxis_bridge argv tokens.

Mirrors get_FGbridge_params.py: sitl_run.sh backtick-substitutes the output
into the bridge command line.

Output (space separated):
    <options_bitmask> <unmapped_default> <nchannels> [rf px4 scale reverse disarm]*nchannels

  options bits: 1=ResetPosition 2=Rev4Servos 4=HeliDemix 8=SilenceFPS
  scale:   0=unipolar (clamp 0..1), 1=bipolar ((v+1)/2)
  reverse: 0/1 (applied after scaling: v -> 1-v)
  disarm:  0..1 value sent when disarmed/NaN, or -1 = hold last/neutral 0.5
"""

import json
import sys
import os

OPTION_BITS = {
    "ResetPosition": 1,
    "Rev4Servos": 2,
    "HeliDemix": 4,
    "SilenceFPS": 8,
}

SCALE_CODES = {
    "unipolar": 0,
    "bipolar": 1,
}

if len(sys.argv) != 2:
    exit(-1)

filename = sys.argv[1]

if not os.path.exists('./' + filename):
    exit(-1)

with open(filename) as json_file:
    data = json.load(json_file)
    channels = data['Channels']
    options = data.get('Options', [])
    unmapped_default = float(data.get('UnmappedDefault', 0.5))

bitmask = 0
for opt in options:
    if opt not in OPTION_BITS:
        sys.stderr.write("get_FAbridge_params: unknown option '%s'\n" % opt)
        exit(-1)
    bitmask |= OPTION_BITS[opt]

print(bitmask, end=" ")
print(unmapped_default, end=" ")
print(len(channels), end=" ")

for c in channels:
    rf = int(c['rf'])
    px4 = int(c['px4'])
    scale = SCALE_CODES.get(c.get('scale', 'unipolar'))
    if scale is None:
        sys.stderr.write("get_FAbridge_params: unknown scale '%s'\n" % c.get('scale'))
        exit(-1)
    reverse = 1 if c.get('reverse', False) else 0
    disarm = float(c.get('disarm', -1))
    print('%d %d %d %d %g' % (rf, px4, scale, reverse, disarm), end=" ")

print(' ')
