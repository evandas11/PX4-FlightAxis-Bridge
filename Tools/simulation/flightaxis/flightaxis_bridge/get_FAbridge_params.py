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

Prefer an explicit "disarm" on every row. -1 ("hold last") is unsafe on any
channel covered by an in-place option: the bridge applies HeliDemix and
Rev4Servos to the persistent channels[] at the end of every buildChannels(),
so a held slot is re-transformed each frame rather than held. HeliDemix then
diverges and rails within a few frames at ~250 Hz; Rev4Servos ping-pongs the
value between rf i and rf i+4. See models/heli.json DisarmComment.
"""

import json
import sys

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

# Every key this script actually consumes. Anything else at the top level is a
# typo, and a typo is silent damage: 'Optons'/'options' would leave the bitmask
# at 0, so e.g. HeliDemix would be off and raw swash-servo values would go
# straight to RealFlight's roll/pitch/collective inputs. Free-form documentation
# is still allowed via any *Comment key, which is how these files annotate
# themselves (AileronComment, DemixComment, YawComment, ...).
TOP_LEVEL_KEYS = {"Options", "Channels", "UnmappedDefault", "RfModel", "Comment"}
CHANNEL_KEYS = {"rf", "px4", "scale", "reverse", "disarm", "comment"}


def fail(msg):
    sys.stderr.write("get_FAbridge_params: %s\n" % msg)
    exit(-1)


def check_keys(keys, allowed, where):
    for k in keys:
        if k in allowed or k.lower().endswith("comment"):
            continue
        near = [a for a in sorted(allowed) if a.lower() == k.lower()]
        hint = " (did you mean '%s'?)" % near[0] if near else ""
        fail("unknown key '%s' in %s%s; expected one of %s or a *Comment key"
             % (k, where, hint, ", ".join(sorted(allowed))))


if len(sys.argv) != 2:
    fail("usage: get_FAbridge_params.py <model.json>")

filename = sys.argv[1]

# Use the filename exactly as given: prefixing './' would break absolute paths
# ('.//etc/hostname' resolves relative to cwd). open() reports a missing or
# unreadable file better than a pre-flight existence check can.
try:
    with open(filename) as json_file:
        data = json.load(json_file)
except OSError as e:
    fail("cannot open '%s': %s" % (filename, e.strerror))
except ValueError as e:
    fail("'%s' is not valid JSON: %s" % (filename, e))

if not isinstance(data, dict):
    fail("'%s' must contain a JSON object, got %s" % (filename, type(data).__name__))

check_keys(data, TOP_LEVEL_KEYS, "'%s'" % filename)

if 'Channels' not in data:
    fail("'%s' has no 'Channels' key" % filename)

channels = data['Channels']
options = data.get('Options', [])
unmapped_default = float(data.get('UnmappedDefault', 0.5))

if not isinstance(channels, list):
    fail("'Channels' must be a JSON array, got %s" % type(channels).__name__)

if not isinstance(options, list):
    fail("'Options' must be a JSON array, got %s" % type(options).__name__)

bitmask = 0
for opt in options:
    if opt not in OPTION_BITS:
        fail("unknown option '%s'; expected one of %s"
             % (opt, ", ".join(sorted(OPTION_BITS))))
    bitmask |= OPTION_BITS[opt]

# Accumulated and written only once every row has validated, so a failure part
# way through can never leave a truncated argv on stdout
out = [str(bitmask), str(unmapped_default), str(len(channels))]

for i, c in enumerate(channels):
    where = "Channels[%d]" % i

    if not isinstance(c, dict):
        fail("%s must be a JSON object, got %s" % (where, type(c).__name__))

    check_keys(c, CHANNEL_KEYS, where)

    def index(key):
        if key not in c:
            fail("%s has no '%s' key" % (where, key))
        v = c[key]
        # int() would silently truncate, so "rf": 1.9 would map channel 1
        if not isinstance(v, int) or isinstance(v, bool):
            fail("%s '%s' must be an integer, got %r" % (where, key, v))
        return v

    rf = index('rf')
    px4 = index('px4')
    scale = SCALE_CODES.get(c.get('scale', 'unipolar'))
    if scale is None:
        fail("%s unknown scale '%s'; expected one of %s"
             % (where, c.get('scale'), ", ".join(sorted(SCALE_CODES))))
    reverse = 1 if c.get('reverse', False) else 0
    disarm = float(c.get('disarm', -1))
    out.append('%d %d %d %d %g' % (rf, px4, scale, reverse, disarm))

sys.stdout.write(" ".join(out) + "  \n")
