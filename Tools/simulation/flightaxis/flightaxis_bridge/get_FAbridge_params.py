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

That rule is ENFORCED here, not just documented: a row on a slot one of those
options rewrites is rejected unless it carries an explicit 0..1 "disarm".
Everything else this script validates is likewise a case the bridge itself
accepts and then gets silently wrong - out-of-range rf/px4, a negative-but-not
-1 disarm, a non-bool "reverse", duplicate rf/px4 - so failing early here is
the only place a user finds out.
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

# Must track src/flightaxis_bridge.cpp. RF_CHANNELS is the size of the channel
# array sent to FlightAxis; controls[] in HIL_ACTUATOR_CONTROLS is 16 wide.
# Both limits are checked here because the bridge fails SILENTLY past them:
# buildChannels() skips an out-of-range rf entirely, and an out-of-range px4
# reads as NaN forever so the channel sits at its disarm value for the whole
# flight. Neither prints anything.
RF_CHANNELS = 12
PX4_CONTROLS = 16


def fail(msg):
    sys.stderr.write("get_FAbridge_params: %s\n" % msg)
    sys.exit(1)


def number(value, where, key):
    """float() that reports a bad value instead of raising a traceback."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        fail("%s '%s' must be a number, got %r" % (where, key, value))
    return float(value)


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

if not isinstance(channels, list):
    fail("'Channels' must be a JSON array, got %s" % type(channels).__name__)

if not channels:
    # The runners only guard against EMPTY output, and "0 0.5 0" is not empty,
    # so this would otherwise start a bridge that drives every RealFlight
    # channel from UnmappedDefault and never responds to PX4 at all.
    fail("'Channels' is empty - the bridge would map no outputs at all")

if not isinstance(options, list):
    fail("'Options' must be a JSON array, got %s (a bare string is not a list; "
         "write [\"HeliDemix\"], not \"HeliDemix\")" % type(options).__name__)

# The bridge constrains this to 0..1 without comment, so an out-of-range value
# silently becomes 0 or 1 rather than the value written here.
unmapped_default = number(data.get('UnmappedDefault', 0.5), "'%s'" % filename,
                          'UnmappedDefault')
if not 0.0 <= unmapped_default <= 1.0:
    fail("'UnmappedDefault' must be between 0 and 1, got %g" % unmapped_default)

bitmask = 0
for opt in options:
    if opt not in OPTION_BITS:
        fail("unknown option '%s'; expected one of %s"
             % (opt, ", ".join(sorted(OPTION_BITS))))
    bitmask |= OPTION_BITS[opt]

# Accumulated and written only once every row has validated, so a failure part
# way through can never leave a truncated argv on stdout
out = [str(bitmask), str(unmapped_default), str(len(channels))]
rows = []

for i, c in enumerate(channels):
    where = "Channels[%d]" % i

    if not isinstance(c, dict):
        fail("%s must be a JSON object, got %s" % (where, type(c).__name__))

    check_keys(c, CHANNEL_KEYS, where)

    def index(key, limit):
        if key not in c:
            fail("%s has no '%s' key" % (where, key))
        v = c[key]
        # int() would silently truncate, so "rf": 1.9 would map channel 1
        if not isinstance(v, int) or isinstance(v, bool):
            fail("%s '%s' must be an integer, got %r" % (where, key, v))
        if not 0 <= v < limit:
            fail("%s '%s' must be between 0 and %d, got %d"
                 % (where, key, limit - 1, v))
        return v

    rf = index('rf', RF_CHANNELS)
    px4 = index('px4', PX4_CONTROLS)

    scale = SCALE_CODES.get(c.get('scale', 'unipolar'))
    if scale is None:
        fail("%s unknown scale '%s'; expected one of %s"
             % (where, c.get('scale'), ", ".join(sorted(SCALE_CODES))))

    # "reverse": "false" is a truthy string and would silently reverse the
    # surface, which reads as an airframe sign error rather than a typo.
    if 'reverse' in c and not isinstance(c['reverse'], bool):
        fail("%s 'reverse' must be true or false, got %r" % (where, c['reverse']))
    reverse = 1 if c.get('reverse', False) else 0

    disarm = number(c.get('disarm', -1), where, 'disarm')
    # -1 is the sentinel for hold-last. The bridge's test is `disarm >= 0`, so
    # any OTHER negative value silently selects hold-last instead of the value
    # written here - the one mode this file's header warns is unsafe. Values
    # above 1 are constrained by the bridge, so "disarm": 100 on a motor row
    # quietly means full throttle.
    if disarm != -1.0 and not 0.0 <= disarm <= 1.0:
        fail("%s 'disarm' must be between 0 and 1, or exactly -1 for hold-last; "
             "got %g" % (where, disarm))

    rows.append((rf, px4, disarm, 'disarm' in c))
    out.append('%d %d %d %d %g' % (rf, px4, scale, reverse, disarm))

# ---------------------------------------------------------------------------
# Cross-row checks. The bridge rejects duplicates too, but in SITL it has
# already been backgrounded by then, so its exit is invisible and PX4 simply
# blocks forever on TCP 4560 with no diagnostic. Failing here instead makes the
# runners' "get_FAbridge_params.py failed" branch fire before anything starts.
# ---------------------------------------------------------------------------
for key, pos in (('rf', 0), ('px4', 1)):
    seen = {}
    for i, row in enumerate(rows):
        if row[pos] in seen:
            fail("'%s' is mapped twice: Channels[%d] and Channels[%d] both use "
                 "%s %d" % (key, seen[row[pos]], i, key, row[pos]))
        seen[row[pos]] = i

# ---------------------------------------------------------------------------
# Option/row interaction. HeliDemix and Rev4Servos are applied to the bridge's
# channel array AFTER the per-row mapping, so they rewrite slots whether or not
# a row wrote them this frame. See the module docstring.
# ---------------------------------------------------------------------------
mapped_rf = {row[0]: row for row in rows}

if bitmask & OPTION_BITS['HeliDemix']:
    missing = [n for n in (0, 1, 2) if n not in mapped_rf]
    if missing:
        # The demix reads rf0-2 unconditionally; an unmapped one is seeded from
        # UnmappedDefault, so the roll/pitch/collective triple it produces is
        # a blend of real swash output and a constant.
        fail("HeliDemix requires rf0, rf1 and rf2 to be the three swash servos, "
             "but %s not mapped by any row"
             % (", ".join("rf%d" % n for n in missing)
                + (" is" if len(missing) == 1 else " are")))

covered = set()
if bitmask & OPTION_BITS['HeliDemix']:
    covered |= {0, 1, 2}
if bitmask & OPTION_BITS['Rev4Servos']:
    covered |= set(range(8))

for n in sorted(covered):
    row = mapped_rf.get(n)
    if row is None:
        continue
    rf, _px4, disarm, explicit = row
    if disarm == -1.0:
        opt = 'HeliDemix' if n < 3 and (bitmask & OPTION_BITS['HeliDemix']) \
            else 'Rev4Servos'
        fail("rf%d %s but %s rewrites that slot every frame, so \"hold last\" "
             "is re-transformed rather than held (HeliDemix diverges and rails "
             "within a few frames; Rev4Servos ping-pongs the value at frame "
             "rate). Give it an explicit \"disarm\" - 0.5 for a bipolar servo, "
             "0.0 for a motor."
             % (rf, "has no \"disarm\" key" if not explicit
                else "sets \"disarm\": -1 (hold last)", opt))

sys.stdout.write(" ".join(out) + "  \n")
