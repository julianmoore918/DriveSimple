#!/bin/bash
# Supervisor for odd_endurance.py — survives a CARLA hang.
#
# Why this exists
# ---------------
# A CARLA RPC against an unresponsive server raises carla::client::
# TimeoutException, a C++ exception Python cannot catch: it calls
# terminate() and takes the process down with no traceback and no final
# report. sim_adapter.py's own comments warn about this. It killed a 2.6 h
# campaign at run 22 of 24 with 5.08 km banked (odd_50km_v4) — the server
# process was still alive and had 21 GB free, it had simply stopped
# answering, on the sixth town reboot of the run.
#
# The campaign's own LegWatchdog cannot help: it fires on a STALL, and this
# is an abrupt kill. So supervision has to be external.
#
# Loop: check the server answers, restart it if not, run the campaign with
# --resume so exposure accumulates in one output directory, and repeat if
# the process dies before the target is met.
#
#   ./scenarios/run_odd_supervised.sh 50 8
#                                     ^  ^ max hours
#                                     target km
set -u
TARGET_KM=${1:-50}
MAX_HOURS=${2:-8}
MINUTES=${3:-10}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/scenarios/results/$(date +%Y%m%d_%H%M%S)_odd_supervised"
LOG=/tmp/odd_supervised.log
mkdir -p "$OUT"

cd "$ROOT" || exit 1
# ROS's setup.bash reads unbound variables, which under `set -u` exits the
# shell instantly and silently — the first attempt at this produced no
# output at all and no process. Relax it just for the sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source install/setup.bash 2>/dev/null
set -u

echo "[supervisor] target ${TARGET_KM} km, cap ${MAX_HOURS} h, out $OUT" | tee "$LOG"
START=$(date +%s)
ATTEMPT=0

carla_alive() {
  timeout 25 python3 - <<'PY' >/dev/null 2>&1
import carla
c = carla.Client('localhost', 2000); c.set_timeout(20.0)
c.get_world().get_map()
PY
}

banked_km() {
  python3 - "$OUT/legs.json" <<'PY' 2>/dev/null || echo 0
import json, sys, os
p = sys.argv[1]
print(f"{sum(r['adas_m'] for r in json.load(open(p)))/1000:.2f}" if os.path.exists(p) else "0")
PY
}

while :; do
  ELAPSED=$(( ($(date +%s) - START) / 60 ))
  KM=$(banked_km)
  if (( $(echo "$KM >= $TARGET_KM" | bc -l) )); then
    echo "[supervisor] target met: ${KM} km" | tee -a "$LOG"; break
  fi
  if (( ELAPSED > MAX_HOURS * 60 )); then
    echo "[supervisor] ${MAX_HOURS} h cap reached with ${KM} km" | tee -a "$LOG"; break
  fi

  ATTEMPT=$((ATTEMPT + 1))
  if ! carla_alive; then
    echo "[supervisor] CARLA not answering — restarting it" | tee -a "$LOG"
    pkill -f CarlaUE4-Linux-Shipping; pkill -f CarlaUE4.sh; sleep 8
    ( cd /home/sirius/CARLA_0.9.16 && setsid nohup ./CarlaUE4.sh -RenderOffScreen \
        -carla-rpc-port=2000 -quality-level=Epic >/tmp/carla_supervised.log 2>&1 & )
    for _ in $(seq 1 40); do sleep 5; carla_alive && break; done
    carla_alive || { echo "[supervisor] CARLA will not come up — stopping" | tee -a "$LOG"; break; }
    echo "[supervisor] CARLA back up" | tee -a "$LOG"
  fi

  echo "[supervisor] attempt $ATTEMPT — ${KM}/${TARGET_KM} km after ${ELAPSED} min" | tee -a "$LOG"
  python3 scenarios/odd_endurance.py \
      --target-adas-km "$TARGET_KM" --minutes-per-leg "$MINUTES" \
      --max-hours "$MAX_HOURS" --out-dir "$OUT" --resume \
      >> "$LOG" 2>&1
  RC=$?
  echo "[supervisor] campaign exited rc=$RC" | tee -a "$LOG"
  # rc 0 means it reached its own target or cap; anything else is a crash
  # and is worth retrying, because the crash is usually the simulator.
  [ "$RC" -eq 0 ] && break
  sleep 10
done

echo "[supervisor] finished: $(banked_km) km in $OUT" | tee -a "$LOG"
