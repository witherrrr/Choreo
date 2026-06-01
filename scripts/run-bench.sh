#!/usr/bin/env bash
# Run choreo-cli generation for one (or every) project variant.
#
# Usage: run-bench.sh <cli-binary> <test-projects-dir> <output-dir> [variant] [runs]
#
# If [variant] is given, only that one project subdirectory is benched (used by
# the CI per-variant matrix so each job runs a single variant concurrently).
# Omit it to bench every variant (local / single-job use). [runs] defaults to 5.
#
# Each trajectory is solved by its OWN `choreo-cli --trajectory` process and all
# of a variant's trajectories for a given run are launched concurrently. The
# single-element reports are then merged back into the `<variant>.run<N>.report.json`
# array bench-report.mjs expects.
#
# Set BENCH_TRAJ_JOBS=N to cap concurrency (0 / unset = no cap, i.e. all of
# the variant's trajectories at once).

set -euo pipefail

# C locale: deterministic `%U`/`%S` decimal point for the timing parse below.
export LC_ALL=C
# `time` emits only "<user_s> <sys_s>" (no real, no labels) to its stderr.
TIMEFORMAT='%U %S'

CLI=${1:?missing CLI binary path}
PROJECTS=${2:?missing test-projects dir}
OUT=${3:?missing output dir}
ONLY_VARIANT=${4:-}
RUNS=${5:-5}
TRAJ_JOBS=${BENCH_TRAJ_JOBS:-4}   # GHA runners are 4-core
TRAJ_TIMEOUT_S=${BENCH_TRAJ_TIMEOUT:-300}
TRAJ_KILL_GRACE_S=${BENCH_TRAJ_KILL_GRACE:-10}

mkdir -p "$OUT"

# Per-trajectory shards live outside $OUT so they neither bloat the uploaded
# artifact nor get picked up as extra "runs" by bench-report.mjs's loadRuns.
SHARD_ROOT=$(mktemp -d)
trap 'rm -rf "$SHARD_ROOT"' EXIT

shopt -s nullglob
if [ -n "$ONLY_VARIANT" ]; then
  project_dirs=("$PROJECTS/$ONLY_VARIANT/")
  if [ ! -d "${project_dirs[0]}" ]; then
    echo "variant '$ONLY_VARIANT' not found under $PROJECTS" >&2
    exit 1
  fi
else
  project_dirs=("$PROJECTS"/*/)
fi
if [ "${#project_dirs[@]}" -eq 0 ]; then
  echo "no project subdirectories found under $PROJECTS" >&2
  exit 1
fi

# Merge the single-element shard reports in $1 into the combined run report $2
# (the array shape bench-report.mjs's loadRuns/aggregateSide consume), while
# replacing each trajectory's solve_ms with the externally measured process CPU
# time from its sibling `<traj>.cpu` file (user+sys seconds → ms). Each shard
# is exactly one trajectory's invocation, so its whole-process CPU maps 1:1 to
# that one entry. If a `.cpu` is missing/garbled the CLI's own wall solve_ms is
# kept as a fallback (logged). An empty shard set yields `[]`, which the report
# treats as "this run produced nothing".
merge_shards() {
  local shard_dir=$1 out_file=$2
  node -e '
    const fs = require("fs");
    const path = require("path");
    const [out, dir] = process.argv.slice(1);
    const merged = [];
    for (const f of fs.readdirSync(dir).sort()) {
      if (!f.endsWith(".report.json")) continue;
      const base = f.slice(0, -".report.json".length);
      let arr;
      try {
        arr = JSON.parse(fs.readFileSync(path.join(dir, f), "utf8"));
      } catch (e) {
        console.error(`merge: skipping ${f}: ${e.message}`);
        continue;
      }
      if (!Array.isArray(arr)) continue;
      let cpuMs = null;
      try {
        const t = fs.readFileSync(path.join(dir, base + ".cpu"), "utf8")
          .trim().split(/\s+/).map(Number);
        if (t.length >= 2 && t.every(Number.isFinite)) cpuMs = (t[0] + t[1]) * 1000;
      } catch { /* fall through to wall fallback */ }
      if (cpuMs == null) console.error(`merge: no CPU time for ${base}, keeping CLI wall solve_ms`);
      for (const e of arr) {
        if (cpuMs != null) e.solve_ms = cpuMs;
        merged.push(e);
      }
    }
    fs.writeFileSync(out, JSON.stringify(merged, null, 2));
  ' "$out_file" "$shard_dir"
}

# Run one trajectory's CLI invocation, optionally under a wall-clock timeout,
# capturing its CPU time to <traj>.cpu. The CLI writes its report shard itself;
# we only step in when `timeout` kills a stalled solve. Because the killed
# process never wrote a shard, that trajectory would otherwise silently vanish
# from the report (merge_shards/bench-report only know trajectories that
# produced a shard), so we synthesize a minimal ok:false shard. We keep it
# deliberately thin — `name` + `ok:false` + `error`, no `solve_ms` — both to
# minimize how much of the CLI's report schema this script duplicates and
# because bench-report drops failed entries from the solve-time stats anyway.
# `timeout` exits 124 when it killed the command on the deadline, 137 when the
# SIGKILL backstop was needed after the grace period.
bench_one() {
  local name=$1 tn=$2 run=$3 rs=$4
  local report="$rs/$tn.report.json" cpu="$rs/$tn.cpu"
  local -a cmd=("$CLI" --chor "$chor" --trajectory "$tn" --generate --report-json "$report")
  if [ "$TRAJ_TIMEOUT_S" -gt 0 ]; then
    cmd=(timeout -k "${TRAJ_KILL_GRACE_S}s" "${TRAJ_TIMEOUT_S}s" "${cmd[@]}")
  fi
  local rc=0
  # fd3 = this shell's stderr (CLI logs flow there as before); the brace
  # group's own stderr — where the `time` keyword writes "%U %S" — goes to
  # <traj>.cpu, isolating the timing from choreo-cli's ~MBs of solver output.
  { time "${cmd[@]}" 2>&3 ; } 3>&2 2>"$cpu" || rc=$?
  case "$rc" in
    0) ;;
    124|137)
      printf '[{"name":"%s","ok":false,"error":"timed out after %ss"}]\n' \
        "$tn" "$TRAJ_TIMEOUT_S" > "$report"
      echo "choreo-cli TIMED OUT after ${TRAJ_TIMEOUT_S}s on $name/$tn run $run/$RUNS" >&2
      ;;
    *)
      echo "choreo-cli exited nonzero ($rc) on $name/$tn run $run/$RUNS" >&2
      ;;
  esac
}

for proj_dir in "${project_dirs[@]}"; do
  name=$(basename "$proj_dir")
  chor="$proj_dir/project.chor"
  if [ ! -f "$chor" ]; then
    echo "skipping $name (no project.chor)" >&2
    continue
  fi
  traj_files=("$proj_dir"/*.traj)
  if [ "${#traj_files[@]}" -eq 0 ]; then
    echo "skipping $name (no .traj files)" >&2
    continue
  fi
  traj_names=()
  for tf in "${traj_files[@]}"; do
    traj_names+=("$(basename "$tf" .traj)")
  done

  echo "::group::$name"
  echo "--- $name: $(( ${#traj_names[@]} * RUNS )) jobs (${#traj_names[@]} trajectories × $RUNS runs) fully parallel, concurrency cap $TRAJ_JOBS (0 = unlimited) ---"
  for run in $(seq 1 "$RUNS"); do
    mkdir -p "$SHARD_ROOT/$name.run$run"
  done
  # Run all trajectories for this variant in parallel across $RUNS runs
  # with concurrency cap $TRAJ_JOBS.
  for run in $(seq 1 "$RUNS"); do
    rs="$SHARD_ROOT/$name.run$run"
    for tn in "${traj_names[@]}"; do
      if [ "$TRAJ_JOBS" -gt 0 ]; then
        while [ "$(jobs -rp | wc -l)" -ge "$TRAJ_JOBS" ]; do wait -n || true; done
      fi
      bench_one "$name" "$tn" "$run" "$rs" &
    done
  done
  wait
  for run in $(seq 1 "$RUNS"); do
    merge_shards "$SHARD_ROOT/$name.run$run" "$OUT/$name.run$run.report.json"
  done
  echo "::endgroup::"
done
