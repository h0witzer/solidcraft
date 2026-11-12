#!/bin/bash
# The main-merge tree in wt-release (target-rel): engine and doc libraries. Usage: mergerun.sh <out-dir>
. /d/Github/solidcraft-6748/env.sh
export CARGO_TARGET_DIR="$SPRINT/target-rel"
cd "$SPRINT/wt-release"; O="$1"; mkdir -p "$O"
echo "mergerun start $(date '+%F %T') head $(git rev-parse --short HEAD) + $(git status --short | wc -l) changed files" > "$O/log.txt"
step() { n=$1; shift; s=$(date +%s); "$@" > "$O/$n.txt" 2>&1; echo "$n exit $? $(( $(date +%s) - s ))s $(date +%T)" >> "$O/log.txt"; }
step engine cargo test -p solidcraft-engine --lib
step doc cargo test -p solidcraft-doc --lib
echo "mergerun done $(date +%T)" >> "$O/log.txt"
