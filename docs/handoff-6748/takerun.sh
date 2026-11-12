#!/bin/bash
# Takeover verification on the main checkout: the probe test (then removed), clippy, UI suites and
# scenarios (debug target), then the release CLI (target-rel), sheet-metal kernel cases and oracle.
# Usage: takerun.sh <out-dir>
. /d/Github/solidcraft-6748/env.sh
cd "$SPRINT/solidcraft"; O="$1"; mkdir -p "$O/kc"
echo "takerun start $(date '+%F %T') head $(git rev-parse --short HEAD) + $(git status --short | wc -l) changed files" > "$O/log.txt"
step() { n=$1; shift; s=$(date +%s); "$@" > "$O/$n.txt" 2>&1; echo "$n exit $? $(( $(date +%s) - s ))s $(date +%T)" >> "$O/log.txt"; }
step probe cargo test -p solidcraft-engine --lib -- zz_probe --nocapture
python -I - <<'PY'
p = "D:/Github/solidcraft-6748/solidcraft/crates/engine/src/cmd/sheet_tests.rs"
t = open(p, encoding="utf-8", newline="").read()
k = t.find("\n#[test]\nfn zz_probe_text_and_round_cut()")
if k < 0: k = t.find("\r\n#[test]\r\nfn zz_probe_text_and_round_cut()")
if k >= 0: open(p, "w", encoding="utf-8", newline="").write(t[:k] + ("\r\n" if "\r\n" in t else "\n"))
PY
echo "probe removed: $(git status --short | wc -l) changed files" >> "$O/log.txt"
step clippy cargo clippy -p solidcraft-doc -p solidcraft-engine -p solidcraft-ui-egui --all-targets
step ui cargo test -p solidcraft-ui-egui --lib -- sheet dialogs_all_tests command_help workspace i18n --skip progressive_image_controls_route_to_the_registered_host
step scen cargo test -p solidcraft-ui-egui --test scenarios sheet_metal
export CARGO_TARGET_DIR="$SPRINT/target-rel"
step rel cargo build -j 12 --release -p solidcraft-cli
CLI="$CARGO_TARGET_DIR/release/solidcraft-cli.exe"
s=$(date +%s); ls -d tests/kernel-cases/sheet-metal/* > "$O/kc/all.txt"; split -n l/8 -d "$O/kc/all.txt" "$O/kc/slice."
for f in "$O"/kc/slice.[0-9][0-9]; do "$CLI" kernel-cases --json $(cat "$f") > "$f.json" 2> "$f.err" & done; wait
python -I -c "import json,sys,glob; print(json.dumps([c for f in sorted(glob.glob(sys.argv[1]+'/kc/slice.[0-9][0-9].json')) for c in json.load(open(f))], indent=2))" "$O" > "$O/kc.json"
echo "kernel-cases $(( $(date +%s) - s ))s $(date +%T)" >> "$O/log.txt"
step oracle cargo xtask oracle --cli "$CLI"
cp docs/oracle.md "$O/oracle.md"; git status --short >> "$O/log.txt"; git checkout -- docs
echo "takerun done $(date +%T)" >> "$O/log.txt"
