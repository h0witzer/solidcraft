#!/bin/bash
# Usage: merge-worker.sh <worker-dir> [--apply]
# 3-way merges <worker-dir>/work/<path> into D:/Github/solidcraft-6748/solidcraft/<path> with <worker-dir>/base/<path> as ancestor.
# Line endings normalized for the merge, then written back in the tree file's style (CRLF if it had any).
W="$1"; APPLY="$2"; T="${T:-/d/Github/solidcraft-6748/solidcraft}"; mkdir -p "$W/dry"; rc=0
cd "$W/work" && find . -type f | sed 's|^\./||' > "$W/files.txt"; cd "$W"
while read f; do
  d="$W/dry/$(echo $f | tr / _)"
  if [ -f "$T/$f" ]; then tr -d '\r' < "$T/$f" > "$d.ours"; crlf=$(grep -c $'\r$' "$T/$f"); else : > "$d.ours"; crlf=0; fi
  tr -d '\r' < "base/$f" > "$d.base"; tr -d '\r' < "work/$f" > "$d.theirs"
  git merge-file -p "$d.ours" "$d.base" "$d.theirs" > "$d.merged"; c=$?
  echo "$f conflicts=$c"; [ $c -ne 0 ] && rc=1
  if [ "$APPLY" = "--apply" ] && [ $c -eq 0 ]; then
    if [ "$crlf" -gt 0 ]; then sed 's/$/\r/' "$d.merged" > "$T/$f"; else cp "$d.merged" "$T/$f"; fi
  fi
done < "$W/files.txt"
exit $rc
