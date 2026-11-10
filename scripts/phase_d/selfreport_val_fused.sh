#!/bin/bash
# Phase D self reports with the self object's position fused from the self report and
# its assigned reports, both refinements on. V2V4Real val only, both trackers.
set -e
OUT=${1:-/workspace/data/v2v4real/d1_selfrep_val_fused}
D=/workspace/data/v2v4real
HERE=$(dirname "$0")
B=$HERE/../../target/release/v2v4real-replay
BASE="--box-size-sys-m 0.50 --box-yaw-sys-deg 5.7 --box-center-sys-frac 0.14 --box-center-sys-z-m 0.22"
SEL="--fresh-only --overlap-iou 0.3 --overlap-keep recent --pub-min-score 0.3"
REF="--self-reports --self-same-source-exact --self-compete"
declare -A CFG=( [committed]="" [committed_self]="$REF" [selected]="$SEL" [selected_self]="$SEL $REF" )
mkdir -p "$OUT"
for n in "${!CFG[@]}"; do
  mkdir -p "$OUT/$n"
  for s in $(ls $D/d1_replay_in/val3); do
    $B --dir $D/d1_replay_in/val3/$s --out $OUT/$n/$s.json --dump $OUT/$n/$s.dump.jsonl --merge-diag $OUT/$n/$s.merge.jsonl $BASE ${CFG[$n]} > $OUT/$n/$s.log 2>&1 &
  done
done
wait
python3 $HERE/aggregate_replay.py $OUT/metrics.json committed=$OUT/committed committed_self=$OUT/committed_self selected=$OUT/selected selected_self=$OUT/selected_self > $OUT/metrics.txt
/workspace/tools/envs/v2v4real/bin/python -I $HERE/ap_eval.py --opencood /workspace/tools/V2V4Real --export $D/d1_detections/val.jsonl.gz --inputs $D/d1_replay_in/val3 --out $OUT/ap.json --check-library \
  --dumps committed=$OUT/committed --dumps committed_self=$OUT/committed_self --dumps selected=$OUT/selected --dumps selected_self=$OUT/selected_self 2>&1 | grep -v "Warning\|return lib" | tee $OUT/ap.txt
/workspace/tools/envs/v2v4real/bin/python -I $HERE/selfreport_coverage.py /workspace/tools/V2V4Real $D/d1_detections/val.jsonl.gz $D/d1_replay_in/val3 $OUT/coverage.json committed=$OUT/committed committed_self=$OUT/committed_self selected=$OUT/selected selected_self=$OUT/selected_self 2>&1 | grep -v "Warning\|return lib" | tee $OUT/coverage.txt
