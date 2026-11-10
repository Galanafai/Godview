#!/bin/bash
# Phase D self reports: V2V4Real val only, with and without self reports.
# Usage: selfreport_val.sh OUT_DIR   (sharing, alignment off, two tracker settings)
set -e
OUT=${1:-/workspace/data/v2v4real/d1_selfrep_val}
D=/workspace/data/v2v4real
B=$(dirname "$0")/../../target/release/v2v4real-replay
BASE="--box-size-sys-m 0.50 --box-yaw-sys-deg 5.7 --box-center-sys-frac 0.14 --box-center-sys-z-m 0.22"
SEL="--fresh-only --overlap-iou 0.3 --overlap-keep recent --pub-min-score 0.3"
REF="--self-same-source-exact --self-compete"
declare -A CFG=( [committed]="" [committed_self]="--self-reports" [committed_self_refined]="--self-reports $REF" \
  [selected]="$SEL" [selected_self]="$SEL --self-reports" [selected_self_same_source]="$SEL --self-reports --self-same-source-exact" \
  [selected_self_compete]="$SEL --self-reports --self-compete" [selected_self_refined]="$SEL --self-reports $REF" )
mkdir -p "$OUT"
for n in "${!CFG[@]}"; do
  mkdir -p "$OUT/$n"
  for s in $(ls $D/d1_replay_in/val3); do
    $B --dir $D/d1_replay_in/val3/$s --out $OUT/$n/$s.json --dump $OUT/$n/$s.dump.jsonl --merge-diag $OUT/$n/$s.merge.jsonl $BASE ${CFG[$n]} > $OUT/$n/$s.log 2>&1 &
  done
done
wait
python3 "$(dirname "$0")/aggregate_replay.py" $OUT/metrics.json $(for n in committed committed_self committed_self_refined selected selected_self selected_self_same_source selected_self_compete selected_self_refined; do printf '%s=%s/%s ' $n $OUT $n; done) > $OUT/metrics.txt
/workspace/tools/envs/v2v4real/bin/python -I "$(dirname "$0")/ap_eval.py" --opencood /workspace/tools/V2V4Real --export $D/d1_detections/val.jsonl.gz --inputs $D/d1_replay_in/val3 --out $OUT/ap.json --baselines 0.2 --check-library \
  $(for n in committed committed_self committed_self_refined selected selected_self selected_self_same_source selected_self_compete selected_self_refined; do printf -- '--dumps %s=%s/%s ' $n $OUT $n; done) 2>&1 | grep -v "Warning\|return lib" | tee $OUT/ap.txt
