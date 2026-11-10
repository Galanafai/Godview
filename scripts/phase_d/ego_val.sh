#!/bin/bash
# Phase D: ego view and offset filter on V2V4Real val only. Selected tracker, sharing.
# Variants: self reports off/on x alignment off / per-frame / filtered, all on ego views,
# each with the shared world dumped for reference.
set -e
OUT=${1:-/workspace/data/v2v4real/d1_ego_val}
D=/workspace/data/v2v4real
HERE=$(dirname "$0")
B=$HERE/../../target/release/v2v4real-replay
BASE="--box-size-sys-m 0.50 --box-yaw-sys-deg 5.7 --box-center-sys-frac 0.14 --box-center-sys-z-m 0.22"
SEL="--fresh-only --overlap-iou 0.3 --overlap-keep recent --pub-min-score 0.3"
SELF="--self-reports --self-same-source-exact --self-compete --self-size 4.73,2.25,2.34 --self-offset 0.10,0.17"
declare -A CFG=(
  [noself_alignoff]="$SEL --ego-view" [noself_perframe]="$SEL --ego-view --align" [noself_filter]="$SEL --ego-view --offset-filter"
  [self_alignoff]="$SEL --ego-view $SELF" [self_perframe]="$SEL --ego-view $SELF --align" [self_filter]="$SEL --ego-view $SELF --offset-filter" )
NAMES="noself_alignoff noself_perframe noself_filter self_alignoff self_perframe self_filter"
mkdir -p "$OUT"
for n in $NAMES; do
  mkdir -p "$OUT/$n"
  for s in $(ls $D/d1_replay_in/val3); do
    $B --dir $D/d1_replay_in/val3/$s --out $OUT/$n/$s.json --dump $OUT/$n/$s.dump.jsonl --merge-diag $OUT/$n/$s.merge.jsonl $BASE ${CFG[$n]} > $OUT/$n/$s.log 2>&1 &
  done
done
wait
python3 $HERE/aggregate_replay.py $OUT/metrics.json $(for n in $NAMES; do printf '%s=%s/%s ' $n $OUT $n; done) > $OUT/metrics.txt
ARGS=""; SH=""; COV=""
for n in $NAMES; do ARGS="$ARGS --dumps $n=$OUT/$n --dumps ${n}_shared=$OUT/$n"; SH="$SH --shared ${n}_shared"; COV="$COV $n=$OUT/$n $n@shared=$OUT/$n"; done
/workspace/tools/envs/v2v4real/bin/python -I $HERE/ap_eval.py --opencood /workspace/tools/V2V4Real --export $D/d1_detections/val.jsonl.gz --inputs $D/d1_replay_in/val3 --out $OUT/ap.json --check-library $ARGS $SH 2>&1 | grep -v "Warning\|return lib" | tee $OUT/ap.txt
/workspace/tools/envs/v2v4real/bin/python -I $HERE/selfreport_coverage.py /workspace/tools/V2V4Real $D/d1_detections/val.jsonl.gz $D/d1_replay_in/val3 $OUT/coverage.json $COV 2>&1 | grep -v "Warning\|return lib" | tee $OUT/coverage.txt
