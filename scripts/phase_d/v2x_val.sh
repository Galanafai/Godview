#!/bin/bash
# Phase D2 step 3: the six committed variants on V2X-Real val (labels plus noise).
# Self reports off/on x alignment off / filtered / filtered with the roadside units as anchors.
# Settings as committed in docs/PHASE_D_HANDOFF.md ("D2 steps 2 and 3"); nothing chosen from results.
set -e
OUT=${1:-/workspace/data/v2xreal/d2_val_runs}
D=/workspace/data/v2xreal
HERE=$(dirname "$0")
B=$HERE/../../target/release/v2x-replay
SEL="--fresh-only --overlap-iou 0.3 --overlap-keep recent --pub-min-score 0.3"
SELF="--self-reports --self-same-source-exact --self-compete"
declare -A CFG=( [noself_alignoff]="$SEL" [noself_filter]="$SEL --offset-filter" [noself_anchor]="$SEL --offset-filter --anchor-rsu"
                 [self_alignoff]="$SEL $SELF" [self_filter]="$SEL $SELF --offset-filter" [self_anchor]="$SEL $SELF --offset-filter --anchor-rsu" )
NAMES="noself_alignoff noself_filter noself_anchor self_alignoff self_filter self_anchor"
mkdir -p "$OUT"
for n in $NAMES; do
  mkdir -p "$OUT/$n"
  for s in $(ls $D/d2_in/val); do
    $B --dir $D/d2_in/val/$s --out $OUT/$n/$s.dump.jsonl ${CFG[$n]} > $OUT/$n/$s.log 2>&1 &
  done
done
wait
/workspace/tools/envs/v2v4real/bin/python -I $HERE/v2x_eval.py $D/d2_in/val $OUT/eval.json $(for n in $NAMES; do printf '%s=%s/%s ' $n $OUT $n; done) 2>&1 | grep -v "Warning\|return lib" | tee $OUT/eval.txt
