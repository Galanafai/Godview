# V2V4Real replay scripts

Phase C tooling. Brief: `docs/PHASE_C_HANDOFF.md`; results:
`docs/results/v2v4real.md`. Data lives under `$V2V4REAL_DIR`, never in the
repo. Run Python that reads the dataset with `python -I` (the files are
untrusted; `v2v4real_io.py` decodes the YAML numpy tags without running
Python constructors).

* `v2v4real_io.py`: safe YAML and ASCII PCD readers.
* `gps_fit.py`: anchors a scenario's map frame on the Earth from the gps field.
* `convert.py`: one scenario to `godview_node` JSON lines per car plus `truth.json`
  (version 1 detections: noisy labels of boxes the car's own lidar hits).
* `run_all.py`: converts and replays every declaration and writes the summary.
* `verify/`: the format checks behind the Findings.
* `test_v2v4real.py`: unit tests (no dataset needed).

The replay itself is `godview_sim/src/bin/v2v4real_replay.rs`
(`cargo build --release -p godview_sim --bin v2v4real-replay`).
