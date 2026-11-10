"""Clean a voiceover and transcribe it into beats with word timings.

    python3 voice.py --audio take.m4a --out voice/ [--beats 6] [--model small]

Writes voice/clean.wav (48 kHz mono: high-pass, gentle denoise, loudness
normalised to -16 LUFS) and voice/voice.json:

    {"duration": s, "beats": [{"start": s, "end": s, "text": "...",
                               "words": [{"start": s, "end": s, "word": "..."}]}]}

Beats are split at the longest pauses between words, so a take recorded with a
pause of a second or two between each part splits cleanly. Needs ffmpeg and
faster-whisper (`pip install faster-whisper`; the model downloads on first use).
Timings are relative to the start of clean.wav.
"""

import argparse
import json
import subprocess
from pathlib import Path


def clean(src, dst):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(src), "-ac", "1", "-ar", "48000",
                    "-af", "highpass=f=80,afftdn=nf=-25,"
                           "silenceremove=start_periods=1:start_threshold=-45dB:start_silence=0.15,"
                           "loudnorm=I=-16:TP=-1.5:LRA=7",
                    str(dst)], check=True)


def transcribe(wav, model_name):
    from faster_whisper import WhisperModel
    try:
        model = WhisperModel(model_name, device="cuda", compute_type="float16")
    except Exception:  # no GPU or no CUDA build: CPU is fine for a 30 s take
        model = WhisperModel(model_name, device="cpu", compute_type="int8")
    segments, info = model.transcribe(str(wav), language="en", word_timestamps=True,
                                      vad_filter=False, beam_size=5)
    words = []
    for seg in segments:
        for w in seg.words:
            words.append({"start": round(w.start, 3), "end": round(w.end, 3), "word": w.word.strip()})
    return words, info.duration


def split_beats(words, n):
    """Split at the n-1 longest gaps between consecutive words."""
    gaps = sorted(range(1, len(words)), key=lambda i: words[i]["start"] - words[i - 1]["end"],
                  reverse=True)[:n - 1]
    cuts = sorted(gaps)
    beats, start = [], 0
    for c in cuts + [len(words)]:
        ws = words[start:c]
        beats.append({"start": ws[0]["start"], "end": ws[-1]["end"],
                      "text": " ".join(w["word"] for w in ws), "words": ws})
        start = c
    return beats


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--audio", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--beats", type=int, default=6)
    p.add_argument("--model", default="small")
    a = p.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    wav = out / "clean.wav"
    clean(a.audio, wav)
    words, duration = transcribe(wav, a.model)
    if len(words) < a.beats:
        raise SystemExit(f"only {len(words)} words transcribed")
    beats = split_beats(words, a.beats)
    (out / "voice.json").write_text(json.dumps({"duration": duration, "beats": beats}, indent=1))
    for i, b in enumerate(beats, 1):
        print(f"beat {i}: {b['start']:6.2f} to {b['end']:6.2f} s  {b['text']}")
    gaps = [beats[i]["start"] - beats[i - 1]["end"] for i in range(1, len(beats))]
    print("pauses between beats (s):", ", ".join(f"{g:.2f}" for g in gaps))
    if min(gaps) < 0.6:
        print("warning: a pause between beats is under 0.6 s; check the split above")


if __name__ == "__main__":
    main()
