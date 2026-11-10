"""Clean a voiceover and transcribe it into beats with word timings.

    python3 voice.py --audio take.m4a --out voice/ [--beats 6] [--model small]
        [--fix "card=car" --fix "Godview=GodView"] [--from-json voice/voice.json]

Writes voice/clean.wav (48 kHz mono: high-pass, gentle denoise, loudness
normalised to -16 LUFS) and voice/voice.json:

    {"duration": s, "beats": [{"start": s, "end": s, "text": "...",
                               "words": [{"start": s, "end": s, "word": "..."}]}]}

Beats are split at the longest silences measured in the audio itself (word
timings from speech recognition are too imprecise for this), and each word goes
to the beat its midpoint falls in. --fix replaces a transcribed word, so the
captions match what was said; --from-json re-splits and re-fixes an existing
transcript without transcribing again. Needs ffmpeg and
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


def silences(wav, noise_db=-35, min_s=0.5):
    """(start, end) of silences in the audio, measured with ffmpeg silencedetect."""
    r = subprocess.run(["ffmpeg", "-nostats", "-i", str(wav), "-af",
                        f"silencedetect=noise={noise_db}dB:d={min_s}", "-f", "null", "-"],
                       capture_output=True, text=True)
    out, start = [], None
    for line in r.stderr.splitlines():
        if "silence_start:" in line:
            start = float(line.split("silence_start:")[1].split()[0])
        elif "silence_end:" in line and start is not None:
            out.append((start, float(line.split("silence_end:")[1].split()[0])))
            start = None
    return out


def split_by_silence(words, wav, n):
    """Split at the n-1 longest silences that fall between words."""
    first, last = words[0]["start"], words[-1]["end"]
    gaps = [g for g in silences(wav) if g[0] > first + 0.2 and g[1] < last - 0.2]
    gaps = sorted(sorted(gaps, key=lambda g: g[1] - g[0], reverse=True)[:n - 1])
    if len(gaps) < n - 1:
        return None
    bounds = [(g[0] + g[1]) / 2 for g in gaps]
    groups = [[] for _ in range(n)]
    for w in words:
        mid = (w["start"] + w["end"]) / 2
        groups[sum(mid > b for b in bounds)].append(w)
    if any(not g for g in groups):
        return None
    beats = []
    for k, g in enumerate(groups):
        a0 = gaps[k - 1][1] - 0.08 if k > 0 else max(0.0, g[0]["start"] - 0.15)
        a1 = gaps[k][0] + 0.15 if k < n - 1 else g[-1]["end"] + 0.35
        beats.append({"start": g[0]["start"], "end": g[-1]["end"],
                      "audio_start": round(a0, 3), "audio_end": round(a1, 3),
                      "text": " ".join(w["word"] for w in g), "words": g})
    return beats


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
    p.add_argument("--fix", action="append", default=[], help="wrong=right, applied to each word")
    p.add_argument("--from-json", default="", help="reuse this transcript instead of transcribing")
    a = p.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    wav = out / "clean.wav"
    if not wav.exists() or not a.from_json:
        clean(a.audio, wav)
    if a.from_json:
        old = json.loads(Path(a.from_json).read_text())
        words = [w for b in old["beats"] for w in b["words"]]
        duration = old["duration"]
    else:
        words, duration = transcribe(wav, a.model)
    fixes = dict(f.split("=", 1) for f in a.fix)
    for w in words:
        core = w["word"].rstrip(".,?!;:")
        tail = w["word"][len(core):]
        if core in fixes:
            w["word"] = fixes[core] + tail
    if len(words) < a.beats:
        raise SystemExit(f"only {len(words)} words transcribed")
    beats = split_by_silence(words, wav, a.beats)
    how = "measured silences"
    if beats is None:
        beats = split_beats(words, a.beats)
        how = "word-timing gaps (silence split failed)"
    (out / "voice.json").write_text(json.dumps({"duration": duration, "beats": beats}, indent=1))
    print(f"split at {how}")
    for i, b in enumerate(beats, 1):
        print(f"beat {i}: {b['start']:6.2f} to {b['end']:6.2f} s  {b['text']}")
    gaps = [beats[i]["start"] - beats[i - 1]["end"] for i in range(1, len(beats))]
    print("pauses between beats (s):", ", ".join(f"{g:.2f}" for g in gaps))


if __name__ == "__main__":
    main()
