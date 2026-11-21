#!/usr/bin/env python3
"""Step 2 of 3: clips -> augmented 2 s windows -> openWakeWord features (N, 16, 96).

Each clip is placed in a 2 s window with reverberation, level changes and background sound.
Put recordings of the real room in ambient/*.wav (16 kHz mono; record on the robot with
`arecord -D plughw:<card>,0 -f S16_LE -r 16000 -c 1 -d 300 ambient.wav`); they are used as
background and as negatives. Files named heldout* are left for testing.

    python3 feats.py                # about 3 minutes; writes feat_*.npy
"""
import glob, os, random, sys
import numpy as np, soundfile as sf
from scipy.signal import fftconvolve
from openwakeword.utils import AudioFeatures

rng = np.random.default_rng(0); random.seed(0)
N = 32000   # 2 s at 16 kHz -> 16 embedding frames

def load(path):
    a, r = sf.read(path, dtype='float32'); assert r == 16000, path
    return a if a.ndim == 1 else a[:, 0]

def trim(a, top=0.02):
    idx = np.where(np.abs(a) > top * np.abs(a).max())[0]
    return a[max(idx[0] - 400, 0): idx[-1] + 800] if len(idx) else a

def rir():
    n = int(rng.uniform(0.05, 0.35) * 16000)
    h = rng.standard_normal(n) * np.exp(-np.arange(n) / (n / rng.uniform(4, 8)))
    h[0] = 1.0 / rng.uniform(0.3, 1.0)       # direct sound against the reverberant tail
    return (h / np.abs(h).sum()).astype(np.float32)

def noise_clip(noises, sentences):
    kind = rng.random()
    if kind < 0.45 and noises:
        src = random.choice(noises); s = rng.integers(0, max(1, len(src) - N)); out = src[s:s + N]
        out = np.pad(out, (0, N - len(out)))
    elif kind < 0.75:                         # babble: other people talking
        out = np.zeros(N, np.float32)
        for _ in range(rng.integers(1, 4)):
            s = random.choice(sentences); o = rng.integers(0, N)
            out[o:o + len(s)][:max(0, N - o)] += s[:N - o] * rng.uniform(0.3, 1.0)
    else:                                     # hiss / hum
        white = rng.standard_normal(N).astype(np.float32)
        out = np.cumsum(white) if rng.random() < 0.5 else white
        out = out - out.mean()
    return out.astype(np.float32)

def window(word, noises, sentences, end_aligned=True):
    """One 2 s training window with the clip placed in it."""
    w = trim(word)
    if rng.random() < 0.6:
        w = fftconvolve(w, rir())[:len(w) + 2000]
    w = w[:N - 1600] if len(w) > N - 1600 else w
    out = np.zeros(N, np.float32)
    if end_aligned:   # the word ends 0-0.3 s before the end of the window, as when it has just been said
        end = N - int(rng.uniform(0.0, 0.30) * 16000)
    else:
        end = int(rng.uniform(len(w), N))
    start = max(0, end - len(w)); out[start:start + len(w)] = w[:N - start]
    level = np.sqrt((w ** 2).mean()) + 1e-9
    if rng.random() < 0.85:
        n = noise_clip(noises, sentences)
        snr = rng.uniform(0, 25)
        out += n * level / (np.sqrt((n ** 2).mean()) + 1e-9) / (10 ** (snr / 20))
    out *= rng.uniform(0.05, 0.9) / (np.abs(out).max() + 1e-9)
    return (out * 32767).astype(np.int16)

def build(paths, copies, noises, sentences, end_aligned=True):
    return np.stack([window(load(p), noises, sentences, end_aligned) for p in paths for _ in range(copies)])

if __name__ == '__main__':
    F = AudioFeatures(inference_framework='onnx')
    noises = [load(p) for p in glob.glob('ambient/*.wav') if 'heldout' not in p]
    noises = [n for n in noises if np.abs(n).max() > 0.001]
    sent_paths = sorted(glob.glob('data/sent/*.wav'))
    sentences = [load(p) * 0.5 for p in sent_paths[:400]]
    sets = {'pos': (sorted(glob.glob('data/pos/*.wav')), 4, True), 'pos_say': (sorted(glob.glob('data/pos_say/*.wav')), 6, True),
            'near': (sorted(glob.glob('data/near/*.wav')), 4, True), 'sent': (sent_paths[400:], 3, False)}
    for name, (paths, copies, aligned) in sets.items():
        clips = build(paths, copies, noises, sentences, aligned)
        feats = F.embed_clips(clips, batch_size=256, ncpu=6)
        np.save('feat_%s.npy' % name, feats.astype(np.float32)); print(name, feats.shape, flush=True)
    # Real lab ambience as negatives, cut into overlapping 2 s windows.
    amb = np.concatenate(noises); step = 4000
    clips = np.stack([(amb[i:i + N] * 32767).astype(np.int16) for i in range(0, len(amb) - N, step)])
    feats = F.embed_clips(clips, batch_size=256, ncpu=6); np.save('feat_ambient.npy', feats.astype(np.float32)); print('ambient', feats.shape)
