"""Picks the wake-word threshold from recordings (PLAN.md section 13.11). No ROS needed.

    python3 -m nontok_voice.eval_wake --model models/nontok.onnx \\
        --positives recordings/nontok/ --background recordings/lab_noise/

positives:  short 16 kHz mono WAV files, each with one person saying "Nontok".
background: long 16 kHz mono WAV files of normal lab noise and talk, with nobody saying it.
Prints, for a range of thresholds, the share of calls detected and the false triggers per
hour. Choose the lowest threshold with fewer than one false trigger per hour.
"""
import argparse
import glob
import os
import wave

import numpy as np

from .wake_logic import WakeTrigger, load_model

BLOCK, SAMPLE_RATE = 1280, 16000


def read_wav(path: str) -> np.ndarray:
    with wave.open(path, 'rb') as handle:
        if handle.getframerate() != SAMPLE_RATE or handle.getnchannels() != 1 or handle.getsampwidth() != 2:
            raise SystemExit('%s must be 16 kHz, mono, 16-bit' % path)
        return np.frombuffer(handle.readframes(handle.getnframes()), dtype=np.int16)


def scores_for(model, samples: np.ndarray):
    model.reset()
    return [float(max(model.predict(samples[i:i + BLOCK]).values()))
            for i in range(0, len(samples) - BLOCK + 1, BLOCK)]


def count_triggers(scores, threshold: float) -> int:
    trigger = WakeTrigger(threshold)
    return sum(trigger.update(score, i * BLOCK / SAMPLE_RATE) for i, score in enumerate(scores))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--model', required=True)
    parser.add_argument('--positives', required=True)
    parser.add_argument('--background', required=True)
    args = parser.parse_args(argv)
    model = load_model(args.model)

    positives = [scores_for(model, read_wav(p)) for p in sorted(glob.glob(os.path.join(args.positives, '*.wav')))]
    background, hours = [], 0.0
    for path in sorted(glob.glob(os.path.join(args.background, '*.wav'))):
        samples = read_wav(path)
        hours += len(samples) / SAMPLE_RATE / 3600.0
        background.append(scores_for(model, samples))
    if not positives or not background:
        raise SystemExit('Need at least one positive and one background recording')

    print('%d calls, %.2f h of background' % (len(positives), hours))
    print('threshold  detected  false/hour')
    for threshold in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
        detected = sum(count_triggers(s, threshold) > 0 for s in positives) / len(positives)
        false_alarms = sum(count_triggers(s, threshold) for s in background) / hours
        print('   %.1f      %5.0f %%   %8.1f' % (threshold, detected * 100, false_alarms))


if __name__ == '__main__':
    main()
