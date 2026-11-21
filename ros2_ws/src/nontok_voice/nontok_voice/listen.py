"""Listens on the microphone and shows what the wake-word model hears. No ROS needed.

    python3 -m nontok_voice.listen --model models/nontok.onnx --device XWF --seconds 60
    ... --save heard/        also keeps each 2 s of sound that triggered, and --save-all
                             keeps everything, as 16 kHz WAV files for eval_wake and training

Prints one line per second: the loudness, the highest score, and TRIGGER when the robot
would have woken. Use it to try the word at different distances and to pick the threshold.
"""
import argparse
import os
import queue
import time
import wave

import numpy as np

from .wake_logic import WakeTrigger, find_microphone, load_model

BLOCK, SAMPLE_RATE = 1280, 16000


def write_wav(path: str, samples: np.ndarray) -> None:
    with wave.open(path, 'wb') as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(SAMPLE_RATE)
        handle.writeframes(samples.astype(np.int16).tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', required=True)
    parser.add_argument('--device', default='')
    parser.add_argument('--threshold', type=float, default=0.5)
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--save', default='')
    parser.add_argument('--save-all', action='store_true')
    args = parser.parse_args()
    import sounddevice

    model = load_model(args.model)
    trigger = WakeTrigger(args.threshold)
    audio = queue.Queue()
    stream = sounddevice.InputStream(
        samplerate=SAMPLE_RATE, channels=1, dtype='int16', blocksize=BLOCK,
        device=find_microphone(sounddevice.query_devices(), args.device),
        callback=lambda data, frames, t, status: audio.put(np.array(data[:, 0], dtype=np.int16)))
    if args.save:
        os.makedirs(args.save, exist_ok=True)
    history, everything, second = [], [], []
    started = time.monotonic()
    triggers = 0
    with stream:
        while time.monotonic() - started < args.seconds:
            try:
                block = audio.get(timeout=1.0)
            except queue.Empty:
                print('no sound from the microphone')
                continue
            history = (history + [block])[-25:]            # the last 2 s
            if args.save_all:
                everything.append(block)
            score = float(max(model.predict(block).values()))
            fired = trigger.update(score, time.monotonic())
            second.append((score, float(np.sqrt(np.mean(block.astype(np.float64) ** 2))), fired))
            if fired:
                triggers += 1
                if args.save:
                    write_wav(os.path.join(args.save, 'trigger_%s_%02d.wav' % (time.strftime('%H%M%S'), triggers)),
                              np.concatenate(history))
            if len(second) >= 12:                          # about one second
                print('%5.1f s  loudness %5d  best score %.2f  %s' % (
                    time.monotonic() - started, max(s[1] for s in second), max(s[0] for s in second),
                    'TRIGGER' if any(s[2] for s in second) else ''), flush=True)
                second = []
    if args.save_all and args.save and everything:
        write_wav(os.path.join(args.save, 'all_%s.wav' % time.strftime('%H%M%S')), np.concatenate(everything))
    print('%d trigger(s) in %.0f s at threshold %.2f' % (triggers, args.seconds, args.threshold))


if __name__ == '__main__':
    main()
