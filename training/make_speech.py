#!/usr/bin/env python3
"""Records the robot's phrases as WAV files, with the Piper text-to-speech voice.

Run on a PC after changing phrases.yaml or the item list (pip install piper-tts pyyaml huggingface_hub):

    python3 training/make_speech.py                 # all phrases
    python3 training/make_speech.py --voice en_GB-alan-medium

Writes ros2_ws/src/nontok_voice/sounds/<name>.wav; the robot only plays these files.
"""
import argparse
import os
import sys
import tempfile
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'ros2_ws', 'src')
sys.path.insert(0, os.path.join(SRC, 'nontok_voice'))
sys.path.insert(0, os.path.join(SRC, 'nontok_perception'))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--voice', default='en_US-lessac-medium')
    parser.add_argument('--out', default=os.path.join(SRC, 'nontok_voice', 'sounds'))
    parser.add_argument('--cache', default=os.path.join(HERE, 'wake', 'voices'))
    args = parser.parse_args()
    import piper
    import yaml
    from huggingface_hub import hf_hub_download
    from piper import PiperVoice

    from nontok_voice.speech_logic import all_phrases

    with open(os.path.join(SRC, 'nontok_perception', 'config', 'items.yaml')) as handle:
        items = {str(k): str(v) for k, v in (yaml.safe_load(handle).get('items') or {}).items()}
    phrases = all_phrases(os.path.join(SRC, 'nontok_voice', 'config', 'phrases.yaml'), items)

    language, name, quality = args.voice.split('-')
    remote = '%s/%s/%s/%s/%s.onnx' % (language.split('_')[0], language, name, quality, args.voice)
    for part in (remote, remote + '.json'):
        hf_hub_download('rhasspy/piper-voices', part, local_dir=args.cache)
    # espeak cannot open its data through a long path, so reach it through a short link.
    link = os.path.join(tempfile.gettempdir(), 'piper-espeak-data')
    if not os.path.exists(link):
        os.symlink(os.path.join(os.path.dirname(piper.__file__), 'espeak-ng-data'), link)
    voice = PiperVoice.load(os.path.join(args.cache, remote), espeak_data_dir=link)

    os.makedirs(args.out, exist_ok=True)
    for old in os.listdir(args.out):
        if old.endswith('.wav'):
            os.remove(os.path.join(args.out, old))
    for sound, text in sorted(phrases.items()):
        with wave.open(os.path.join(args.out, sound + '.wav'), 'wb') as handle:
            voice.synthesize_wav(text, handle)
        print('%-28s %s' % (sound, text))
    print('%d phrases in %s' % (len(phrases), args.out))


if __name__ == '__main__':
    main()
