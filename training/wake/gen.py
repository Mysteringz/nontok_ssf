#!/usr/bin/env python3
"""Step 1 of 3: synthetic speech for the wake-word model.

"Nontok" in about 900 voices (Piper, libritts_r) and in the macOS voices, plus near-miss
phrases and ordinary sentences as negatives. Run on a Mac, in training/wake/:

    pip install piper-tts soundfile scipy "numpy<2" huggingface_hub openwakeword onnxruntime torch
    python3 gen.py 5000 8000        # positives, negatives; about 5 minutes

Without macOS the `say` voices are skipped. Output: data/{pos,pos_say,near,sent}/*.wav
"""
import os, random, shutil, subprocess, sys
import numpy as np, soundfile as sf
from scipy.signal import resample_poly
from piper import PiperVoice, SynthesisConfig

random.seed(0)
POS = ['Nontok', 'Nontok!', 'non tok', 'nontock', 'non tock', 'Non-tok', 'nohn tok', 'nawn tock', 'Nontok.', 'nontok?']
NEAR = ['non stop', 'not talk', "don't talk", 'long talk', 'tick tock', 'non toxic', 'nonsense', 'monk', 'Bangkok',
        'knock knock', 'on top', 'none taken', 'no talk', 'talk', 'tock', 'non', 'nine o clock', 'contact', 'laptop',
        'not ok', 'Hong Kong', 'non stick', 'next up', 'Nantucket', 'noon talk', 'on dock', 'unlock', 'nun', 'known',
        'montage', 'pontoon', 'non profit', 'not now', 'no thanks', 'gone to', 'want to', 'non fiction', 'non toxic glue',
        'bon voyage', 'tok', 'none', 'top', 'stock', 'nonstop talking', 'dont knock', 'on talk', 'long tock', 'nan', 'tonk',
        'honk', 'donkey', 'north dock', 'nacho', 'no clock', 'new dock', 'not here', 'one two', 'on to', 'non', 'Norton',
        'cantaloupe', 'notebook', 'not took', 'nun took', 'known talk', 'bento', 'lentil', 'Toronto', 'pronto', 'monotone']
WORDS = ('the a robot please can you bring this over there resistor on table lab is it time for lunch now what where when '
         'solder iron wire board check voltage turn off power hello thanks okay yes no maybe later today tomorrow project '
         'group meeting code upload motor wheel camera test again start stop wait going back left right forward tools '
         'screw driver pliers need pass me that one two three four five six seven eight nine ten report deadline professor '
         'coffee water charge battery cable usb laptop screen keyboard arduino jetson sensor reading number value high low').split()

def sentence():
    return ' '.join(random.choice(WORDS) for _ in range(random.randint(2, 9)))

def save(path, audio, rate):
    audio = np.asarray(audio, dtype=np.float32)
    if rate != 16000:
        audio = resample_poly(audio, 16000, rate)
    peak = np.abs(audio).max() or 1.0
    sf.write(path, (audio / peak * 0.8 * 32767).astype(np.int16), 16000, subtype='PCM_16')

def piper_batch(voice, texts, out_dir, count, prefix):
    os.makedirs(out_dir, exist_ok=True)
    n_speakers = voice.config.num_speakers
    for i in range(count):
        path = os.path.join(out_dir, '%s_%05d.wav' % (prefix, i))
        if os.path.exists(path):
            continue
        cfg = SynthesisConfig(speaker_id=random.randrange(n_speakers), length_scale=random.uniform(0.75, 1.5),
                              noise_scale=random.uniform(0.4, 0.9), noise_w_scale=random.uniform(0.5, 1.0))
        text = texts() if callable(texts) else random.choice(texts)
        chunks = [c.audio_float_array for c in voice.synthesize(text, syn_config=cfg)]
        save(path, np.concatenate(chunks), voice.config.sample_rate)

def say_batch(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    voices = [('Daniel', POS), ('Karen', POS), ('Moira', POS), ('Rishi', POS), ('Samantha', POS), ('Tessa', POS),
              ('Fred', POS), ('Kathy', POS), ('Ralph', POS), ('Albert', POS), ('Junior', POS),
              ('Eddy (English (US))', POS), ('Flo (English (UK))', POS), ('Reed (English (US))', POS),
              ('Sandy (English (UK))', POS), ('Shelley (English (US))', POS), ('Rocko (English (UK))', POS),
              ('Grandma (English (US))', POS), ('Grandpa (English (UK))', POS),
              ('Kanya', ['นนทก', 'นนทก!', 'นน ทก', 'Nontok']), ('Sinji', ['Nontok', 'non tok', 'nontock']),
              ('Tingting', ['Nontok', 'non tok', 'nontock'])]
    k = 0
    for name, texts in voices:
        for text in texts:
            for rate in (140, 180, 220):
                path = os.path.join(out_dir, 'say_%04d.wav' % k); k += 1
                if os.path.exists(path): continue
                tmp = path + '.aiff'
                r = subprocess.run(['say', '-v', name, '-r', str(rate), '-o', tmp, text], capture_output=True)
                if r.returncode == 0 and os.path.exists(tmp):
                    subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-i', tmp, '-ar', '16000', '-ac', '1', path])
                    os.remove(tmp)

if __name__ == '__main__':
    from huggingface_hub import hf_hub_download
    name = 'en/en_US/libritts_r/medium/en_US-libritts_r-medium.onnx'
    for part in (name, name + '.json'):
        hf_hub_download('rhasspy/piper-voices', part, local_dir='voices')
    # espeak cannot open its data through a long path, so reach it through a short link.
    import piper, tempfile
    link = os.path.join(tempfile.gettempdir(), 'piper-espeak-data')
    if not os.path.exists(link):
        os.symlink(os.path.join(os.path.dirname(piper.__file__), 'espeak-ng-data'), link)
    voice = PiperVoice.load(os.path.join('voices', name), espeak_data_dir=link)
    n_pos, n_neg = int(sys.argv[1]), int(sys.argv[2])
    if shutil.which('say'):
        say_batch('data/pos_say')
    piper_batch(voice, POS, 'data/pos', n_pos, 'pos')
    piper_batch(voice, NEAR, 'data/near', n_neg // 2, 'near')
    piper_batch(voice, sentence, 'data/sent', n_neg // 2, 'sent')
    print('done', {d: len(os.listdir('data/' + d)) for d in os.listdir('data')})
