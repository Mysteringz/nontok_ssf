#!/usr/bin/env python3
"""Turns the item descriptions in items.yaml into the file the zero-shot classifier reads.

Run on a PC after editing the descriptions (pip install onnxruntime tokenizers huggingface_hub pyyaml):

    python3 training/build_item_prompts.py

Downloads the CLIP text and vision models once (to training/clip/), writes
ros2_ws/src/nontok_perception/models/items_clip.npz, and copies the vision model next to it
as clip_vision.onnx. Only those two files are needed on the robot.
"""
import argparse
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.join(HERE, '..', 'ros2_ws', 'src', 'nontok_perception')
sys.path.insert(0, PACKAGE)
REPO = 'Xenova/clip-vit-base-patch32'
TEMPLATES = ['a photo of {}', 'a close-up photo of {} seen from above', '{} lying in a tray']


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--items', default=os.path.join(PACKAGE, 'config', 'items.yaml'))
    parser.add_argument('--models', default=os.path.join(PACKAGE, 'models'))
    parser.add_argument('--cache', default=os.path.join(HERE, 'clip'))
    args = parser.parse_args()
    import onnxruntime
    from huggingface_hub import hf_hub_download
    from tokenizers import Tokenizer

    from nontok_perception.classifier import load_descriptions

    paths = {name: hf_hub_download(REPO, name, local_dir=args.cache)
             for name in ('onnx/text_model.onnx', 'onnx/vision_model.onnx', 'tokenizer.json')}
    tokenizer = Tokenizer.from_file(paths['tokenizer.json'])
    tokenizer.enable_padding(length=77, pad_id=49407)
    tokenizer.enable_truncation(77)
    text_model = onnxruntime.InferenceSession(paths['onnx/text_model.onnx'], providers=['CPUExecutionProvider'])
    wanted = {i.name for i in text_model.get_inputs()}

    labels, rows = [], []
    for label, phrases in load_descriptions(args.items).items():
        sentences = [template.format(phrase) for phrase in phrases for template in TEMPLATES]
        encoded = tokenizer.encode_batch(sentences)
        feed = {'input_ids': np.array([e.ids for e in encoded], dtype=np.int64),
                'attention_mask': np.array([e.attention_mask for e in encoded], dtype=np.int64)}
        embeddings = text_model.run(['text_embeds'], {k: v for k, v in feed.items() if k in wanted})[0]
        embeddings = embeddings / np.linalg.norm(embeddings, axis=1, keepdims=True)
        mean = embeddings.mean(axis=0)
        labels.append(label)
        rows.append(mean / np.linalg.norm(mean))
        print('%-16s %d phrases' % (label, len(sentences)))
    os.makedirs(args.models, exist_ok=True)
    out = os.path.join(args.models, 'items_clip.npz')
    np.savez(out, labels=np.array(labels), embeddings=np.array(rows, dtype=np.float32), scale=np.float32(100.0))
    shutil.copyfile(paths['onnx/vision_model.onnx'], os.path.join(args.models, 'clip_vision.onnx'))
    print('Wrote %s and clip_vision.onnx (%d labels)' % (out, len(labels)))


if __name__ == '__main__':
    main()
