#!/usr/bin/env python3
"""Step 3 of 3: train the "Nontok" detector on the features and export nontok.onnx.

Needs, in neg/ (from huggingface.co/datasets/davidscripka/openwakeword_features):
  validation_set_features.npy                           11 h; the last fifth is held out
  acav_part.npy   the first 3 GB (or all) of openwakeword_features_ACAV100M_2000_hrs_16bit.npy
                  e.g. curl -L -r 0-3221225599 -o neg/acav_part.npy <file url>

    python3 train.py        # about 15 minutes on a laptop CPU

Prints, per threshold: calls detected (held-out Piper clips, held-out macOS voices), false
alarms on near-miss phrases and sentences, and false triggers per hour on 2 h of unseen
speech and music. Copy nontok.onnx to ros2_ws/src/nontok_voice/models/.
Result of 16 November 2026: 93 % / 80 % detected, 2-3 false triggers per hour.
"""
import os, sys, numpy as np, torch, torch.nn as nn
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'ros2_ws', 'src', 'nontok_voice'))
from nontok_voice.wake_logic import WakeTrigger
torch.manual_seed(0); rng = np.random.default_rng(0)
L = lambda n: np.load('feat_%s.npy' % n)
pos, say, near, sent, amb = L('pos'), L('pos_say'), L('near'), L('sent'), L('ambient')
# macOS voices: 6 windows per file, 30 files per voice for the first 19 voices; hold out every other voice.
file_idx = np.arange(len(say)) // 6
voice = np.where(file_idx < 570, file_idx // 30, 19 + (file_idx - 570) // 3)   # Thai/Chinese voices: groups of 3 files
say_train, say_test = say[voice % 2 == 0], say[voice % 2 == 1]
cut = lambda a, f=0.9: (a[:int(len(a) * f)], a[int(len(a) * f):])
pos_tr, pos_va = cut(pos); near_tr, near_va = cut(near); sent_tr, sent_va = cut(sent); amb_tr, amb_va = cut(amb)
val = np.load('neg/validation_set_features.npy', mmap_mode='r')
split = int(len(val) * 0.8)
starts = rng.integers(0, split - 16, 250000)
bg_tr = np.stack([val[s:s + 16] for s in starts])
held = np.asarray(val[split:])                                   # ~2.1 h, never trained on
# First part of the 2000 h ACAV100M feature set (the file is cut short; read what is whole).
with open('neg/acav_part.npy', 'rb') as f:
    np.lib.format.read_magic(f); shape, _, dtype = np.lib.format.read_array_header_1_0(f); offset = f.tell()
n_whole = (os.path.getsize('neg/acav_part.npy') - offset) // (16 * 96 * dtype.itemsize)
acav = np.memmap('neg/acav_part.npy', dtype=dtype, mode='r', offset=offset, shape=(n_whole, 16, 96))
print('acav windows', n_whole, dtype, 'hours', n_whole * 1.28 / 3600)
pick = np.sort(rng.choice(n_whole - 20000, 700000, replace=False))
acav_tr = np.asarray(acav[pick], dtype=np.float32); acav_va = np.asarray(acav[-20000:], dtype=np.float32)
Xp = np.concatenate([pos_tr, np.repeat(say_train, 3, axis=0)])
Xn = np.concatenate([near_tr, sent_tr, np.repeat(amb_tr, 10, axis=0), bg_tr, acav_tr])
print('train pos', len(Xp), 'neg', len(Xn), 'held-out background h', len(held) * 0.08 / 3600)
X = torch.tensor(np.concatenate([Xp, Xn])); y = torch.tensor(np.r_[np.ones(len(Xp)), np.zeros(len(Xn))], dtype=torch.float32)

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.body = nn.Sequential(nn.Flatten(), nn.Linear(16 * 96, 256), nn.LayerNorm(256), nn.ReLU(),
                                  nn.Linear(256, 128), nn.LayerNorm(128), nn.ReLU(), nn.Linear(128, 1))
    def forward(self, x):
        return torch.sigmoid(self.body(x))
net = Net(); opt = torch.optim.AdamW(net.parameters(), 1e-3, weight_decay=1e-2)
EPOCHS = 24; sched = torch.optim.lr_scheduler.OneCycleLR(opt, 3e-3, total_steps=EPOCHS * (len(X) // 1024 + 1))
weight = torch.ones(len(X))
for epoch in range(EPOCHS):
    net.train(); perm = torch.randperm(len(X))
    for i in range(0, len(X), 1024):
        idx = perm[i:i + 1024]; out = net(X[idx] + 0.02 * torch.randn_like(X[idx])).squeeze(1)
        w = torch.where(y[idx] > 0, 8.0, 1.0) * weight[idx]   # few positives, many negatives       # a false trigger costs more than a missed call
        loss = (nn.functional.binary_cross_entropy(out, y[idx], reduction='none') * w).mean()
        opt.zero_grad(); loss.backward(); opt.step(); sched.step()
    if epoch in (8, 14, 19):                                    # lean on the negatives it still gets wrong
        with torch.no_grad(): s = torch.cat([net(X[i:i + 8192]).squeeze(1) for i in range(0, len(X), 8192)])
        weight = torch.where((y == 0) & (s > 0.1), 20.0, 1.0)
net.eval()
score = lambda a: np.concatenate([net(torch.tensor(a[i:i + 8192])).squeeze(1).detach().numpy() for i in range(0, len(a), 8192)])
windows = np.lib.stride_tricks.sliding_window_view(held, (16, 96)).reshape(-1, 16, 96)
bg = score(windows); hours = len(bg) * 0.08 / 3600
sv = {k: score(v) for k, v in dict(piper=pos_va, mac_voices=say_test, near=near_va, sentences=sent_va, lab=amb_va, acav=acav_va).items()}
print('thr  piper  mac_voices | false: near-miss  sentences  lab-windows  acav-windows | background triggers/hour')
for thr in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
    t = WakeTrigger(thr); trig = sum(t.update(float(s), i * 0.08) for i, s in enumerate(bg))
    print('%.1f  %5.1f%%  %5.1f%%   |   %5.2f%%  %5.2f%%  %5.2f%%  %6.3f%%   |  %.2f' % (
        thr, 100 * (sv['piper'] >= thr).mean(), 100 * (sv['mac_voices'] >= thr).mean(), 100 * (sv['near'] >= thr).mean(),
        100 * (sv['sentences'] >= thr).mean(), 100 * (sv['lab'] >= thr).mean(), 100 * (sv['acav'] >= thr).mean(), trig / hours))
torch.onnx.export(net, torch.zeros(1, 16, 96), 'nontok.onnx', input_names=['x'], output_names=['score'], opset_version=13, dynamo=False)
torch.save(net.state_dict(), 'nontok.pt'); print('exported nontok.onnx')
