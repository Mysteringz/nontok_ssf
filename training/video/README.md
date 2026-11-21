# Item classifier from videos

Builds training images for the tray item classifier (PLAN.md 7.4, 13.10) from short videos,
without the robot: record on a laptop with the tray camera model, extract the still frames,
crop them exactly like the robot, train, and check on a test video.

| File | What it does |
|---|---|
| `record.sh` | Mac (ffmpeg avfoundation): records `videos/<session>/<item>.mkv` at 640x480, 30 fps |
| `video_to_dataset.py` | Videos -> `dataset/<session>/<item>/*.jpg`, the robot's crop (`tray_logic.item_crop`) |
| `train_quick.py` | Quick check: split by time, YOLO11n-cls on MPS/GPU, export `items.onnx`, score with the robot's `preprocess` |
| `detect_video.py` | Runs `items.onnx` through a video: what was on the surface, and when |

Videos, datasets, runs and `.onnx` files stay local (`.gitignore`).

## Setup (Mac)

```bash
conda create -n David python=3.11 && conda activate David
pip install ultralytics onnx onnxruntime onnxslim opencv-python pyyaml
brew install ffmpeg
```

## Recording

Use the tray camera model: the **"1080P USB Camera"** (`2bdf:0289`, `/dev/cam_tray` on the robot).

```bash
cd training/video
./record.sh preview          # aim the camera; q to close
./record.sh bolt s1          # item "bolt", session "s1"; press q to stop
```

- Camera **fixed, looking straight down**, at the tray camera's height, over a surface like
  the tray floor. **The camera must not move during a video.**
- Wait ~5 s after starting (the camera sends black frames while it warms up), keep 2-3 s of
  the empty surface, then put the item in.
- Hand away for ~1 s between moves; shift, rotate and flip the item. About 1 minute per item.
- One video per item; a new session name per day or lighting setup. Item names as in
  `ros2_ws/src/nontok_perception/config/items.yaml`.

## Videos -> images -> model

```bash
python video_to_dataset.py videos/s1/*.mkv --show        # open dataset/review_*.jpg, delete bad crops
python train_quick.py --items led resistor multimeter     # -> items.onnx and a score
python detect_video.py videos/test/test.mkv               # -> what appeared when, + test_detections.jpg
```

- `video_to_dataset.py` keeps frames where the item lies still and drops near-duplicates. The
  empty-surface reference is a moving median of the video (the robot captures an explicit
  empty-tray reference instead). The crop is pixel-identical to `TrayLogic.item_crop` (checked
  on synthetic items with exposure shifts): grey, 5x5 blur, brightness offset removed,
  difference > 25, square 1.4x the box, at least 96 px, 224x224.
  Recording-only extra: changed regions touching the image border are ignored (camera shake).
- `train_quick.py` holds out the last 25 % of each item's images: optimistic. For the real
  model, use `../split_by_session.py` (whole sessions held out) and `../train_cls.py`.
- Every item the robot may see needs a class; anything else is reported as "unknown" only
  when confidence is below 0.80.
