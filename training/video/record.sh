#!/bin/bash
# Records tray-classifier videos from the robot's USB webcam at the robot's own
# resolution (640x480), so frames match what the Jetson sees.
#
#   ./record.sh preview                  live view, to aim the camera (q or Esc to close)
#   ./record.sh <item> <session>         record; press q in this terminal to stop
#
# Example:  ./record.sh bolt s1          -> videos/s1/bolt.mkv  (bolt_2.mkv if it exists)
#
# Each video: 2-3 s of the EMPTY surface first, then the item, hand away, moved between
# pauses (shift, rotate, flip). About 1 minute per item. One item per video.
# Mac only (ffmpeg avfoundation). CAM="name" picks another camera (default: "1080P USB Camera",
# the model used as the robot's tray camera, /dev/cam_tray).
set -euo pipefail
CAM="${CAM:-1080P USB Camera}"
HERE="$(cd "$(dirname "$0")" && pwd)"

# ffmpeg reads a name starting with digits as an index, so look the index up.
index=$(ffmpeg -hide_banner -f avfoundation -list_devices true -i "" 2>&1 \
        | sed -n '/video devices/,/audio devices/p' | grep -F "] $CAM" | head -1 \
        | sed -E 's/.*\[([0-9]+)\] .*/\1/' || true)
[ -n "$index" ] || { echo "Camera \"$CAM\" not found. Plug it in, or set CAM=..."; exit 1; }
IN=(-f avfoundation -pixel_format uyvy422 -video_size 640x480 -framerate 30 -i "$index:none")

if [ "${1:-}" = preview ]; then
    exec ffplay -hide_banner -loglevel error -window_title "tray camera preview (q to close)" "${IN[@]}"
fi
[ $# -eq 2 ] || { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
item="$1"; session="$2"
dir="$HERE/videos/$session"; mkdir -p "$dir"
out="$dir/$item.mkv"; n=2
while [ -e "$out" ]; do out="$dir/${item}_$n.mkv"; n=$((n + 1)); done
echo "Recording $out  (camera [$index] $CAM, 640x480 @ 30 fps). Press q to stop."
ffmpeg -hide_banner -loglevel warning -stats "${IN[@]}" \
       -c:v libx264 -preset veryfast -crf 12 -pix_fmt yuv420p "$out"
echo "Saved $out ($(du -h "$out" | cut -f1))"
