#!/bin/bash
# Retime the raw recording to real time, then bake the README GIF. Run on the host (needs ffmpeg).
# Usage: make_gif.sh <wrench.mp4> <frames> <seconds> [speedup=3]
# <frames> is printed by record.py; <seconds> is the recording duration passed to it.
set -e
in="$1"; frames="$2"; secs="$3"; speed="${4:-3}"
fps=$(python3 -c "print($frames/$secs)")
ffmpeg -y -loglevel error -i "$in" -vf "setpts=PTS*10/$fps" -r 10 -c:v libx264 -pix_fmt yuv420p -crf 26 brov2_wrench_sim.mp4
ffmpeg -y -loglevel error -i brov2_wrench_sim.mp4 \
  -vf "setpts=PTS/$speed,fps=6,scale=400:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer:bayer_scale=5" \
  media/brov2_wrench_sim.gif
ls -la media/brov2_wrench_sim.gif

