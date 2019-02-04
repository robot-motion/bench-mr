#!/usr/bin/env bash
ffmpeg -i ../bin/log/step_%05d.png -qscale 2 -r 10 -vf tpad=stop_mode=clone:stop_duration=3 video.mp4
