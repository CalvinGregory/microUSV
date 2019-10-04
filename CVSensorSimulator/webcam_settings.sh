# Script to configure webcam settings for use with the microUSV.

# Disable Auto focus
v4l2-ctl -d /dev/video2 --set-ctrl=focus_auto=0
v4l2-ctl -d /dev/video2 --set-ctrl=focus_absolute=0

# Set exposure manually
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_absolute=250

# Set gain level
v4l2-ctl -d /dev/video2 --set-ctrl=gain=255

# Set sharpness
v4l2-ctl -d /dev/video2 --set-ctrl=sharpness=255


