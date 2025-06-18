export XDG_RUNTIME_DIR=/dev/socket/weston && export WAYLAND_DISPLAY=wayland-1



python3 /usr/bin/gst-ai-object-detection.py \
  -f 2 \
  -ml yolov8 \
  -m /opt/yamnet.tflite \
  -l /opt/yolov8.labels \
  -k "YOLOv8,q-offsets=<-107.0, -128.0, 0.0>,q-scales=<3.093529462814331, 0.00390625, 1.0>"

