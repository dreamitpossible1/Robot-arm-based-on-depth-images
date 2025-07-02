export XDG_RUNTIME_DIR=/dev/socket/weston && export WAYLAND_DISPLAY=wayland-1



python get_dep.py -f 2 -ml midas-v2 --model /opt/midas_quantized.tflite --labels /opt/monodepth.labels --constants "Midas,q-offsets=<0.0>,q-scales=<6.846843242645264>;"

