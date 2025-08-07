#cense-Identifier: BSD-3-Clause-Clear
################################################################################

import os
import sys
import signal
import argparse
import numpy as np
import socket
import threading
import time

import gi
gi.require_version('Gst', '1.0')
gi.require_version("GLib", "2.0")
from gi.repository import Gst, GLib

# Constants
DESCRIPTION = """
This app sets up GStreamer pipeline for depth estimation using segmentation models.
Initializes and links elements for capturing live stream from camera or offline source,
performs inference (depth estimation) using MODEL file,
and renders depth map on display or dumps it as OUTPUT.
"""
DELEGATE_PATH = "libQnnTFLiteDelegate.so"

# Framework
SNPE = 1
TFLITE = 2

ML_PLUGINS = {
    SNPE   : 'qtimlsnpe',
    TFLITE : 'qtimltflite'
}

waiting_for_eos = False
eos_received = False

# UDP server setup
UDP_IP = "192.168.1.28" 
UDP_PORT = 9090
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((UDP_IP, UDP_PORT))
client_address = None
center_values = []
latest_distance = None

def udp_wait_client():
    global client_address
    data, client_address = server_socket.recvfrom(1024)

def handle_interrupt_signal(pipeline, mloop):
    """Handle Ctrl+C."""
    global waiting_for_eos
    _, state, _ = pipeline.get_state(Gst.CLOCK_TIME_NONE)
    if state != Gst.State.PLAYING or waiting_for_eos:
        mloop.quit()
        return GLib.SOURCE_CONTINUE
    event = Gst.Event.new_eos()
    if pipeline.send_event(event):
        print("EoS sent to the pipeline")
        waiting_for_eos = True
    else:
        print("Failed to send EoS event to the pipeline!")
        mloop.quit()
    return GLib.SOURCE_CONTINUE

def handle_bus_message(bus, message, mloop):
    """Handle messages posted on pipeline bus."""
    global eos_received
    if message.type == Gst.MessageType.ERROR:
        error, debug_info = message.parse_error()
        print("ERROR:", message.src.get_name(), " ", error.message)
        if debug_info:
            print("Debugging info:", debug_info)
        mloop.quit()
    elif message.type == Gst.MessageType.EOS:
        print("EoS received")
        eos_received = True
        mloop.quit()
    return True

def create_element(factory_name, name):
    """Create a GStreamer element."""
    element = Gst.ElementFactory.make(factory_name, name)
    if not element:
        raise Exception(f"Unable to create element {name}")
    return element

def on_pad_added(_, pad, target):
    """Link dynamic pads from demuxer to target element."""
    if "video" in pad.get_name():
        sink_pad = target.get_static_pad("sink")
        if not sink_pad.is_linked():
            if pad.link(sink_pad) != Gst.PadLinkReturn.OK:
                raise Exception(f"Failed linking demux to queue")

def link_elements(link_orders, elements):
    """Link elements in the specified order."""
    for link_order in link_orders:
        src = None
        for element in link_order:
            dest = elements[element]
            if src:
                if src.get_name() == "demux":
                    src.connect("pad-added", on_pad_added, dest)
                elif not src.link(dest):
                    raise Exception(
                        f"Unable to link element {src.get_name()} to {dest.get_name()}"
                    )
            src = dest

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description=DESCRIPTION,
        formatter_class=type(
            'CustomFormatter',
            (argparse.ArgumentDefaultsHelpFormatter, argparse.RawTextHelpFormatter),
            {}
        )
    )
    parser.add_argument(
        '-c', '--camera', type=int, choices=[0, 1], default=0,
        help='Select (0) for Primary Camera and (1) for Secondary Camera.\n'
    )
    parser.add_argument(
        '-cw', '--width', type=int, default=1280,
        help='Camera Output Width'
    )
    parser.add_argument(
        '-ch', '--height', type=int, default=720,
        help='Camera Output Height'
    )
    parser.add_argument(
        '-cf', '--framerate', type=str, default='60/1',
        help='Camera Output Framerate (fraction)'
    )
    parser.add_argument(
        '-s', '--file-path', type=str,
        help='File source path. If not specified, input is taken from the camera.'
    )
    parser.add_argument(
        '-f', '--ml-framework', type=int, choices=[1, 2], required=True,
        help='Execute Model in SNPE DLC (1) or TFlite (2) format'
    )

    parser.add_argument('-ml', '--module', type=str, required=True,
                        help='The ML module to be used (e.g., midas-v2).')
    parser.add_argument('-m', '--model', type=str, required=True,
                        help='Path to the model file.')
    parser.add_argument('-l', '--labels', type=str,
                        help='Path to the labels file. Required for certain models.')
    parser.add_argument('-k', '--constants', type=str,
                        help='Constants, offsets and coefficients used by the module for post-processing of incoming tensors.')
    parser.add_argument('-o', '--output', type=str, help='Output File Path. If not specified, the output is displayed on the screen.')
    parser.add_argument('--use_cpu', action='store_true', help='Optional parameter to inference on CPU Runtime')
    parser.add_argument('--use_gpu', action='store_true', help='Optional parameter to inference on GPU Runtime')
    parser.add_argument('--use_dsp', action='store_true', default=True,
                        help='Default and optional parameter to inference on DSP Runtime')
    args = parser.parse_args()
    # Validate arguments based on ML framework
    if args.ml_framework == TFLITE and not args.constants:
        parser.error("--constants is required when --ml-framework is 2 (TFLite)")
    return args

def create_source_elements(args):
    """Create source elements based on input type."""
    if args.file_path:
        source_elements = {
            "filesrc": create_element("filesrc", "src"),
            "demux": create_element("qtdemux", "demux"),
            "parse": create_element("h264parse", "parse"),
            "decoder": create_element("v4l2h264dec", "decoder"),
            "deccaps": create_element("capsfilter", "deccaps")
        }
        source_elements["filesrc"].set_property("location", args.file_path)
        source_elements["decoder"].set_property("capture-io-mode", "dmabuf")
        source_elements["decoder"].set_property("output-io-mode", "dmabuf")
        source_elements["deccaps"].set_property(
            "caps", Gst.Caps.from_string("video/x-raw,format=NV12")
        )
    else:
        source_elements = {
            "camsrc": create_element("qtiqmmfsrc", "camsrc"),
            "camcaps": create_element("capsfilter", "camcaps")
        }
        source_elements["camsrc"].set_property("camera", args.camera)
        source_elements["camcaps"].set_property(
            "caps", Gst.Caps.from_string(
                "video/x-raw,format=NV12,"
                f"width={args.width},height={args.height},"
                f"framerate={args.framerate}"
            )
        )
    return source_elements

def create_sink_elements(args):
    """Create sink elements based on output type."""
    sink_elements = {}
    if args.output:
        sink_elements["sink"] = create_element("filesink", "sink")
        sink_elements["sink"].set_property("location", args.output)
    else:
        sink_elements["display"] = create_element("waylandsink", "display")
        sink_elements["display"].set_property("fullscreen", True)
        if args.file_path is None:
            sink_elements["display"].set_property("sync", False)
        sink_elements["appsink"] = create_element("appsink", "appsink")
        sink_elements["appsink"].set_property("emit-signals", True)
        sink_elements["appsink"].set_property("sync", False)
    return sink_elements

def set_snpe_properties(elements, args):
    """Set properties for SNPE element."""
    if args.use_cpu:
        elements["ml"].set_property("delegate", "none")
        print("Using CPU delegate")
    elif args.use_gpu:
        elements["ml"].set_property("delegate", "gpu")
        print("Using GPU delegate")
    elif args.use_dsp:
        elements["ml"].set_property("delegate", "dsp")
        print("Using DSP delegate")

def set_tflite_properties(elements, args):
    """Set properties for TFLite element."""
    if args.use_cpu:
        elements["ml"].set_property("delegate", "none")
        print("Using CPU delegate")
    elif args.use_gpu:
        elements["ml"].set_property("delegate", "gpu")
        print("Using GPU delegate")
    elif args.use_dsp:
        print("Using DSP delegate")
        delegate_options = Gst.Structure.new_empty("QNNExternalDelegate")
        delegate_options.set_value("backend_type", "htp")
        elements["ml"].set_property("delegate", "external")
        elements["ml"].set_property("external-delegate-path", DELEGATE_PATH)
        elements["ml"].set_property("external-delegate-options", delegate_options)

def create_link_orders(args):
    """Create link orders based on source and sink types."""
    link_orders = [
        ["split", "queue1", "converter", "queue2", "ml"]
    ]
    if args.file_path:
        link_orders.append(["filesrc", "demux", "parse", "decoder", "deccaps", "queue0", "split"])
    else:
        link_orders.append(["camsrc", "camcaps", "queue0", "split"])
    if args.output:
        link_orders.append(["ml", "tee3", "queue3", "segmentation", "overlay", "queue4", "sink"])
    else:
        link_orders.append(["ml", "tee3"])
        link_orders.append(["tee3", "queue3", "segmentation", "overlay", "tee2"])
        link_orders.append(["tee2", "queue4", "display"])
        link_orders.append(["tee3", "queue5", "appsink"])
    return link_orders

def create_pipeline(pipeline, args):
    """Initialize and link elements for the GStreamer pipeline."""
    if not os.path.exists(args.model):
        print(f"File {args.model} does not exist")
        sys.exit(1)
    elements = {
        "split"        : create_element("tee", "split"),
        "overlay"      : create_element("qtivoverlay", "overlay"),
        "tee2"         : create_element("tee", "tee2"),
        "tee3"         : create_element("tee", "tee3"),
        "converter"    : create_element("qtimlvconverter", "converter"),
        "ml"           : create_element(ML_PLUGINS.get(args.ml_framework), "ml"),
        "segmentation" : create_element("qtimlvsegmentation", "segmentation")
    }
    source_elements = create_source_elements(args)
    sink_elements = create_sink_elements(args)
    elements.update(source_elements)
    elements.update(sink_elements)
    queue_count = 6
    for i in range(queue_count):
        queue_name = f"queue{i}"
        elements[queue_name] = create_element("queue", queue_name)
    elements["ml"].set_property("model", args.model)
    if args.ml_framework == SNPE:
        set_snpe_properties(elements, args)
    elif args.ml_framework == TFLITE:
        set_tflite_properties(elements, args)
    elements["segmentation"].set_property("module", args.module)
    if args.labels:
        elements["segmentation"].set_property("labels", args.labels)
    if args.constants:
        elements["segmentation"].set_property("constants", args.constants)
    pipeline.add(*elements.values())
    link_orders = create_link_orders(args)
    link_elements(link_orders, elements)

def is_linux():
    try:
        with open("/etc/os-release") as f:
            for line in f:
                if "Linux" in line:
                    return True
    except FileNotFoundError:
        return False
    return False

def pixel_to_distance(pixel):
    mapping = [(70, 33.0), (73, 34.0), (78, 35.0)]
    mapping.sort()
    for i in range(len(mapping) - 1):
        x0, y0 = mapping[i]
        x1, y1 = mapping[i + 1]
        if x0 <= pixel <= x1:
            return y0 + (y1 - y0) * (pixel - x0) / (x1 - x0)
    if pixel < mapping[0][0]:
        return mapping[0][1]
    else:
        return mapping[-1][1]

def send_distance_periodically():
    global client_address, latest_distance
    while True:
        if client_address is not None and latest_distance is not None:
            msg = f"Distance: {latest_distance:.2f} cm".encode()
            try:
                server_socket.sendto(msg, client_address)
            except Exception as e:
                print(f"Send error: {e}")
        time.sleep(1)

def on_new_sample(sink):
    global client_address, latest_distance
    sample = sink.emit("pull-sample")
    if sample:
        buf = sample.get_buffer()
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        if result:
            arr = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape(256, 256)
            pixel_128_128 = arr[128, 128]
            distance = pixel_to_distance(pixel_128_128)
            print(f"Pixel at (128,128): {pixel_128_128} | Distance: {distance:.2f} cm")
            latest_distance = distance
            buf.unmap(mapinfo)
    return Gst.FlowReturn.OK

def main():
    """Main function to set up and run the GStreamer pipeline."""
    if is_linux():
        os.environ["XDG_RUNTIME_DIR"] = "/dev/socket/weston"
        os.environ["WAYLAND_DISPLAY"] = "wayland-1"
    threading.Thread(target=udp_wait_client, daemon=True).start()
    threading.Thread(target=send_distance_periodically, daemon=True).start()
    Gst.init(None)
    mloop = GLib.MainLoop()
    args = parse_arguments()
    try:
        pipeline = Gst.Pipeline.new("depth-estimation-pipeline")
        if not pipeline:
            raise Exception("Unable to create depth estimation pipeline")
        create_pipeline(pipeline, args)
    except Exception as e:
        print(f"{e} Exiting...")
        return -1
    appsink = pipeline.get_by_name("appsink")
    if appsink:
        appsink.connect("new-sample", on_new_sample)
    interrupt_watch_id = GLib.unix_signal_add(
        GLib.PRIORITY_HIGH, signal.SIGINT, handle_interrupt_signal, pipeline, mloop
    )
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", handle_bus_message, mloop)
    pipeline.set_state(Gst.State.PLAYING)
    print("------------------------------")
    print("test info:",pipeline)
    mloop.run()
    print("------test info--------:",pipeline) 
    GLib.source_remove(interrupt_watch_id)
    bus.remove_signal_watch()
    bus = None
    print("Setting to NULL...")
    pipeline.set_state(Gst.State.NULL)
    mloop = None
    pipeline = None
    Gst.deinit()
    if eos_received:
        print("App execution successful")

if __name__ == "__main__":
    sys.exit(main())
