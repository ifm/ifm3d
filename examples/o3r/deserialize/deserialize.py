#################################################################
# This examples shows how to use the deserializer module
# to extract data from the RGBInfoV1 buffer.
# The same principles can be applied to deserialize data from
# other buffers (see accompanying documentation for mode details)
#################################################################
# Import the relevant modules
from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
from ifm3dpy.deserialize import RGBInfoV1

###########################
# Choose the IP
# and create the O3R object
###########################
IP = "192.168.0.69"
o3r = O3R(IP)

###########################
# Choose the port number
# and choose which images
# to receive
###########################
# Assuming this is a 2D port
PORT = 0
pcic_port = o3r.get()["ports"][f"port{PORT}"]["data"]["pcicTCPPort"]
fg = FrameGrabber(cam=o3r, pcic_port=pcic_port)
# Define the images to receive when starting the data stream
fg.start([buffer_id.RGB_INFO])
try:
    # Get a frame
    [ok, frame] = fg.wait_for_frame().wait_for(500)
    # Retrieve the data from the relevant buffer
    if ok:
        rgb_info_buffer = frame.get_buffer(buffer_id.RGB_INFO)
    else:
        raise TimeoutError
except Exception as e:
    raise e

###############################
# Extract data from the buffer
# Using the deserializer module
###############################
rgb_info = RGBInfoV1()
rgb_info_deserialized = rgb_info.deserialize(rgb_info_buffer)
print("Sample of data available in the RGBInfoV1 buffer:")
print(f"RGB info timestamp: {rgb_info_deserialized.timestamp_ns}")
print(f"Exposure time used for rgb images: {rgb_info_deserialized.exposure_time}")
print(
    f"RGB intrinsic calibration model id: {rgb_info_deserialized.intrinsic_calibration.model_id}"
)
print(
    f"RGB intrinsic calibration parameters: {rgb_info_deserialized.intrinsic_calibration.parameters}"
)
