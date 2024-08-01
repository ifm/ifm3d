#%%
from ifm3dpy.device import O3R, O3D, O3X
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
from ifm3dpy.logging import Logger, LogLevel
import time
import json
# Set a log level
Logger.set_log_level(LogLevel.Debug)

device = O3D("192.168.0.70")
# device.from_json(json.loads('{"ifm3d":{"Apps":[{"TriggerMode": "0"}]}}'))
pcic_port = int(device.device_parameter("PcicTcpPort"))

fg = FrameGrabber(device, pcic_port=pcic_port)#, 50012)

fg.start(
    [
        buffer_id.EXPOSURE_TIME,
        buffer_id.EXTRINSIC_CALIB,
        buffer_id.INTRINSIC_CALIB,
        buffer_id.INVERSE_INTRINSIC_CALIBRATION,
        buffer_id.ILLUMINATION_TEMP,
    ]
)

time.sleep(5)

[ok, frame] = fg.wait_for_frame().wait_for(1000)

if not ok:
    raise TimeoutError

available_buffers = frame.get_buffers()
print(f"Available buffers: {available_buffers}")
try:
    data = frame.get_buffer(buffer_id.ILLUMINATION_TEMP)
    print(f"Data from buffer: {data}")
except:
    print("No data in buffer")
#%%
fg.stop()
#%%
# import numpy as np
# import struct

# unpacked_data = struct.unpack('<4i2I', data)

# print(unpacked_data)
