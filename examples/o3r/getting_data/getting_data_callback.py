from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
import time


def callback(frame):
    # Read the distance image and display a pixel in the center
    dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
    (width, height) = dist.shape
    print(dist[width//2,height//2])
    pass


# Initialize the objects
o3r = O3R()
fg = FrameGrabber(o3r, pcic_port=50012)

#set schema and start Grabber
fg.start([buffer_id.NORM_AMPLITUDE_IMAGE,buffer_id.RADIAL_DISTANCE_IMAGE,buffer_id.XYZ])

fg.on_new_frame(callback)

# Sleep to avoid exiting the program too soon
time.sleep(1)

fg.stop()
