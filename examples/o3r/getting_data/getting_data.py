from ifm3dpy import O3R, FrameGrabber, buffer_id
import asyncio

async def getter(fg):
      return await fg.wait_for_frame()
# Initialize the objects
o3r = O3R('192.168.0.69')
port='port2'
fg = FrameGrabber(o3r, pcic_port=50012)

#set schema and start Grabber
fg.start([buffer_id.NORM_AMPLITUDE_IMAGE,buffer_id.RADIAL_DISTANCE_IMAGE,buffer_id.XYZ])

# Get a frame
frame =  getter(fg)

# Read the distance image and display a pixel in the center
dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
(width, height) = dist.shape
print(dist[width//2,height//2])
