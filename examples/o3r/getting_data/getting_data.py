from ifm3dpy import O3R, FrameGrabber, buffer_id

# Initialize the objects
o3r = O3R()
fg = FrameGrabber(o3r, pcic_port=50012)

#set schema and start Grabber
fg.start([buffer_id.NORM_AMPLITUDE_IMAGE,buffer_id.RADIAL_DISTANCE_IMAGE,buffer_id.XYZ])

# Get a frame
frame =  fg.wait_for_frame()

# Read the distance image and display a pixel in the center
dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
(width, height) = dist.shape
print(dist[width//2,height//2])
