from ifm3dpy import O3RCamera, ImageBuffer, FrameGrabber

# Initialize the objects
o3r = O3RCamera('192.168.0.69')
port='port2'
fg = FrameGrabber(o3r, pcic_port=50012)
im = ImageBuffer()

# Get a frame
if fg.wait_for_frame(im, 500)==False:
    raise ValueError #Exception('fg-timeout on ' + port + ' reached')

# Read the distance image and display a pixel in the center
dist = im.distance_image()
print(dist[100, 100])
