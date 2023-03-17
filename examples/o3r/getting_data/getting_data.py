from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id

def main():
  # Initialize the objects
  o3r = O3R()
  fg = FrameGrabber(o3r, pcic_port=50012)

  # Set schema and start Grabber
  fg.start([buffer_id.NORM_AMPLITUDE_IMAGE,buffer_id.RADIAL_DISTANCE_IMAGE,buffer_id.XYZ])

  # Get a frame
  [ok, frame] = fg.wait_for_frame().wait_for(1500) # wait with 1500ms timeout

  # Read the distance image and display a pixel in the center
  dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
  (width, height) = dist.shape
  print(dist[width//2,height//2])
  fg.stop()

if __name__ == "__main__":
    main()
