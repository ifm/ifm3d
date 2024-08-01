import time
from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id

o3r = O3R()
fg = FrameGrabber(o3r, 50012) 

buffer_ids = [
    # buffer_id.ALGO_DEBUG,
    # buffer_id.AMPLITUDE_IMAGE, # Triggers endless error.
    # buffer_id.CARTESIAN_ALL, # Triggers endless error.
    # buffer_id.CARTESIAN_X_COMPONENT, # Triggers endless error.
    # buffer_id.CARTESIAN_Y_COMPONENT, # Triggers endless error.
    # buffer_id.CARTESIAN_Z_COMPONENT, # Triggers endless error.
    buffer_id.CONFIDENCE_IMAGE,
    buffer_id.DIAGNOSTIC,
    buffer_id.EXPOSURE_TIME,
    buffer_id.EXTRINSIC_CALIB,
    buffer_id.GRAYSCALE_IMAGE,
    buffer_id.INTRINSIC_CALIB,
    buffer_id.INVERSE_INTRINSIC_CALIBRATION,
    buffer_id.JPEG_IMAGE,
    buffer_id.JSON_DIAGNOSTIC,
    buffer_id.JSON_MODEL,
    buffer_id.MONOCHROM_2D,
    buffer_id.MONOCHROM_2D_12BIT,
    buffer_id.NORM_AMPLITUDE_IMAGE,
    buffer_id.O3R_ODS_INFO,
    buffer_id.O3R_ODS_OCCUPANCY_GRID,
    buffer_id.O3R_RESULT_ARRAY2D,
    buffer_id.O3R_RESULT_IMU,
    buffer_id.O3R_RESULT_JSON,
    buffer_id.RADIAL_DISTANCE_IMAGE,
    buffer_id.RADIAL_DISTANCE_NOISE,
    buffer_id.REFLECTIVITY,
    buffer_id.RGB_INFO,
    buffer_id.TOF_INFO,
    buffer_id.UNIT_VECTOR_ALL,
    buffer_id.XYZ
]

available_buffers = []
timeout_buffers = []
empty_frame_buffers = []
error_buffers = []

for buffer in buffer_ids:
    time.sleep(5)
    print(f"Trying out buffer: {buffer}")
    fg.start([buffer])
    time.sleep(5)
    try:
        [ok, frame] = fg.wait_for_frame().wait_for(500)
        if not ok:
            timeout_buffers.append(buffer)
            print("Timeout waiting for frame")
        elif buffer not in frame.get_buffers():
            empty_frame_buffers.append(buffer)
            print(f"Buffer {buffer} not in frame")
        else:
            available_buffers.append(buffer)
            data = frame.get_buffer(buffer)
            print(data)
        fg.stop()
    except Exception as e:
        error_buffers.append(buffer)
        fg.stop()
        print(e)
    
print(f"Available buffers: {available_buffers}")
print(f"Timeout buffers: {timeout_buffers}")
print(f"Empty frame buffers: {empty_frame_buffers}")
print(f"Error buffers: {error_buffers}")
        