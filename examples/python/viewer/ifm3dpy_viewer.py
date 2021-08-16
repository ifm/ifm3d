#!/usr/bin/env python3

from ifm3dpy import O3RCamera, FrameGrabber, ImageBuffer
import cv2
import argparse

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ModuleNotFoundError:
    OPEN3D_AVAILABLE = False


def get_jpeg(buf):
    return cv2.imdecode(buf.jpeg_image(), cv2.IMREAD_UNCHANGED)


def get_distance(buf):
    img = cv2.normalize(buf.distance_image(), None, 0,
                        255, cv2.NORM_MINMAX, cv2.CV_8U)
    img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    return img


def get_amplitude(buf):
    return buf.amplitude_image()


def get_xyz(buf):
    return buf.xyz_image()


def display_2d(fg, buf, getter, title):
    cv2.startWindowThread()
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)

    while True:
        while not fg.wait_for_frame(buf, 500):
            continue

        img = getter(buf)

        cv2.imshow(title, img)
        cv2.waitKey(15)

        if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


def display_3d(fg, buf, getter, title):
    vis = o3d.visualization.Visualizer()
    vis.create_window(title)

    first = True

    while True:
        while not fg.wait_for_frame(buf, 500):
            continue

        img = getter(buf)

        img = img.reshape(img.shape[0]*img.shape[1], 3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(img)

        vis.clear_geometries()
        vis.add_geometry(pcd, first)
        if not vis.poll_events():
            break

        vis.update_renderer()

        first = False

    vis.destroy_window()


def main():
    image_choices = ["jpeg", "distance", "amplitude"]
    if OPEN3D_AVAILABLE:
        image_choices += ["xyz"]

    parser = argparse.ArgumentParser()
    parser.add_argument("--head", help="The Head from which images should be received", type=int,
                        choices=[0, 1, 2, 3, 4, 5], required=True)
    parser.add_argument("--image", help="The image to received (Only 3D heads) (default: distance)", type=str,
                        choices=image_choices, required=True)
    parser.add_argument("--ip", help="IP address of the sensor (default: 192.168.0.69)",
                        type=str, required=False, default="192.168.0.69")
    parser.add_argument("--xmlrpc-port", help="XMLRPC port of the sensor (default: 80)",
                        type=int, required=False, default=80)
    args = parser.parse_args()

    getter = globals()["get_" + args.image]

    cam = O3RCamera(args.ip, args.xmlrpc_port)
    fg = FrameGrabber(cam, pcic_port=50010 + args.head)
    buf = ImageBuffer()
    title = "O3R Head " + str(args.head)

    if args.image == "xyz":
        display_3d(fg, buf, getter, title)
    else:
        display_2d(fg, buf, getter, title)


if __name__ == "__main__":
    main()
