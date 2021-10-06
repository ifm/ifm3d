# How to get started with the ifm3dpy

At the end of this 'how to', you should be able to receive images and know the basic usage of the O3RCamera(), FrameGrabber() and ImageBuffer(). This document is only scratching on the surface of the O3R and the ifm3dpy. Please refer to the other documentation for in depth information.

## First: install ifm3dpy

You can use the ifm3dpy PyPI package for an easy installation.

*NOTE: The ifm3d is also available for C++*


```python
!pip install ifm3dpy
```

    Looking in indexes: https://nexus.intra.ifm/repository/pypi_python_org/simple
    Requirement already satisfied: ifm3dpy in /home/devoegse/Git/ifm3d/venv/lib/python3.8/site-packages (0.91.0)


## Import ifm3d

Let's start with importing ifm3d (for python called ifm3dpy)  


```python
from ifm3dpy import O3RCamera, FrameGrabber, ImageBuffer
```

## Device, FrameGrabber and ImageBuffer
ifm3dTiny provides three objects:
- Device holds the configuration of the camera head, handles the connection and sets the camera in the proper state (RUN or CONF);  
- FrameGrabber receives frames (images);  
- ImageBuffer stores the image data.  

Instantiating these objects is as follows:


```python
IP = '192.168.0.69'
PORT = 50012
o3r = O3RCamera(IP, PORT)
fg = FrameGrabber(cam)
im = ImageBuffer()
```

## Receiving an image
You just need to call the function 'wait_for_frame()' from the FrameGrabber, provided with an image buffer and a timeout (in seconds). Be aware, that you can only receive an image if the connected head(s) are in "RUN" mode.


```python
timeout = 110 #ms
fg.wait_for_frame(im,timeout)
```




    False



Afterwards - expecting you received an image - you can call functions from the 'ImageBuffer()' to get to some of the data directly.


```python
amp = im.amplitude_image()
print(f"{amp[0][:10]} ... {amp[0][-10:]}")
```


    ---------------------------------------------------------------------------

    RuntimeError                              Traceback (most recent call last)

    /tmp/ipykernel_46778/969249777.py in <module>
    ----> 1 amp = im.amplitude_image()
          2 print(f"{amp[0][:10]} ... {amp[0][-10:]}")


    RuntimeError: cannot convert due to type or channel mistmatch


## Display the image

An easy way for displaying the image (without an viewer like the ifmVisionAssistant or RVIZ) is matplot.


```python
!pip install matplotlib
```

    Looking in indexes: https://nexus.intra.ifm/repository/pypi_python_org/simple
    Collecting matplotlib
      Using cached https://nexus.intra.ifm/repository/pypi_python_org/packages/matplotlib/3.4.3/matplotlib-3.4.3-cp38-cp38-manylinux1_x86_64.whl (10.3 MB)
    Collecting pillow>=6.2.0
      Using cached https://nexus.intra.ifm/repository/pypi_python_org/packages/pillow/8.3.2/Pillow-8.3.2-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl (3.0 MB)
    Collecting numpy>=1.16
      Using cached https://nexus.intra.ifm/repository/pypi_python_org/packages/numpy/1.21.2/numpy-1.21.2-cp38-cp38-manylinux_2_12_x86_64.manylinux2010_x86_64.whl (15.8 MB)
    Collecting cycler>=0.10
      Using cached https://nexus.intra.ifm/repository/pypi_python_org/packages/cycler/0.10.0/cycler-0.10.0-py2.py3-none-any.whl (6.5 kB)
    Collecting kiwisolver>=1.0.1
      Using cached https://nexus.intra.ifm/repository/pypi_python_org/packages/kiwisolver/1.3.2/kiwisolver-1.3.2-cp38-cp38-manylinux_2_5_x86_64.manylinux1_x86_64.whl (1.2 MB)
    Requirement already satisfied: python-dateutil>=2.7 in /home/devoegse/Git/ifm3d/venv/lib/python3.8/site-packages (from matplotlib) (2.8.2)
    Collecting pyparsing>=2.2.1
      Using cached https://nexus.intra.ifm/repository/pypi_python_org/packages/pyparsing/2.4.7/pyparsing-2.4.7-py2.py3-none-any.whl (67 kB)
    Requirement already satisfied: six in /home/devoegse/Git/ifm3d/venv/lib/python3.8/site-packages (from cycler>=0.10->matplotlib) (1.16.0)
    Installing collected packages: pyparsing, pillow, numpy, kiwisolver, cycler, matplotlib
    Successfully installed cycler-0.10.0 kiwisolver-1.3.2 matplotlib-3.4.3 numpy-1.21.2 pillow-8.3.2 pyparsing-2.4.7



```python
%matplotlib inline
import matplotlib.pyplot as plt

def showImage(img):
    plt.imshow(img)        
    plt.show()
```

We can now diplay the image easily with the above function.


```python
showImage(im.amplitude_image())
```

## Change configuration

The whole configuration of the O3R is done via json strings. You can use `ifm3dpy` and the `O3RCamera`-object to receive/forward this json configurations.


```python
config = o3r.get()
```


    ---------------------------------------------------------------------------

    RuntimeError                              Traceback (most recent call last)

    /tmp/ipykernel_46778/1406448996.py in <module>
    ----> 1 config = o3r.get()
    

    RuntimeError: Lib: XMLRPC Timeout - can you `ping' the sensor?


As an example, we change one parameter. In this case the exposure time (expLong/expShort).


```python
config['ports']['port2']['acquisition']['exposureLong'] = 2000
config['ports']['port2']['acquisition']['exposureShort'] = 200
```


    ---------------------------------------------------------------------------

    NameError                                 Traceback (most recent call last)

    /tmp/ipykernel_46778/3973814803.py in <module>
    ----> 1 config['ports']['port2']['acquisition']['exposureLong'] = 2000
          2 config['ports']['port2']['acquisition']['exposureShort'] = 200


    NameError: name 'config' is not defined



```python
o3r.set(config)
```


    ---------------------------------------------------------------------------

    NameError                                 Traceback (most recent call last)

    /tmp/ipykernel_46778/3382604224.py in <module>
    ----> 1 o3r.set(config)
    

    NameError: name 'config' is not defined



```python

```
