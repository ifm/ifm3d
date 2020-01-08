# Using ifm3d with the O3R B-Sample

## Configuring the Windows host
The O3R B-Sample is a USB device with Windows-only drivers. However, the data
is exposed over Ethernet using the same PCIC protocol used by other ifm
devices.

Thanks to this design, the Windows host driving the USB device can be
configured as and used as an IP camera natively on Linux, through standard
`ifm3d` and `ifm3d-ros` interfaces.

Setup:
1. Follow the windows `README.txt` that accompanies B-Sample software releases
to configure a target PC running the `o3r.exe` process.
2. Configure the PC's networking stack for a static IP in the subnet of
choice (in this guide we use an IP of `192.168.0.69` as the default, same as
the O3D and O3X).
3. Open the listening port (specified by `o3r.exe` -- 50010
by default, same as O3D and O3X) on the firwarwall.
4. [Optional]: Enable Remote Desktop. Allows for RDP access to start the
O3R.exe process on demand, rather than requiring a dedicated
keyboard/mouse/monitor on the Windows PC.
5. Run the `o3r.exe` process with desired parameters on the PC.

## Configuring the ifm3d Client (Linux or Windows)

Since the communication protocol on the O3R B-Sample follows the same PCIC
protocol as the O3D and O3R, the workflow is largely unchanged.

The primary difference is that the O3R B-Sample has no configuration store,
and so runtime discovery of device type does not work.

Using the custom `ifm3d` driver for the O3R (starting with version 0.100.0),
the device type can be explicity chosen by setting the `IFM3D_DEVICE`
environment variable:

```
$ export IFM3D_DEVICE=O3R
```

Otherwise, use the `ifm3d` and `ifm3d-ros` drivers as normal, pointing them at
the IP Address of the Windows host machine.