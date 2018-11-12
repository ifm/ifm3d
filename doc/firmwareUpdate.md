ifm3d - Firmware update Guide
=============================

you can use this guide to help you update the firmware of the ifm 3d cameras. This guide is divided in following sections

- [Obtaining the firmware](#Obtaining-the-firmware)
- [Uploading firmware to camera](#Uploading-firmware-to-camera)
- [swupdate command](#swupdate-command)
- [Script for updating the  firmware](#Script-for-updating-the-firmware)

# Obtaining the firmware
you can download the latest firmware from [**ifm camera firmwares**](https://www.ifm.com/ifmus/web/dualis-download.htm)

# Uploading firmware to camera

ifm3d provides command line tools swupdate and reboot to upload the firmware to the device. Before uploading the firmware we need to first put the camera in recovery mode.
Following are the steps to upload the firmware to the device

- Put the camera in recovery mode
```
$ifm3d --ip=<camera IP> reboot --r
```
successful execution of above command will put the device in recovery mode. device will accept firmware update only in this mode.

- uploading the firmware to the device
```
$ifm3d --ip=<camera IP> swupdate --file=<firmware file Name>
```
succesfull execution will upload the device firmware and reboot the device to the productive mode.

# swupdate command
swupdate command is used to update the firmware, for more details of the command use --help switch
```
$ifm3d swupdate --help
```
- checking the recovery mode
swupdate supports checking the wether device is in recovery mode or not, with --check switch
```
$ifm3d --ip=<camera IP> swupdate --check
```

-Reboot to Productive mode
swupdate provides --r flag to rebbot device from Recovery mode to Productive mode
```
$ifm3d --ip=<camera IP> swupdate --rebbot
```

#Script for updating the firmware
Following is the python script which will update the fimware on the device and put it back to produtive mode

```
from __future__ import print_function
import os
import subprocess
import time
import argparse

def converttosignedchar(value):
	if value == 0:
	   return 0;
	value =value & 0xff
	value = value - 2**8
	return value;

#argument parsing
parser = argparse.ArgumentParser()
parser.add_argument('--ip', '-ip', help="Device IP", type= str, default=os.environ.get("IFM3D_IP", "192.168.0.69"))
parser.add_argument('--file', '-f', help=" Firmware FileName", type= str)

args=parser.parse_args()

# file is needed
if args.file == "":
  print ("Firmware File needed")
  exit(1)

#creating ip switch
ip ="--ip="+args.ip

# creating function calls
functn_swupdate_check = ["ifm3d", ip ,"swupdate", "--check"]
functn_reboot_r = ["ifm3d", ip ,"reboot", "--r"]
functn_swupdate_file = ["ifm3d", ip ,"swupdate", "--file="+args.file]

count = 0
while True:
  #swupdate --check for checking the recovery mode of camera
  ret = subprocess.call(functn_swupdate_check)
  ret = converttosignedchar(ret)
  # on success
  if ret == 0:
    print ("Device is in Recovery Mode")
    break;
  # on failure due to productive mode
  elif ret == -1:
    print ("Device is in Productive Mode, Rebooting to Recovery Mode")
    # reboot -r to reboot device in the recovery mode
    subprocess.call(functn_reboot_r)
    time.sleep(20)
  # on failure due to IP address
  elif ret == -2:
    print ("Not able to connect to Device")
  count = count + 1;
  time.sleep(1)
  # if timeout occured
  if count == 20 :
    print ("Timeout occoured")
    exit(1)
# swupdate --file to upload firmware file to the device
ret = subprocess.call(functn_swupdate_file)
ret = converttosignedchar(ret)
if ret == 0:
  print ("Firmware update succesfull")
else :
  print ("Firmware update failled")
```
save the script as ifm_firmware_update.py and use following command to upload the firmware
```
$python ifm_firmware_update.py  --ip=<camera IP>  --file=<firmware file name>
e.g.
$python ifm_firmware_update.py  --ip="192.168.0.69" --file=O3D3XX_Firmware_1.23.1522.swu
```
Please not --file switch is compulsary and if --ip swith is not present it will connect to value of enviornment variable IFM3D_IP and if IFM3D_IP is laso not set then it will connect to default "192.168.0.69"