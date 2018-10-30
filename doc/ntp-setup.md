NTP setup
=========

You can use this guide to help you start the NTP server on Windows OS and synchronize
NTP client running on O3D3xx devices.

- [Enabling the Windows NTP Server](#Enabling-the-Windows-NTP-Server)
- [Enabling NTP client on the device with ifm3d tooling](#Enabling-NTP-client-on-the-device-with-ifm3d-tooling)
- [Establish a synchronization between client and server](#Establish-a-synchronization-between-client-and-server)

### Enabling the Windows NTP Server
To Enable the NTP server on the Windows, we need to make following changes to Windows
registry

 - Open the registry
```
> regedit
```
This will open the regedit UI

 - Change the time **server** type to NTP
```
HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Parameters\Type

Set value to NTP
```
 - Set **AnnounceFlags** to 5
```
HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config\AnnounceFlags

Set value to 5
```
 - Enable NTPServer
```
HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer\Enabled

set value to 1
```
 - Restaring the NTP server on the Windows machine
 
To stop the NTP servers
```
> net stop w32time
```
To start the NTP servers
```
> net start w32time
```

### Enabling NTP client on the device with ifm3d tooling

Enabling NTP client on device is all about setting the address of the NTP server
to the device for which user must set the parameter "NTPservers" on device.

- Setting **NTPServers** on the device
```
> echo {"ifm3d":{"Time":{"NTPServers": "<Server-IP>"}}} | ifm3d --ip=<device-IP> config

e.g

> echo {"ifm3d":{"Time":{"NTPServers": "192.168.0.70"}}} | ifm3d --ip=192.168.0.69 config
```
This will set the IP of the NTP server on device. user can add upto 5 servers IP
seperated by comma or/and space.

### Establish a synchronization between client and server

Device will automatically start synchronization with the NTP servers on the power up if
the "SynchronizationActivated" value is "true"

- Setting **SynchronizationActivated** to value "true"
```
> echo {"ifm3d":{"Time":{"SynchronizationActivated": "true"}}} | ifm3d --ip=<device-IP> config

e.g

> echo {"ifm3d":{"Time":{"SynchronizationActivated": "true"}}} | ifm3d --ip=192.168.0.69 config
```
To disable the client server synchronization set the "SynchronizationActivated" to "false". 

### Check the configurations of device using dump command
```
ifm3d --ip=192.168.0.69 dump
```
This will give you all the configuration of device in json format. NTP parameters
are available in Time Node.
