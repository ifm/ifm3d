NTP setup
=========

You can use this guide to help you start the NTP server on Windows and Linux OS and synchronize NTP client running on O3D3xx/O3X devices.

- [Enabling the Windows NTP Server](#Enabling-the-Windows-NTP-Server)
- [Enabling the Linux NTP Server](#Enabling-the-Linux-NTP-Server)
- [Enabling NTP client on the device with ifm3d tooling](#Enabling-NTP-client-on-the-device-with-ifm3d-tooling)
- [Establish a synchronization between client and server](#Establish-a-synchronization-between-client-and-server)

### Enabling the Windows NTP Server
To Enable the NTP server on the Windows, we need to make following changes to Windows
registry

#### Open the registry

```
> regedit
```
This will open the regedit UI

#### Change the time **server** type to NTP

```
HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Parameters\Type

Set value to NTP
```

#### Set **AnnounceFlags** to 5

```
HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\Config\AnnounceFlags

Set value to 5
```
#### Enable NTPServer

```
HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer\Enabled

set value to 1
```
#### Restaring the NTP server on the Windows machine

To stop the NTP servers
```
> net stop w32time
```
To start the NTP servers
```
> net start w32time
```
To update the configuration without restart of NTP server 
```
> w32tm /config /update
```

Note : Firewall or port filter blocks NTP packages. Make sure that firewall settings in Windows 
enable UDP protocol in both ways (inbound/outbound) on port 123

### Enabling the Linux NTP Server

This was tested on a Ubuntu 16.04 and a Raspbian installation. You have to
install the ntp server on your machine.

```
$ sudo apt-get install ntp
```

After installing the ntp server the configuration needs to be changed to fit
your network set-up.

```patch
--- etc/ntp.conf        2018-07-06 22:11:36.000000000 +0200
+++ /etc/ntp.conf       2018-11-07 13:58:37.128245106 +0100
@@ -15,13 +15,11 @@
 # Use servers from the NTP Pool Project. Approved by Ubuntu Technical Board
 # on 2011-02-08 (LP: #104525). See http://www.pool.ntp.org/join.html for
 # more information.
-pool 0.ubuntu.pool.ntp.org iburst
-pool 1.ubuntu.pool.ntp.org iburst
-pool 2.ubuntu.pool.ntp.org iburst
-pool 3.ubuntu.pool.ntp.org iburst
+server <your first NTP server> iburst
+server <your second NTP Server> iburst
+server <your third NTP server> iburst

 # Use Ubuntu's ntp server as a fallback.
-pool ntp.ubuntu.com

 # Access control configuration; see /usr/share/doc/ntp-doc/html/accopt.html for
 # details.  The web page
<http://support.ntp.org/bin/view/Support/AccessRestrictions>
@@ -32,8 +30,8 @@
 # up blocking replies from your own upstream servers.

 # By default, exchange time with everybody, but don't allow configuration.
-restrict -4 default kod notrap nomodify nopeer noquery limited
-restrict -6 default kod notrap nomodify nopeer noquery limited
+restrict -4 default kod notrap nomodify nopeer noquery
+restrict -6 default kod notrap nomodify nopeer noquery

 # Local users may interrogate the ntp server more closely.
 restrict 127.0.0.1
@@ -49,7 +47,7 @@

 # If you want to provide time to your local subnet, change the next line.
 # (Again, the address is an example only.)
-#broadcast 192.168.123.255
+broadcast 192.168.0.255

 # If you want to listen to time broadcasts on your local subnet, de-comment the
 # next lines.  Please do this only if you trust everybody on the network!
```

After configuration modification you have to restart the NTP server

```
$ sudo systemctl restart ntp
```

### Enabling NTP client on the device with ifm3d tooling

Enabling NTP client on device is all about setting the address of the NTP server
to the device for which user must set the parameter "NTPservers" on device.

#### Setting **NTPServers** on the device

```
> echo {"ifm3d":{"Time":{"NTPServers": "<Server-IP>"}}} | ifm3d --ip=<device-IP> config
```
This will set the IP of the NTP server on the device. user can add up to 5 servers IP
separated by comma or/and space.

### Establish a synchronization between client and server

Device will automatically start synchronization with the NTP servers on the power up if
the "SynchronizationActivated" value is "true"

#### Setting **SynchronizationActivated** to value "true"

```
> echo {"ifm3d":{"Time":{"SynchronizationActivated": "true"}}} | ifm3d --ip=<device-IP> config
```
To disable the client server synchronization set the "SynchronizationActivated" to "false".

### Check the configurations of device using dump command
```
ifm3d --ip=192.168.0.69 dump
...
  "Time": {
    "CurrentTime": "1541596487",
    "NTPServers": "192.168.0.20",
    "StartingSynchronization": "false",
    "Stats": "Server 192.168.0.20 current synced: +629ns ± 38us\n",
    "SynchronizationActivated": "true",
    "Syncing": "true",
    "WaitSyncTries": "2"
  },
...
```
This will give you all the configuration of device in JSON format. NTP parameters
are available in Time Node.
